'''Fleet-owned, process-lifetime parking slot registry.'''

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import hmac
import math
import secrets
from typing import Mapping, Optional, Sequence

from cooperative_parking_robot.parking_geometry import Pose2D


class SlotLifecycle(str, Enum):
    EMPTY = 'EMPTY'
    RESERVED = 'RESERVED'
    OCCUPIED = 'OCCUPIED'
    EXIT_RESERVED = 'EXIT_RESERVED'
    EXITING = 'EXITING'


class RegistryTransitionError(ValueError):
    '''Lifecycle state or mission binding is invalid.'''


def normalize_vehicle_number(value: str) -> str:
    '''Return the session vehicle identifier used for Registry matching.'''
    if not isinstance(value, str):
        raise ValueError('vehicle_number must be a string')
    normalized = ''.join(value.split()).upper()
    if not normalized or len(normalized) > 32:
        raise ValueError('vehicle_number must contain 1 to 32 characters')
    return normalized


def validate_parking_password(value: str) -> str:
    '''Validate a password without normalizing or retaining a second copy.'''
    if not isinstance(value, str):
        raise ValueError('password must be a string')
    if not 4 <= len(value) <= 64 or len(value.encode('utf-8')) > 256:
        raise ValueError('password must contain 4 to 64 characters')
    return value


@dataclass(frozen=True)
class ParkingCredential:
    '''Salted password verifier; the plaintext password is never retained.'''

    iterations: int
    salt: bytes = field(repr=False)
    digest: bytes = field(repr=False)

    ALGORITHM = 'sha256'
    DEFAULT_ITERATIONS = 200_000
    SALT_BYTES = 16

    @classmethod
    def create(cls, password: str) -> 'ParkingCredential':
        password = validate_parking_password(password)
        salt = secrets.token_bytes(cls.SALT_BYTES)
        digest = hashlib.pbkdf2_hmac(
            cls.ALGORITHM,
            password.encode('utf-8'),
            salt,
            cls.DEFAULT_ITERATIONS,
        )
        return cls(cls.DEFAULT_ITERATIONS, salt, digest)

    def verify(self, password: str) -> bool:
        try:
            password = validate_parking_password(password)
        except ValueError:
            return False
        candidate = hashlib.pbkdf2_hmac(
            self.ALGORITHM,
            password.encode('utf-8'),
            self.salt,
            self.iterations,
        )
        return hmac.compare_digest(candidate, self.digest)


@dataclass(frozen=True)
class ParkingRecord:
    slot_id: str
    lifecycle: SlotLifecycle = SlotLifecycle.EMPTY
    reservation_mission_id: str = ''
    reservation_kind: str = ''
    parked_by_mission_id: str = ''
    final_vehicle_pose: Optional[Pose2D] = None
    parking_direction: str = ''
    vehicle_spec: Optional[dict] = None
    vehicle_number: str = ''
    credential: Optional[ParkingCredential] = field(
        default=None, repr=False)


class ParkingRegistry:
    '''Own session-scoped slot lifecycle and vehicle records.'''

    def __init__(self, slot_ids: Sequence[str]):
        ordered = tuple(str(value).strip() for value in slot_ids)
        if not ordered or any(not value for value in ordered):
            raise ValueError('slot_ids must contain non-empty values')
        if len(set(ordered)) != len(ordered):
            raise ValueError('slot_ids must be unique')
        self._slot_ids = ordered
        self._records = {
            slot_id: ParkingRecord(slot_id=slot_id)
            for slot_id in ordered
        }

    def _require(self, slot_id: str) -> ParkingRecord:
        key = str(slot_id)
        if key not in self._records:
            raise KeyError(key)
        return self._records[key]

    @staticmethod
    def _mission_id(value: str) -> str:
        mission_id = str(value).strip()
        if not mission_id:
            raise RegistryTransitionError('mission_id must not be empty')
        return mission_id

    @staticmethod
    def _require_state(record, lifecycle, mission_id=None, kind=None):
        if record.lifecycle is not lifecycle:
            raise RegistryTransitionError(
                f'{record.slot_id}: expected {lifecycle.value}, '
                f'got {record.lifecycle.value}')
        if (mission_id is not None and
                record.reservation_mission_id != mission_id):
            raise RegistryTransitionError(
                f'{record.slot_id}: reservation mission mismatch')
        if kind is not None and record.reservation_kind != kind:
            raise RegistryTransitionError(
                f'{record.slot_id}: reservation kind mismatch')

    @staticmethod
    def _validated_spec(vehicle_spec: Mapping) -> dict:
        if not isinstance(vehicle_spec, Mapping):
            raise RegistryTransitionError('vehicle_spec must be a mapping')
        result = deepcopy(dict(vehicle_spec))
        for key in ('wheelbase', 'vehicle_length_m', 'vehicle_width_m'):
            try:
                value = float(result[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise RegistryTransitionError(
                    f'vehicle_spec missing/invalid {key}') from exc
            if not math.isfinite(value) or value <= 0.0:
                raise RegistryTransitionError(
                    f'vehicle_spec invalid {key}')
            result[key] = value
        return result

    def get(self, slot_id: str) -> ParkingRecord:
        record = self._require(slot_id)
        return replace(
            record,
            vehicle_spec=(None if record.vehicle_spec is None
                          else deepcopy(record.vehicle_spec)))

    def lifecycle(self, slot_id: str) -> SlotLifecycle:
        return self._require(slot_id).lifecycle

    def empty_slot_ids(self):
        return tuple(
            slot_id for slot_id in self._slot_ids
            if self._records[slot_id].lifecycle is SlotLifecycle.EMPTY)

    def find_by_vehicle_number(self, vehicle_number: str):
        try:
            normalized = normalize_vehicle_number(vehicle_number)
        except ValueError:
            return None
        for slot_id in self._slot_ids:
            record = self._records[slot_id]
            if (record.lifecycle is not SlotLifecycle.EMPTY and
                    record.vehicle_number == normalized):
                return self.get(slot_id)
        return None

    def authenticate_vehicle(self, vehicle_number: str, password: str):
        '''Return the occupied record only when identifier and secret match.'''
        record = self.find_by_vehicle_number(vehicle_number)
        if (record is None or record.lifecycle is not SlotLifecycle.OCCUPIED or
                record.credential is None or
                not record.credential.verify(password)):
            return None
        return record

    def reserve_park(
            self, slot_id: str, mission_id: str, vehicle_number: str = '',
            credential: Optional[ParkingCredential] = None):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(record, SlotLifecycle.EMPTY)
        if bool(vehicle_number) != (credential is not None):
            raise RegistryTransitionError(
                'vehicle_number and credential must be provided together')
        normalized_number = ''
        if vehicle_number:
            try:
                normalized_number = normalize_vehicle_number(vehicle_number)
            except ValueError as exc:
                raise RegistryTransitionError(str(exc)) from exc
            if not isinstance(credential, ParkingCredential):
                raise RegistryTransitionError(
                    'credential must be ParkingCredential')
            if any(
                    existing.lifecycle is not SlotLifecycle.EMPTY and
                    existing.vehicle_number == normalized_number
                    for existing in self._records.values()):
                raise RegistryTransitionError(
                    'vehicle_number is already registered')
        self._records[record.slot_id] = ParkingRecord(
            slot_id=record.slot_id,
            lifecycle=SlotLifecycle.RESERVED,
            reservation_mission_id=mission_id,
            reservation_kind='park',
            vehicle_number=normalized_number,
            credential=credential,
        )

    def rollback_unpublished_park(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.RESERVED, mission_id, 'park')
        self._records[record.slot_id] = ParkingRecord(slot_id=record.slot_id)

    def complete_park(
            self, slot_id: str, mission_id: str, final_vehicle_pose: Pose2D,
            parking_direction: str, vehicle_spec: Mapping):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.RESERVED, mission_id, 'park')
        if not isinstance(final_vehicle_pose, Pose2D):
            raise RegistryTransitionError(
                'final_vehicle_pose must be Pose2D')
        direction = str(parking_direction).strip().lower()
        if direction not in ('forward', 'reverse', 'unknown'):
            raise RegistryTransitionError('invalid parking_direction')
        spec = self._validated_spec(vehicle_spec)
        self._records[record.slot_id] = ParkingRecord(
            slot_id=record.slot_id,
            lifecycle=SlotLifecycle.OCCUPIED,
            parked_by_mission_id=mission_id,
            final_vehicle_pose=final_vehicle_pose,
            parking_direction=direction,
            vehicle_spec=spec,
            vehicle_number=record.vehicle_number,
            credential=record.credential,
        )

    def reserve_retrieve(
            self, slot_id: str, mission_id: str) -> ParkingRecord:
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(record, SlotLifecycle.OCCUPIED)
        if record.final_vehicle_pose is None or record.vehicle_spec is None:
            raise RegistryTransitionError(
                f'{record.slot_id}: missing vehicle record')
        self._records[record.slot_id] = replace(
            record,
            lifecycle=SlotLifecycle.EXIT_RESERVED,
            reservation_mission_id=mission_id,
            reservation_kind='retrieve',
        )
        return self.get(record.slot_id)

    def mark_retrieve_exiting(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.EXIT_RESERVED, mission_id, 'retrieve')
        self._records[record.slot_id] = replace(
            record, lifecycle=SlotLifecycle.EXITING)

    def complete_retrieve(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.EXITING, mission_id, 'retrieve')
        self._records[record.slot_id] = ParkingRecord(slot_id=record.slot_id)

    def summaries(self, supported_directions=('forward',)):
        supported = {
            str(value).strip().lower() for value in supported_directions}
        result = []
        for slot_id in self._slot_ids:
            record = self._records[slot_id]
            result.append({
                'slot_id': slot_id,
                'lifecycle': record.lifecycle.value,
                'retrievable': (
                    record.lifecycle is SlotLifecycle.OCCUPIED and
                    record.parking_direction in supported and
                    record.final_vehicle_pose is not None and
                    record.vehicle_spec is not None and
                    bool(record.vehicle_number) and
                    record.credential is not None),
            })
        return result
