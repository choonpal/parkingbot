'''Fleet-owned parking slot registry with optional SQLite persistence.'''

from __future__ import annotations

from copy import deepcopy
from contextlib import closing
from dataclasses import dataclass, field, replace
from enum import Enum
import hashlib
import hmac
import json
import math
from pathlib import Path
import secrets
import sqlite3
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


class RegistryPersistenceError(RuntimeError):
    '''Parking Registry could not be durably stored or restored.'''


def registered_slots_fingerprint(registered_slots: Sequence[object]) -> str:
    '''Return a deterministic identity for registered slot geometry.'''
    descriptors = []
    for slot in registered_slots:
        try:
            descriptor = {
                'slot_id': str(slot.slot_id),
                'center_x_m': float(slot.center_x_m).hex(),
                'center_y_m': float(slot.center_y_m).hex(),
                'length_m': float(slot.length_m).hex(),
                'width_m': float(slot.width_m).hex(),
                'entry_yaw_rad': float(slot.entry_yaw_rad).hex(),
            }
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError('invalid registered slot geometry') from exc
        if not descriptor['slot_id']:
            raise ValueError('registered slot id must not be empty')
        descriptors.append(descriptor)
    if not descriptors:
        raise ValueError('registered_slots must not be empty')
    descriptors.sort(key=lambda item: item['slot_id'])
    if len({item['slot_id'] for item in descriptors}) != len(descriptors):
        raise ValueError('registered slot ids must be unique')
    payload = json.dumps(
        descriptors, ensure_ascii=False, sort_keys=True,
        separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(b'parking-layout-v1\0' + payload).hexdigest()


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


class _SQLiteParkingStore:
    '''Small transactional store behind the ParkingRegistry boundary.'''

    SCHEMA_VERSION = '1'

    def __init__(
            self, database_path: str, slot_ids: Sequence[str],
            layout_fingerprint: str):
        raw_path = str(database_path).strip()
        if not raw_path:
            raise ValueError('database_path must not be empty')
        fingerprint = str(layout_fingerprint).strip()
        if not fingerprint:
            raise ValueError(
                'layout_fingerprint is required for persistent Registry')
        self.path = Path(raw_path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._slot_ids = tuple(slot_ids)
        self._layout_fingerprint = fingerprint
        self._ensure_schema()

    def _connect(self):
        connection = sqlite3.connect(str(self.path), timeout=5.0)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA synchronous = FULL')
        return connection

    def _ensure_schema(self):
        try:
            with closing(self._connect()) as connection:
                existing_tables = {
                    row['name'] for row in connection.execute(
                        '''
                        SELECT name FROM sqlite_master
                        WHERE type='table' AND name NOT LIKE 'sqlite_%'
                        ''').fetchall()
                }
                expected_tables = {'registry_metadata', 'parking_slots'}
                if existing_tables and not expected_tables.issubset(
                        existing_tables):
                    raise RegistryPersistenceError(
                        'invalid Registry database schema')
                is_new_database = not existing_tables
                with connection:
                    connection.execute(
                        '''
                        CREATE TABLE IF NOT EXISTS registry_metadata (
                            key TEXT PRIMARY KEY,
                            value TEXT NOT NULL
                        )
                        ''')
                    connection.execute(
                        '''
                        CREATE TABLE IF NOT EXISTS parking_slots (
                            slot_id TEXT PRIMARY KEY,
                            lifecycle TEXT NOT NULL,
                            reservation_mission_id TEXT NOT NULL,
                            reservation_kind TEXT NOT NULL,
                            parked_by_mission_id TEXT NOT NULL,
                            pose_x REAL,
                            pose_y REAL,
                            pose_yaw REAL,
                            parking_direction TEXT NOT NULL,
                            vehicle_spec_json TEXT,
                            vehicle_number TEXT NOT NULL,
                            credential_iterations INTEGER,
                            credential_salt BLOB,
                            credential_digest BLOB
                        )
                        ''')
                    connection.execute(
                        '''
                        CREATE UNIQUE INDEX IF NOT EXISTS
                            unique_nonempty_vehicle_number
                        ON parking_slots(vehicle_number)
                        WHERE vehicle_number <> ''
                        ''')
                    if is_new_database:
                        connection.executemany(
                            '''
                            INSERT INTO registry_metadata(key, value)
                            VALUES(?, ?)
                            ''',
                            (
                                ('schema_version', self.SCHEMA_VERSION),
                                ('layout_fingerprint',
                                 self._layout_fingerprint),
                                ('slot_ids', json.dumps(
                                    self._slot_ids, ensure_ascii=False,
                                    separators=(',', ':'))),
                            ))
                        connection.executemany(
                            '''
                            INSERT INTO parking_slots(
                                slot_id, lifecycle, reservation_mission_id,
                                reservation_kind, parked_by_mission_id,
                                pose_x, pose_y, pose_yaw, parking_direction,
                                vehicle_spec_json, vehicle_number,
                                credential_iterations, credential_salt,
                                credential_digest
                            ) VALUES (?, 'EMPTY', '', '', '', NULL, NULL, NULL,
                                      '', NULL, '', NULL, NULL, NULL)
                            ''',
                            ((slot_id,) for slot_id in self._slot_ids))
                metadata = {
                    row['key']: row['value']
                    for row in connection.execute(
                        'SELECT key, value FROM registry_metadata').fetchall()
                }
                if metadata.get('schema_version') != self.SCHEMA_VERSION:
                    raise RegistryPersistenceError(
                        'invalid Registry database schema version')
                if metadata.get(
                        'layout_fingerprint') != self._layout_fingerprint:
                    raise RegistryPersistenceError(
                        'Registry database layout fingerprint mismatch')
                try:
                    stored_slot_ids = tuple(json.loads(metadata['slot_ids']))
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    raise RegistryPersistenceError(
                        'invalid Registry database slot metadata')
                if stored_slot_ids != self._slot_ids:
                    raise RegistryPersistenceError(
                        'Registry database slot set mismatch')
                stored_rows = tuple(
                    row['slot_id'] for row in connection.execute(
                        'SELECT slot_id FROM parking_slots '
                        'ORDER BY rowid').fetchall())
                if stored_rows != self._slot_ids:
                    raise RegistryPersistenceError(
                        'Registry database slot rows mismatch')
                self.path.chmod(0o600)
        except (OSError, sqlite3.Error) as exc:
            raise RegistryPersistenceError(
                f'failed to initialize Registry database: {exc}') from exc

    @staticmethod
    def _record_values(record: ParkingRecord):
        pose = record.final_vehicle_pose
        credential = record.credential
        spec_json = (
            None if record.vehicle_spec is None else
            json.dumps(
                record.vehicle_spec, ensure_ascii=False, sort_keys=True,
                separators=(',', ':')))
        return (
            record.slot_id,
            record.lifecycle.value,
            record.reservation_mission_id,
            record.reservation_kind,
            record.parked_by_mission_id,
            None if pose is None else pose.x_m,
            None if pose is None else pose.y_m,
            None if pose is None else pose.yaw_rad,
            record.parking_direction,
            spec_json,
            record.vehicle_number,
            None if credential is None else credential.iterations,
            None if credential is None else sqlite3.Binary(credential.salt),
            None if credential is None else sqlite3.Binary(credential.digest),
        )

    def save(self, record: ParkingRecord):
        try:
            values = self._record_values(record)
            with closing(self._connect()) as connection:
                with connection:
                    connection.execute(
                        '''
                        INSERT INTO parking_slots(
                            slot_id, lifecycle, reservation_mission_id,
                            reservation_kind, parked_by_mission_id,
                            pose_x, pose_y, pose_yaw, parking_direction,
                            vehicle_spec_json, vehicle_number,
                            credential_iterations, credential_salt,
                            credential_digest
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        ON CONFLICT(slot_id) DO UPDATE SET
                            lifecycle=excluded.lifecycle,
                            reservation_mission_id=
                                excluded.reservation_mission_id,
                            reservation_kind=excluded.reservation_kind,
                            parked_by_mission_id=
                                excluded.parked_by_mission_id,
                            pose_x=excluded.pose_x,
                            pose_y=excluded.pose_y,
                            pose_yaw=excluded.pose_yaw,
                            parking_direction=excluded.parking_direction,
                            vehicle_spec_json=excluded.vehicle_spec_json,
                            vehicle_number=excluded.vehicle_number,
                            credential_iterations=
                                excluded.credential_iterations,
                            credential_salt=excluded.credential_salt,
                            credential_digest=excluded.credential_digest
                        ''',
                        values)
        except (TypeError, ValueError, OSError, sqlite3.Error) as exc:
            raise RegistryPersistenceError(
                f'failed to save slot {record.slot_id}: {exc}') from exc

    @staticmethod
    def _row_to_record(row):
        pose_values = (row['pose_x'], row['pose_y'], row['pose_yaw'])
        pose = (
            None if all(value is None for value in pose_values)
            else Pose2D(*pose_values))
        spec = (
            None if row['vehicle_spec_json'] is None
            else json.loads(row['vehicle_spec_json']))
        credential_values = (
            row['credential_iterations'],
            row['credential_salt'],
            row['credential_digest'],
        )
        credential = (
            None if all(value is None for value in credential_values)
            else ParkingCredential(
                int(credential_values[0]),
                bytes(credential_values[1]),
                bytes(credential_values[2]),
            ))
        return ParkingRecord(
            slot_id=row['slot_id'],
            lifecycle=SlotLifecycle(row['lifecycle']),
            reservation_mission_id=row['reservation_mission_id'],
            reservation_kind=row['reservation_kind'],
            parked_by_mission_id=row['parked_by_mission_id'],
            final_vehicle_pose=pose,
            parking_direction=row['parking_direction'],
            vehicle_spec=spec,
            vehicle_number=row['vehicle_number'],
            credential=credential,
        )

    def load(self):
        try:
            with closing(self._connect()) as connection:
                placeholders = ','.join('?' for _ in self._slot_ids)
                rows = connection.execute(
                    f'''
                    SELECT * FROM parking_slots
                    WHERE slot_id IN ({placeholders})
                    ''',
                    self._slot_ids).fetchall()
            return {
                row['slot_id']: self._row_to_record(row)
                for row in rows
            }
        except (
                KeyError, TypeError, ValueError, OSError, json.JSONDecodeError,
                sqlite3.Error) as exc:
            raise RegistryPersistenceError(
                f'failed to load Registry database: {exc}') from exc


class ParkingRegistry:
    '''Own slot lifecycle and vehicle records, optionally backed by SQLite.'''

    def __init__(
            self, slot_ids: Sequence[str], database_path: Optional[str] = None,
            layout_fingerprint: str = ''):
        ordered = tuple(str(value).strip() for value in slot_ids)
        if not ordered or any(not value for value in ordered):
            raise ValueError('slot_ids must contain non-empty values')
        if len(set(ordered)) != len(ordered):
            raise ValueError('slot_ids must be unique')
        self._slot_ids = ordered
        self._store = None
        if database_path is None:
            self._records = {
                slot_id: ParkingRecord(slot_id=slot_id)
                for slot_id in ordered
            }
        else:
            self._store = _SQLiteParkingStore(
                database_path, ordered, layout_fingerprint)
            self._records = self._store.load()
            if set(self._records) != set(ordered):
                raise RegistryPersistenceError(
                    'Registry database does not contain every configured slot')
            self._validate_restored_records()

    def _validate_restored_records(self):
        for slot_id in self._slot_ids:
            record = self._records[slot_id]
            if record.lifecycle not in (
                    SlotLifecycle.EMPTY, SlotLifecycle.OCCUPIED):
                raise RegistryPersistenceError(
                    f'{slot_id}: unfinished mission state '
                    f'{record.lifecycle.value} requires operator recovery')
            if record.lifecycle is SlotLifecycle.EMPTY:
                if record != ParkingRecord(slot_id=slot_id):
                    raise RegistryPersistenceError(
                        f'{slot_id}: invalid EMPTY Registry record')
                continue
            if (
                    record.reservation_mission_id or record.reservation_kind or
                    not record.parked_by_mission_id or
                    record.final_vehicle_pose is None or
                    record.parking_direction not in (
                        'forward', 'reverse', 'unknown') or
                    record.vehicle_spec is None or
                    bool(record.vehicle_number) != (
                        record.credential is not None)):
                raise RegistryPersistenceError(
                    f'{slot_id}: invalid OCCUPIED Registry record')
            try:
                normalized_number = (
                    '' if not record.vehicle_number else
                    normalize_vehicle_number(record.vehicle_number))
                spec = self._validated_spec(record.vehicle_spec)
            except (RegistryTransitionError, ValueError) as exc:
                raise RegistryPersistenceError(
                    f'{slot_id}: invalid OCCUPIED Registry record') from exc
            if normalized_number != record.vehicle_number:
                raise RegistryPersistenceError(
                    f'{slot_id}: invalid normalized vehicle number')
            credential = record.credential
            if credential is not None and (
                    credential.iterations <= 0 or not credential.salt or
                    not credential.digest):
                raise RegistryPersistenceError(
                    f'{slot_id}: invalid parking credential')
            self._records[slot_id] = replace(record, vehicle_spec=spec)

    def _replace_record(self, record: ParkingRecord):
        if self._store is not None:
            self._store.save(record)
        self._records[record.slot_id] = record

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
        self._replace_record(ParkingRecord(
            slot_id=record.slot_id,
            lifecycle=SlotLifecycle.RESERVED,
            reservation_mission_id=mission_id,
            reservation_kind='park',
            vehicle_number=normalized_number,
            credential=credential,
        ))

    def rollback_unpublished_park(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.RESERVED, mission_id, 'park')
        self._replace_record(ParkingRecord(slot_id=record.slot_id))

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
        self._replace_record(ParkingRecord(
            slot_id=record.slot_id,
            lifecycle=SlotLifecycle.OCCUPIED,
            parked_by_mission_id=mission_id,
            final_vehicle_pose=final_vehicle_pose,
            parking_direction=direction,
            vehicle_spec=spec,
            vehicle_number=record.vehicle_number,
            credential=record.credential,
        ))

    def reserve_retrieve(
            self, slot_id: str, mission_id: str) -> ParkingRecord:
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(record, SlotLifecycle.OCCUPIED)
        if record.final_vehicle_pose is None or record.vehicle_spec is None:
            raise RegistryTransitionError(
                f'{record.slot_id}: missing vehicle record')
        self._replace_record(replace(
            record,
            lifecycle=SlotLifecycle.EXIT_RESERVED,
            reservation_mission_id=mission_id,
            reservation_kind='retrieve',
        ))
        return self.get(record.slot_id)

    def mark_retrieve_exiting(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.EXIT_RESERVED, mission_id, 'retrieve')
        self._replace_record(replace(
            record, lifecycle=SlotLifecycle.EXITING))

    def complete_retrieve(self, slot_id: str, mission_id: str):
        mission_id = self._mission_id(mission_id)
        record = self._require(slot_id)
        self._require_state(
            record, SlotLifecycle.EXITING, mission_id, 'retrieve')
        self._replace_record(ParkingRecord(slot_id=record.slot_id))

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
