#!/usr/bin/env python3
"""Per-robot mission state machine with mission-scoped two-phase barriers."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from cooperative_parking_robot.latest_qos import (
    SAFETY_STATE_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.mission_protocol import parse_arrival_status
from cooperative_parking_robot.sync_faults import is_fatal_sync_error


class RobotStateMachineNode(Node):
    def __init__(self, **kwargs):
        super().__init__("robot_state_machine_node", **kwargs)
        self.declare_parameter("role", "front")
        self.declare_parameter("require_hardware_ready", True)
        self.declare_parameter("approach_timeout_s", 150.0)
        self.declare_parameter("align_timeout_s", 120.0)
        self.declare_parameter("drive_timeout_s", 120.0)
        self.declare_parameter("return_timeout_s", 90.0)
        self.declare_parameter("coordination_timeout_s", 1.5)
        self.declare_parameter("fleet_timeout_s", 2.5)
        self.declare_parameter("future_tolerance_s", 0.25)
        self.declare_parameter("stop_after_align", False)

        gp = self.get_parameter
        self.role = str(gp("role").value)
        if self.role not in ("front", "rear"):
            raise ValueError("role must be 'front' or 'rear'")
        self.is_rear = self.role == "rear"
        self.other_role = "front" if self.is_rear else "rear"
        self.require_hardware_ready = bool(
            gp("require_hardware_ready").value)
        self.approach_timeout = float(gp("approach_timeout_s").value)
        self.align_timeout = float(gp("align_timeout_s").value)
        self.drive_timeout = float(gp("drive_timeout_s").value)
        self.return_timeout = float(gp("return_timeout_s").value)
        self.coordination_timeout = float(
            gp("coordination_timeout_s").value)
        self.fleet_timeout = float(gp("fleet_timeout_s").value)
        self.future_tolerance = float(gp("future_tolerance_s").value)
        self.stop_after_align = bool(gp("stop_after_align").value)
        if any(value <= 0.0 for value in (
                self.approach_timeout, self.align_timeout,
                self.drive_timeout, self.return_timeout,
                self.coordination_timeout, self.fleet_timeout)):
            raise ValueError("state timeouts must be positive")
        if self.future_tolerance < 0.0:
            raise ValueError("future_tolerance_s must be non-negative")

        self.state = "IDLE"
        self.enter_time = time.monotonic()
        self.wheel_aligned = False
        self.fleet_state = "WAIT_TARGET"
        self.lift_done = False
        self.release_done = False
        self.other_align_done = False
        self.alignment_announced = False
        self.aligned_hold = False
        self.other_release_done = False
        self.release_announced = False
        self.arrived = False
        self.approach_done = False
        self.return_done = False
        self.hardware_ready = False
        self.hardware_fault = None
        self.local_lifted = False
        self.aggregate_lifted = False
        self.last_action_time = 0.0
        self.rear_lifted = False
        self.self_lifted = False
        self.active_mission_id = ""
        self.active_plan_stamp_ns = 0
        self.fleet_sequence = -1
        self.fleet_receipt_time = 0.0
        self.local_ready_stage = None
        self.other_ready_stage = None
        self.other_ready_sequence = -1
        self.other_ready_receipt_time = 0.0
        self.ready_sequence = 0
        self.commit_sequence = 0
        self.last_commit_sequence = -1
        self.last_commit_publish = 0.0
        self.committed_stages = set()
        self.coordination_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            Bool, f"/{self.role}/wheel_aligned", self.aligned_cb, 10)
        self.create_subscription(
            String, "/fleet/state", self.fleet_cb, STATE_LATEST_QOS)
        self.create_subscription(
            String, f"/{self.role}/lift_status", self.lift_cb, 10)
        self.create_subscription(
            String, f"/{self.role}/hardware_status", self.hardware_cb,
            SAFETY_STATE_QOS)
        self.create_subscription(
            Bool, f"/{self.role}/hardware_ready",
            self.hardware_ready_cb, SAFETY_STATE_QOS)
        self.create_subscription(
            String, "/sync/error_state", self.sync_cb, 10)
        self.create_subscription(
            Bool, f"/{self.role}/approach_done", self.approach_cb, 10)
        self.create_subscription(
            Bool, f"/{self.role}/return_done", self.return_cb, 10)
        self.create_subscription(
            String, f"/{self.role}/motion_fault",
            self.motion_fault_cb, 10)
        self.create_subscription(
            Bool, f"/align/{self.other_role}_done",
            self.other_done_cb, 10)
        self.create_subscription(
            Bool, f"/release/{self.other_role}_done",
            self.other_release_cb, 10)
        self.create_subscription(
            String, f"/mission/{self.other_role}/ready",
            self.other_ready_cb, self.coordination_qos)
        self.create_subscription(
            String, "/mission/commit", self.commit_cb,
            self.coordination_qos)

        self.pub_state = self.create_publisher(
            String, f"/{self.role}/robot_state", STATE_LATEST_QOS)
        self.pub_grip = self.create_publisher(
            String, f"/{self.role}/grip_command", 10)
        self.pub_lifted = self.create_publisher(
            Bool, f"/{self.role}/lifted", 10)
        self.pub_align_done = self.create_publisher(
            Bool, f"/align/{self.role}_done", 10)
        self.pub_aligned_hold = self.create_publisher(
            Bool, f"/{self.role}/aligned_hold", STATE_LATEST_QOS)
        self.pub_release_done = self.create_publisher(
            Bool, f"/release/{self.role}_done", 10)
        self.pub_ready = self.create_publisher(
            String, f"/mission/{self.role}/ready",
            self.coordination_qos)
        self.pub_estop = self.create_publisher(
            Bool, "/emergency_stop", SAFETY_STATE_QOS)
        self.pub_commit = None
        self.pub_mission_complete = None
        if not self.is_rear:
            self.pub_commit = self.create_publisher(
                String, "/mission/commit", self.coordination_qos)
            # P3: 임무 종료 통지. Front가 이미 commit 발행자(조정 권한자)이므로
            # 완료 선언도 같은 노드가 맡는다. transient-local을 쓰지 않는 이유는
            # 나중에 기동한 노드가 과거 완료를 새 완료로 오인하면 안 되기 때문이다.
            self.pub_mission_complete = self.create_publisher(
                String, "/mission/complete", 10)
            self.pub_lifted_all = self.create_publisher(
                Bool, "/robot/lifted", 10)
            self.create_subscription(
                Bool, "/rear/lifted", self.rear_lifted_cb, 10)

        self.create_timer(0.1, self.state_machine)
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info(
            f"robot_state_machine [{self.role}] | "
            f"timeouts=approach:{self.approach_timeout:.1f}s/"
            f"align:{self.align_timeout:.1f}s/"
            f"drive:{self.drive_timeout:.1f}s/"
            f"return:{self.return_timeout:.1f}s | "
            f"coordination_timeout={self.coordination_timeout:.1f}s | "
            f"stop_after_align={self.stop_after_align}")

    def aligned_cb(self, msg):
        if msg.data:
            self.wheel_aligned = True

    def fleet_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            fleet_state = str(payload.get("state", "WAIT_TARGET"))
            mission_id = str(payload.get("mission_id", ""))
            plan_stamp_ns = int(payload.get("plan_stamp_ns", 0))
            sequence = int(payload["sequence"])
            stamp_ns = int(payload["stamp_ns"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn(
                f"[{self.role}] invalid fleet/state envelope",
                throttle_duration_sec=2.0)
            return
        if not self.source_stamp_is_fresh(stamp_ns, self.fleet_timeout):
            self.get_logger().warn(
                f"[{self.role}] stale/future fleet/state rejected",
                throttle_duration_sec=2.0)
            return
        if sequence <= self.fleet_sequence:
            return
        if (mission_id and self.active_mission_id and
                mission_id != self.active_mission_id and
                self.state not in ("IDLE", "FAULT")):
            self.hardware_fault = "MISSION_CHANGED_WHILE_ACTIVE"
            return
        if (not self.active_mission_id and mission_id and
                fleet_state == "WAIT_LIFT" and self.state == "IDLE"):
            self.active_mission_id = mission_id
            self.clear_coordination()
            self.get_logger().info(
                f"[{self.role}] accepted mission {mission_id}")
        self.fleet_state = fleet_state
        self.active_plan_stamp_ns = plan_stamp_ns
        self.fleet_sequence = sequence
        self.fleet_receipt_time = time.monotonic()

    def lift_cb(self, msg):
        if msg.data == "GRIP_DONE":
            self.lift_done = True
        elif msg.data == "RELEASE_DONE":
            self.release_done = True

    def other_done_cb(self, msg):
        self.other_align_done = bool(msg.data)

    def other_release_cb(self, msg):
        self.other_release_done = bool(msg.data)

    def source_stamp_is_fresh(self, stamp_ns, max_age_s):
        if stamp_ns <= 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - stamp_ns) * 1e-9
        return -self.future_tolerance <= age_s <= max_age_s

    def decode_coordination_event(self, msg, expected_role):
        try:
            payload = json.loads(msg.data)
            mission_id = str(payload["mission_id"])
            role = str(payload["role"])
            stage = str(payload["stage"])
            sequence = int(payload["sequence"])
            stamp_ns = int(payload["stamp_ns"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None
        if not self.active_mission_id or mission_id != self.active_mission_id:
            return None
        if role != expected_role:
            return None
        if stage not in ("LIFT", "DRIVE", "RELEASE", "RETURN", "HOME"):
            return None
        if not self.source_stamp_is_fresh(
                stamp_ns, self.coordination_timeout):
            return None
        return {
            "stage": stage,
            "sequence": sequence,
        }

    def other_ready_cb(self, msg):
        event = self.decode_coordination_event(msg, self.other_role)
        if event is None or event["sequence"] <= self.other_ready_sequence:
            return
        self.other_ready_sequence = event["sequence"]
        self.other_ready_stage = event["stage"]
        self.other_ready_receipt_time = time.monotonic()
        if (not self.is_rear and self.other_role == "rear" and
                self.other_ready_stage == "DRIVE"):
            self.rear_lifted = True
            self.check_both_lifted()

    def commit_cb(self, msg):
        event = self.decode_coordination_event(msg, "front")
        if event is None or event["sequence"] <= self.last_commit_sequence:
            return
        if self.stop_after_align and event["stage"] == "LIFT":
            self.get_logger().warn(
                f"[{self.role}] ignored LIFT commit in stop_after_align mode",
                throttle_duration_sec=2.0)
            return
        self.last_commit_sequence = event["sequence"]
        self.committed_stages.add(event["stage"])

    def hardware_ready_cb(self, msg):
        was_ready = self.hardware_ready
        self.hardware_ready = bool(msg.data)
        if (self.require_hardware_ready and was_ready and
                not self.hardware_ready and
                self.state not in ("IDLE", "FAULT")):
            self.hardware_fault = "HARDWARE_ACK_TIMEOUT"
            self.get_logger().error(
                f"[{self.role}] STM32 ACK lost")

    def hardware_cb(self, msg):
        if not msg.data.startswith(("ERR,", "ESTOP")):
            return
        if "LIFT_WHILE_MOVING" in msg.data:
            self.get_logger().warn(
                f"[{self.role}] {msg.data}; retry after stop")
            return
        self.hardware_fault = msg.data
        self.get_logger().error(
            f"[{self.role}] hardware fault: {msg.data}")

    def approach_cb(self, msg):
        if msg.data:
            self.approach_done = True

    def return_cb(self, msg):
        if msg.data:
            self.return_done = True

    def motion_fault_cb(self, msg):
        reason = str(msg.data).strip()
        if reason and self.state not in ("IDLE", "FAULT"):
            self.hardware_fault = f"MOTION,{reason}"
            self.get_logger().error(
                f"[{self.role}] individual motion fault: {reason}")

    def sync_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            error = str(payload.get("error", "OK"))
            plan_stamp_ns = int(payload.get("plan_stamp_ns", 0))
        except (TypeError, ValueError, json.JSONDecodeError, AttributeError):
            return
        if error == "ARRIVED":
            if (self.state == "DRIVE" and self.active_plan_stamp_ns > 0 and
                    plan_stamp_ns == self.active_plan_stamp_ns and
                    parse_arrival_status(
                        payload, self.active_plan_stamp_ns) is not None):
                self.arrived = True
            return
        if is_fatal_sync_error(error):
            fault = f"SYNC,{error}"
            if self.hardware_fault != fault:
                self.hardware_fault = fault
                self.get_logger().error(
                    f"[{self.role}] sync fault: {error}")

    def transition(self, new):
        self.get_logger().info(
            f"[{self.role}] {self.state} -> {new}")
        self.state = new
        if new != "ALIGN":
            self.aligned_hold = False
        self.enter_time = time.monotonic()
        self.last_action_time = 0.0
        self.local_ready_stage = None
        self.pub_state.publish(String(data=new))

    def elapsed(self):
        return time.monotonic() - self.enter_time

    def clear_coordination(self):
        self.local_ready_stage = None
        self.other_ready_stage = None
        self.other_ready_sequence = -1
        self.other_ready_receipt_time = 0.0
        self.ready_sequence = 0
        self.commit_sequence = 0
        self.last_commit_sequence = -1
        self.last_commit_publish = 0.0
        self.committed_stages.clear()

    def publish_ready_stage(self, stage):
        if not self.active_mission_id:
            return
        self.local_ready_stage = stage
        self.ready_sequence += 1
        payload = {
            "mission_id": self.active_mission_id,
            "role": self.role,
            "stage": stage,
            "sequence": self.ready_sequence,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self.pub_ready.publish(String(data=json.dumps(payload)))

    def other_ready_is_fresh(self, stage):
        return (
            self.other_ready_stage == stage and
            time.monotonic() - self.other_ready_receipt_time <=
            self.coordination_timeout)

    def maybe_publish_commit(self, stage):
        if self.is_rear or self.pub_commit is None:
            return
        if stage in self.committed_stages:
            return
        if (self.local_ready_stage != stage or
                not self.other_ready_is_fresh(stage)):
            return
        now = time.monotonic()
        if now - self.last_commit_publish < 0.2:
            return
        self.last_commit_publish = now
        self.commit_sequence += 1
        payload = {
            "mission_id": self.active_mission_id,
            "role": "front",
            "stage": stage,
            "sequence": self.commit_sequence,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self.pub_commit.publish(String(data=json.dumps(payload)))
        self.committed_stages.add(stage)
        self.get_logger().info(
            f"[front] mission commit {stage} ({self.active_mission_id})")

    def state_machine(self):
        if (self.active_mission_id and
                self.state not in ("IDLE", "FAULT") and
                time.monotonic() - self.fleet_receipt_time >
                self.fleet_timeout):
            self.hardware_fault = "FLEET_STATE_TIMEOUT"
        if self.hardware_fault and self.state != "FAULT":
            # HELLO 전 previous-session timeout은 bridge가 INFO로 격리한다.
            # 여기 도달한 실제 ERR/ESTOP은 IDLE 중이어도 current fault다.
            # Communication loss is already fail-closed by bridge velocity
            # suppression and the MCU's 250 ms command watchdog.  Latching the
            # physical ESTOP here would prevent the new HELLO session needed
            # to diagnose/recover communication.  Mission state still latches
            # FAULT and therefore cannot resume motion automatically.
            if self._fault_requires_estop(self.hardware_fault):
                self.pub_estop.publish(Bool(data=True))
            self.transition("FAULT")

        if self.state == "IDLE":
            hardware_ok = (
                self.hardware_ready or not self.require_hardware_ready)
            if (self.fleet_state == "WAIT_LIFT" and hardware_ok and
                    self.active_mission_id):
                self.transition("APPROACH")

        elif self.state == "APPROACH":
            if self.approach_done:
                self.transition("ALIGN")
            elif self.elapsed() > self.approach_timeout:
                self.fail("APPROACH_TIMEOUT")

        elif self.state == "ALIGN":
            if self.wheel_aligned:
                self.alignment_announced = True
            if self.alignment_announced:
                self.pub_align_done.publish(Bool(data=True))
                if self.stop_after_align:
                    if not self.aligned_hold:
                        self.get_logger().warn(
                            f"[{self.role}] axle aligned; holding at zero "
                            "with grip/lift disabled")
                    self.aligned_hold = True
                    self.pub_aligned_hold.publish(Bool(data=True))
                    return
                self.publish_ready_stage("LIFT")
                self.maybe_publish_commit("LIFT")
            if "LIFT" in self.committed_stages:
                self.transition("LIFT")
            elif self.elapsed() > self.align_timeout:
                self.fail("ALIGN_TIMEOUT")

        elif self.state == "LIFT":
            if not self.lift_done:
                self.send_action_with_retry("grip")
            if self.lift_done:
                if not self.local_lifted:
                    self.publish_lifted()
                self.publish_ready_stage("DRIVE")
                self.maybe_publish_commit("DRIVE")
            if "DRIVE" in self.committed_stages:
                self.transition("DRIVE")
            elif self.elapsed() > 15.0:
                self.fail("LIFT_TIMEOUT")

        elif self.state == "DRIVE":
            if self.arrived:
                self.transition("WAIT_RELEASE")
            elif self.elapsed() > self.drive_timeout:
                self.fail("DRIVE_TIMEOUT")

        elif self.state == "WAIT_RELEASE":
            self.publish_ready_stage("RELEASE")
            self.maybe_publish_commit("RELEASE")
            if "RELEASE" in self.committed_stages:
                self.transition("RELEASE")
            elif self.elapsed() > 15.0:
                self.fail("RELEASE_BARRIER_TIMEOUT")

        elif self.state == "RELEASE":
            if not self.release_done:
                self.send_action_with_retry("release")
            if self.release_done:
                self.release_announced = True
            if self.release_announced:
                self.pub_release_done.publish(Bool(data=True))
                self.publish_ready_stage("RETURN")
                self.maybe_publish_commit("RETURN")
            if "RETURN" in self.committed_stages:
                self.local_lifted = False
                self.pub_lifted.publish(Bool(data=False))
                if not self.is_rear:
                    self.aggregate_lifted = False
                    self.pub_lifted_all.publish(Bool(data=False))
                self.transition("RETURN")
            elif self.elapsed() > 15.0:
                self.fail("RELEASE_TIMEOUT")

        elif self.state == "RETURN":
            if self.return_done:
                self.publish_ready_stage("HOME")
                self.maybe_publish_commit("HOME")
            if "HOME" in self.committed_stages:
                # 양쪽 HOME ready 이후의 commit만 진짜 mission 완료다.
                self.publish_mission_complete()
                self.reset()
                self.transition("IDLE")
            elif self.elapsed() > self.return_timeout:
                self.fail("RETURN_TIMEOUT")

        elif self.state == "FAULT":
            if self._fault_requires_estop(self.hardware_fault):
                self.pub_estop.publish(Bool(data=True))

    @staticmethod
    def _fault_requires_estop(fault):
        from cooperative_parking_robot.fault_policy import classify_fault
        return classify_fault(fault).estop_required

    def send_action_with_retry(self, action):
        now = time.monotonic()
        if now - self.last_action_time < 0.5:
            return
        self.last_action_time = now
        self.pub_grip.publish(String(data=action))

    def fail(self, reason):
        # Mission failure always stops progression, but only an explicitly
        # classified physical emergency is promoted to the MCU hard latch.
        self.hardware_fault = reason
        if self._fault_requires_estop(reason):
            self.pub_estop.publish(Bool(data=True))
        self.transition("FAULT")

    def rear_lifted_cb(self, msg):
        if msg.data:
            self.rear_lifted = True
            self.check_both_lifted()

    def check_both_lifted(self):
        if self.self_lifted and self.rear_lifted:
            self.pub_lifted_all.publish(Bool(data=True))
            if not self.aggregate_lifted:
                self.aggregate_lifted = True
                self.get_logger().info(
                    "both robots lifted; /robot/lifted=true")

    def publish_lifted(self):
        self.local_lifted = True
        self.pub_lifted.publish(Bool(data=True))
        if not self.is_rear:
            self.self_lifted = True
            self.check_both_lifted()

    def publish_mission_complete(self):
        """양쪽 HOME commit 뒤 Front만 최종 완료를 발행한다."""
        if self.pub_mission_complete is None or not self.active_mission_id:
            return
        payload = {
            "mission_id": self.active_mission_id,
            "stamp_ns": self.get_clock().now().nanoseconds,
        }
        self.pub_mission_complete.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"mission complete: {self.active_mission_id}")

    def reset(self):
        self.wheel_aligned = False
        self.lift_done = False
        self.release_done = False
        self.other_align_done = False
        self.alignment_announced = False
        self.aligned_hold = False
        self.other_release_done = False
        self.release_announced = False
        self.arrived = False
        self.approach_done = False
        self.return_done = False
        self.local_lifted = False
        self.aggregate_lifted = False
        self.hardware_fault = None
        self.rear_lifted = False
        self.self_lifted = False
        self.active_mission_id = ""
        self.active_plan_stamp_ns = 0
        self.fleet_state = "WAIT_TARGET"
        self.fleet_sequence = -1
        self.fleet_receipt_time = 0.0
        self.clear_coordination()

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))
        self.pub_lifted.publish(Bool(data=self.local_lifted))
        self.pub_align_done.publish(
            Bool(data=self.alignment_announced))
        self.pub_aligned_hold.publish(Bool(data=self.aligned_hold))
        self.pub_release_done.publish(
            Bool(data=self.release_announced))
        if not self.is_rear:
            self.pub_lifted_all.publish(
                Bool(data=self.aggregate_lifted))


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
