"""UI 승인 게이트(P2)와 임무 리셋(P3)의 순수 로직 검증.

rclpy 없이 돌리기 위해 노드 전체가 아니라 두 규칙만 모델링한다.
두 규칙 모두 회귀하면 증상이 '조용한 오작동'이라 반드시 고정해 둔다.

  1. 승인은 1회성이다 — WAIT_LIFT 진입 순간 소비돼야 하며, 소비되지 않으면
     다음 임무가 버튼 없이 자동 시작된다.
  2. 임무 완료는 latch를 풀어야 한다 — 풀지 않으면 두 번째 차량을
     영원히 인식하지 못한다.
"""

import unittest


class ApprovalGate:
    """fleet_manager_node.manage_loop의 WAIT_TARGET 전이 규칙 모델."""

    def __init__(self, require_ui_confirmation=True, timeout_s=10.0):
        self.require_ui_confirmation = require_ui_confirmation
        self.timeout_s = timeout_s
        self.state = 'WAIT_TARGET'
        self.approved = False
        self.approved_time = 0.0
        self.sequence = -1

    def approve(self, sequence, now):
        if sequence <= self.sequence:
            return False
        if self.state != 'WAIT_TARGET':
            return False
        self.sequence = sequence
        self.approved = True
        self.approved_time = now
        return True

    def step(self, has_target, now):
        if self.state != 'WAIT_TARGET':
            return self.state
        if self.approved and now - self.approved_time > self.timeout_s:
            self.approved = False
        if has_target:
            if not self.require_ui_confirmation:
                self.state = 'WAIT_LIFT'
            elif self.approved:
                self.approved = False       # 진입 순간 소비
                self.state = 'WAIT_LIFT'
        return self.state

    def mission_complete(self):
        self.state = 'WAIT_TARGET'
        self.approved = False


class UiGateTest(unittest.TestCase):
    def test_target_alone_does_not_start_mission(self):
        gate = ApprovalGate()
        for tick in range(20):
            self.assertEqual(gate.step(True, tick * 0.5), 'WAIT_TARGET')

    def test_button_starts_mission(self):
        gate = ApprovalGate()
        self.assertTrue(gate.approve(1, 0.0))
        self.assertEqual(gate.step(True, 0.1), 'WAIT_LIFT')

    def test_approval_is_consumed_once(self):
        """두 번째 차량이 버튼 없이 실려 나가면 안 된다."""
        gate = ApprovalGate()
        gate.approve(1, 0.0)
        self.assertEqual(gate.step(True, 0.1), 'WAIT_LIFT')
        gate.mission_complete()
        for tick in range(10):
            self.assertEqual(gate.step(True, 20.0 + tick), 'WAIT_TARGET')

    def test_approval_expires_without_target(self):
        gate = ApprovalGate(timeout_s=10.0)
        gate.approve(1, 0.0)
        gate.step(False, 5.0)
        self.assertTrue(gate.approved)
        gate.step(False, 11.0)
        self.assertFalse(gate.approved)
        self.assertEqual(gate.step(True, 11.1), 'WAIT_TARGET')

    def test_duplicate_sequence_rejected(self):
        gate = ApprovalGate()
        self.assertTrue(gate.approve(5, 0.0))
        gate.approved = False
        self.assertFalse(gate.approve(5, 1.0))
        self.assertFalse(gate.approve(4, 1.0))

    def test_disabled_confirmation_keeps_v19_behaviour(self):
        gate = ApprovalGate(require_ui_confirmation=False)
        self.assertEqual(gate.step(True, 0.0), 'WAIT_LIFT')


class TargetLatch:
    """yolo_bev_map_node의 타겟 latch / spec 발행 상태 모델."""

    def __init__(self):
        self.latched = None
        self.spec_sent = False

    def observe(self, target):
        if self.latched is not None:
            return self.latched
        if target is not None:
            self.latched = target
        return self.latched

    def publish_spec(self):
        if self.spec_sent:
            return False
        self.spec_sent = True
        return True

    def mission_complete(self):
        self.latched = None
        self.spec_sent = False


class MissionResetTest(unittest.TestCase):
    def test_second_vehicle_is_latched_after_completion(self):
        latch = TargetLatch()
        self.assertEqual(latch.observe((2.3, 0.6)), (2.3, 0.6))
        self.assertTrue(latch.publish_spec())
        latch.mission_complete()
        self.assertIsNone(latch.latched)
        self.assertEqual(latch.observe((2.4, 0.6)), (2.4, 0.6))

    def test_spec_is_republished_for_each_mission(self):
        """spec_sent가 남으면 Fleet이 이전 차량 제원으로 계획한다."""
        latch = TargetLatch()
        latch.observe((2.3, 0.6))
        self.assertTrue(latch.publish_spec())
        self.assertFalse(latch.publish_spec())
        latch.mission_complete()
        latch.observe((2.3, 0.6))
        self.assertTrue(latch.publish_spec())

    def test_without_reset_second_mission_is_impossible(self):
        """리셋 경로가 없던 v1.9의 증상을 명시적으로 고정한다."""
        latch = TargetLatch()
        latch.observe((2.3, 0.6))
        # mission_complete 호출 없음
        self.assertEqual(latch.observe((2.4, 0.6)), (2.3, 0.6))


if __name__ == '__main__':
    unittest.main()
