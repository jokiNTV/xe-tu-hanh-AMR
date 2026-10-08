import unittest

from amr_stm32_bridge.kinematics import CommandLatch, twist_to_wheels


class KinematicsTest(unittest.TestCase):
    def test_forward_0_2_m_s_is_two_rad_s(self):
        command = twist_to_wheels(0.2, 0.0, 0.1, 0.6, 4.5)
        self.assertAlmostEqual(command.left_rad_s, 2.0)
        self.assertAlmostEqual(command.right_rad_s, 2.0)
        self.assertFalse(command.limited)

    def test_positive_yaw_turns_left(self):
        command = twist_to_wheels(0.0, 0.3, 0.1, 0.6, 4.5)
        self.assertAlmostEqual(command.left_rad_s, -0.9)
        self.assertAlmostEqual(command.right_rad_s, 0.9)

    def test_limit_scales_both_wheels_and_preserves_curvature(self):
        command = twist_to_wheels(0.4, 1.0, 0.1, 0.6, 4.5)
        self.assertTrue(command.limited)
        self.assertAlmostEqual(command.right_rad_s, 4.5)
        self.assertAlmostEqual(command.left_rad_s / command.right_rad_s, 1 / 7)

    def test_command_latch_times_out_to_zero(self):
        latch = CommandLatch(0.25)
        self.assertEqual(latch.sample(1.0), (0.0, 0.0, True))
        latch.update(2.0, 2.0, 1.0)
        self.assertEqual(latch.sample(1.20), (2.0, 2.0, False))
        self.assertEqual(latch.sample(1.26), (0.0, 0.0, True))


if __name__ == "__main__":
    unittest.main()
