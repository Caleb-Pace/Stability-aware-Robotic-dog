import unittest
from types import SimpleNamespace

import numpy as np

from py_src.state_estimator import StabilityEKF, StabilitySensorSample


class StabilityEKFTests(unittest.TestCase):
    def stationary_sample(self, contacts=None):
        return StabilitySensorSample(
            accelerometer=np.array([0.0, 0.0, 9.81]),
            gyroscope=np.zeros(3),
            foot_contacts=np.ones(4, dtype=bool) if contacts is None else contacts,
        )

    def test_stationary_robot_is_stable(self):
        estimator = StabilityEKF(dt=0.005)

        estimate = None
        for _ in range(100):
            estimate = estimator.step(self.stationary_sample())

        self.assertIsNotNone(estimate)
        self.assertTrue(estimate.stable)
        self.assertAlmostEqual(estimate.roll, 0.0, places=5)
        self.assertAlmostEqual(estimate.pitch, 0.0, places=5)
        np.testing.assert_allclose(estimate.horizontal_velocity, np.zeros(2), atol=1e-5)

    def test_contact_update_removes_velocity_drift(self):
        estimator = StabilityEKF(dt=0.01)
        estimator.x[2:5] = [1.0, -0.5, 0.25]

        estimate = estimator.step(self.stationary_sample())

        self.assertLess(np.linalg.norm(estimate.horizontal_velocity), 0.1)
        self.assertEqual(estimate.contact_count, 4)

    def test_tilted_gravity_is_observed(self):
        estimator = StabilityEKF(dt=0.01)
        roll = np.deg2rad(10.0)
        accelerometer = estimator._rotation_world_from_body(roll, 0.0).T @ np.array([0.0, 0.0, 9.81])
        sample = StabilitySensorSample(accelerometer, np.zeros(3), np.ones(4, dtype=bool))

        for _ in range(30):
            estimate = estimator.step(sample)

        self.assertAlmostEqual(estimate.roll, roll, delta=np.deg2rad(1.0))

    def test_low_state_adapter_reads_imu_and_contacts(self):
        estimator = StabilityEKF(dt=0.01)
        low_state = SimpleNamespace(
            imu_state=SimpleNamespace(
                accelerometer=np.array([0.0, 0.0, 9.81]),
                gyroscope=np.zeros(3),
            ),
            foot_force=np.array([30.0, 0.0, 25.0, 5.0]),
        )

        estimate = estimator.step_low_state(low_state)

        self.assertEqual(estimate.contact_count, 2)
        self.assertTrue(estimate.stable)


if __name__ == "__main__":
    unittest.main()