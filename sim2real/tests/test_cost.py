import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from spi.cost import CostWeights, PredictionCost, quat_err

N = 12  # steps


def make_ref():
    rng = np.random.default_rng(0)
    q = rng.normal(size=(N, 4)); q /= np.linalg.norm(q, axis=1, keepdims=True)
    return {"quat": q, "gyro": rng.normal(size=(N, 3)),
            "q": rng.normal(size=(N, 29)) * 0.1,
            "qd": rng.normal(size=(N, 29)),
            "tau": rng.normal(size=(N, 29))}


class TestQuatErr(unittest.TestCase):
    def test_zero_for_identical(self):
        q = np.array([[1, 0, 0, 0], [0.7, 0.1, 0.1, 0.7]])
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        np.testing.assert_allclose(quat_err(q, q), 0.0, atol=1e-12)

    def test_sign_invariant(self):
        q = np.array([[0.7, 0.1, 0.1, 0.7]]); q /= np.linalg.norm(q)
        np.testing.assert_allclose(quat_err(q, -q), quat_err(q, q), atol=1e-12)

    def test_positive_for_different(self):
        q1 = np.array([[1.0, 0, 0, 0]])
        q2 = np.array([[0.0, 1.0, 0, 0]])
        self.assertGreater(float(quat_err(q1, q2)[0]), 0.99)


class TestPredictionCost(unittest.TestCase):
    def test_zero_when_sim_equals_ref(self):
        ref = make_ref()
        c = PredictionCost(weights=CostWeights())
        sim = {k: v.copy() for k, v in ref.items()}
        # tau mask: all finite here
        self.assertAlmostEqual(c.evaluate(sim, ref), 0.0, places=9)

    def test_positive_when_different(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        sim["q"][3, 17:] += 0.2
        c = PredictionCost(weights=CostWeights(),
                           joint_mask=np.arange(29) >= 17)
        self.assertGreater(c.evaluate(sim, ref), 0.0)

    def test_nan_tau_ignored(self):
        ref = make_ref()
        ref["tau"][:, :17] = np.nan  # upper body torque not logged
        sim = {k: v.copy() for k, v in ref.items()}
        sim["tau"][:, :17] = 123.0
        c = PredictionCost(weights=CostWeights())
        self.assertAlmostEqual(c.evaluate(sim, ref), 0.0, places=9)

    def test_masked_joints_excluded(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        sim["q"][:, :17] += 5.0            # upper-body error must not count
        mask = np.zeros(29, dtype=bool); mask[17:] = True
        c = PredictionCost(weights=CostWeights(), joint_mask=mask)
        self.assertAlmostEqual(c.evaluate(sim, ref), 0.0, places=9)

    def test_weights_from_dict_rejects_unknown(self):
        with self.assertRaises(KeyError):
            CostWeights.from_dict({"nope": 1.0})


if __name__ == "__main__":
    unittest.main()
