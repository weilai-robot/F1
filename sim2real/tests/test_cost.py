import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from spi.cost import CostWeights, PredictionCost, quat_err, per_signal_cost

N = 12  # steps


def make_ref():
    rng = np.random.default_rng(0)
    q = rng.normal(size=(N, 4)); q /= np.linalg.norm(q, axis=1, keepdims=True)
    return {"quat": q, "gyro": rng.normal(size=(N, 3)),
            "accel": rng.normal(size=(N, 3)),
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

    def test_accel_term_scales_with_weight(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        sim["accel"] = ref["accel"] + 0.3
        c1 = PredictionCost(weights=CostWeights(base_accel=1.0))
        c2 = PredictionCost(weights=CostWeights(base_accel=2.0))
        e1, e2 = c1.evaluate(sim, ref), c2.evaluate(sim, ref)
        self.assertAlmostEqual(e2, 2.0 * e1, places=9)
        # expected analytic value: weight * n * 3 * 0.09
        self.assertAlmostEqual(e1, 1.0 * N * 3 * 0.3 ** 2, places=9)

    def test_accel_nan_ref_masked(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        ref["accel"][3] = np.nan
        sim["accel"] = ref["accel"] + 0.5
        c = PredictionCost(weights=CostWeights(base_accel=1.0))
        self.assertAlmostEqual(c.evaluate(sim, ref), 1.0 * (N - 1) * 3 * 0.25, places=9)

    def test_accel_disabled_when_weight_zero(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        sim["accel"] = ref["accel"] + 5.0
        c = PredictionCost(weights=CostWeights(base_accel=0.0))
        self.assertAlmostEqual(c.evaluate(sim, ref), 0.0, places=9)

    def test_per_signal_breakdown_sums_to_total(self):
        ref = make_ref()
        sim = {k: v.copy() for k, v in ref.items()}
        sim["q"][:, 17:] += 0.1
        sim["gyro"] += 0.2
        sim["accel"] += 0.3
        sim["tau"] += 1.0
        c = PredictionCost(weights=CostWeights(),
                           joint_mask=np.arange(29) >= 17)
        parts = per_signal_cost(c, sim, ref)
        self.assertAlmostEqual(sum(parts.values()), c.evaluate(sim, ref), places=6)
        self.assertIn("accel", parts)

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
