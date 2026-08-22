import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from spi.param_space import (phi_search_box, phi_to_physical, physical_to_phi,
                             phi_to_U, tanh_motor_torque, ParamSpace, BodyParams,
                             MotorGroup, physical_violations,
                             physical_range_penalty)

# X1 pelvis nominal (from xyber_x1_serial.xml)
M0 = 4.3041648
COM0 = np.array([0.00252285, -0.00063439, 0.03023409])
I0 = np.array([[0.02680559, -5.49e-06, 5.389e-05],
               [-5.49e-06, 0.01083128, -0.00011229],
               [5.389e-05, -0.00011229, 0.02180955]])


class TestLogCholesky(unittest.TestCase):
    def test_roundtrip(self):
        phi = physical_to_phi(M0, COM0, I0)
        p = phi_to_physical(phi)
        self.assertAlmostEqual(p["mass"], M0, places=9)
        np.testing.assert_allclose(p["com"], COM0, atol=1e-9)
        np.testing.assert_allclose(p["inertia"], I0, atol=1e-9)

    def test_any_phi_is_feasible(self):
        rng = np.random.default_rng(0)
        for _ in range(200):
            phi = rng.normal(scale=1.0, size=10)
            p = phi_to_physical(phi)
            self.assertGreater(p["mass"], 0.0)
            lam = np.linalg.eigvalsh(p["inertia"])
            self.assertGreater(lam.min(), -1e-12)   # inertia PSD
            # fullinertia triangle inequality on diagonal
            d = np.diag(p["inertia"])
            self.assertTrue(np.all(d.sum() >= 2 * d - 1e-12))

    def test_U_upper_triangular_positive_diag(self):
        rng = np.random.default_rng(1)
        U = phi_to_U(rng.normal(size=10) * 0.5)
        self.assertTrue(np.all(np.diag(U) > 0))
        self.assertTrue(np.allclose(U, np.triu(U)))

    def test_search_box_valid(self):
        box = phi_search_box({"mass": M0}, (2.0, 10.0), (-0.15, 0.15), (0.005, 1.0))
        self.assertEqual(box.shape, (10, 2))
        self.assertTrue(np.all(box[:, 0] < box[:, 1]))
        # mass bound maps exactly: alpha range == 0.5 log mass range
        self.assertAlmostEqual(box[0, 0], 0.5 * np.log(2.0), places=9)
        self.assertAlmostEqual(box[0, 1], 0.5 * np.log(10.0), places=9)
        # a nominal-mass phi with zero com lands inside the com (t) box
        phi0 = physical_to_phi(M0, [0, 0, 0], np.diag([0.02, 0.012, 0.022]))
        self.assertTrue(np.all(phi0[7:10] >= box[7:10, 0] - 1e-12))
        self.assertTrue(np.all(phi0[7:10] <= box[7:10, 1] + 1e-12))


class TestMotorModel(unittest.TestCase):
    def test_small_command_linear(self):
        tau = tanh_motor_torque(np.array([1.0, 5.0]), kappa=100.0)
        np.testing.assert_allclose(tau, [1.0, 5.0], rtol=1e-2)

    def test_saturation(self):
        tau = tanh_motor_torque(np.array([1e4]), kappa=20.0)
        self.assertLess(tau[0], 20.0 * 1.001)
        self.assertGreater(tau[0], 19.0)
        # kappa_s linear gain
        tau2 = tanh_motor_torque(np.array([1.0]), kappa=100.0, kappa_s=1.5)
        self.assertAlmostEqual(tau2[0], 1.5, delta=1e-3)


class TestParamSpace(unittest.TestCase):
    def _space(self):
        body = BodyParams("base", {"mass": M0, "com": COM0, "inertia": I0},
                          (2.0, 10.0), (-0.15, 0.15), (0.005, 1.0))
        mg = MotorGroup("knee", ["left_knee_pitch_joint"], 120.0, (40.0, 160.0))
        return ParamSpace(bodies=[body], motor_groups=[mg])

    def test_regularization_zero_at_nominal(self):
        s = self._space()
        self.assertAlmostEqual(s.regularization(s.nominal_params()), 0.0, places=9)

    def test_regularization_grows(self):
        s = self._space()
        p = s.nominal_params()
        p["bodies"]["base"] = {"mass": M0 + 1.0, "com": COM0 + 0.01, "inertia": I0 * 1.1}
        self.assertGreater(s.regularization(p), 0.0)

    def test_dim(self):
        s = self._space()
        self.assertEqual(s.dim, 10 + 1 + 1)

    def test_to_json(self):
        import json
        s = self._space()
        d = json.loads(s.to_json(s.nominal_params()))
        self.assertAlmostEqual(d["bodies"]["base"]["mass"], M0, places=6)


class TestPhysicalRangePenalty(unittest.TestCase):
    def _cfg_body(self):
        return {"name": "base", "mass_range": (3.0, 5.5),
                "com_range": (-0.06, 0.06), "inertia_diag_range": (0.005, 0.15)}

    def _params(self, mass=M0, com=None, inertia=None):
        return {"bodies": {"base": {"mass": mass,
                                    "com": np.asarray(com if com is not None else COM0),
                                    "inertia": np.asarray(inertia if inertia is not None else I0)}}}

    def test_nominal_no_violation(self):
        cfg = [self._cfg_body()]
        self.assertEqual(physical_violations(self._params(), cfg), {})
        self.assertEqual(physical_range_penalty(self._params(), cfg), 0.0)

    def test_out_of_range_detected(self):
        cfg = [self._cfg_body()]
        # 旧首轮辨识结果：质量 6.97 kg、com_y/z ±0.19/-0.20、惯量 1.5-2.0 —— 必须被标记
        bad = self._params(mass=6.97,
                           com=[0.06, 0.19, -0.20],
                           inertia=np.diag([1.5, 1.2, 2.0]))
        viol = physical_violations(bad, cfg)
        self.assertIn("base", viol)
        v = viol["base"]
        self.assertGreater(v["mass"], 1.4)
        self.assertIn("com_y", v)
        self.assertIn("inertia_z", v)
        self.assertGreater(physical_range_penalty(bad, cfg), 1e4)

    def test_inertia_boundary_in_range(self):
        cfg = [self._cfg_body()]
        p = self._params(inertia=np.diag([0.13, 0.02, 0.14]))
        self.assertEqual(physical_violations(p, cfg), {})


if __name__ == "__main__":
    unittest.main()
