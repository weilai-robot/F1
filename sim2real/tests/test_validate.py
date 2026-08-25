import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from spi.validate import (ACCEL_RMS_MAX, EFFECTIVE_RATIO, accel_rms, assess,
                          boundary_warnings, credibility_grade, split_clips)

M0 = 4.3041648
COM0 = np.array([0.00252285, -0.00063439, 0.03023409])
I0 = np.array([[0.02680559, -5.49e-06, 5.389e-05],
               [-5.49e-06, 0.01083128, -0.00011229],
               [5.389e-05, -0.00011229, 0.02180955]])

CFG = {
    "bodies": [{"name": "base",
                "nominal": {"mass": M0, "com": COM0.tolist(), "inertia": I0.tolist()},
                "mass_range": (3.0, 5.5), "com_range": (-0.06, 0.06),
                "inertia_diag_range": (0.005, 0.15), "inertia_offdiag_max": 0.03}],
    "motor_groups": [{"name": "knee", "joints": ["left_knee_pitch_joint"],
                      "kappa_nominal": 120.0, "kappa_range": (40.0, 160.0)}],
    "kappa_s_nominal": 0.55, "kappa_s_range": (0.3, 0.8),
}

NOMINAL = {"bodies": {"base": {"mass": M0, "com": COM0, "inertia": I0}},
           "motors": {"knee": 120.0}, "kappa_s": 0.55}

SIG = {"quat": 0.0, "angvel": 0.0, "accel": 0.0, "q": 0.0, "qd": 0.0, "tau": 0.0}


def mk_params(**kw):
    p = {"bodies": {"base": {"mass": M0, "com": COM0.copy(), "inertia": I0.copy()}},
         "motors": {"knee": 120.0}, "kappa_s": 0.55}
    if "mass" in kw:
        p["bodies"]["base"]["mass"] = kw["mass"]
    return p


class TestSplitClips(unittest.TestCase):
    def test_deterministic_and_exhaustive(self):
        clips = [{"n": i} for i in range(20)]
        tr1, va1 = split_clips(clips, 0.2, seed=0)
        tr2, va2 = split_clips(clips, 0.2, seed=0)
        self.assertEqual([c["n"] for c in va1], [c["n"] for c in va2])
        self.assertEqual(len(va1), 4)
        self.assertEqual(len(tr1), 16)
        self.assertEqual(sorted([c["n"] for c in tr1] + [c["n"] for c in va1]),
                         list(range(20)))


class TestAccelRms(unittest.TestCase):
    def test_math(self):
        # term = w * n * 3 * err^2 (3 axes) -> rms = err * sqrt(3)
        rms = accel_rms({"accel": 1.0 * 100 * 3 * 0.25}, weight=1.0, n_steps=100)
        self.assertAlmostEqual(rms, 0.5 * np.sqrt(3.0), places=9)

    def test_disabled(self):
        self.assertIsNone(accel_rms({"accel": 5.0}, weight=0.0, n_steps=10))
        self.assertIsNone(accel_rms({}, weight=1.0, n_steps=10))


class TestAssess(unittest.TestCase):
    def _run(self, params, val_nominal, val_best, n_val_steps=1000,
             accel_weight=1.0, train_costs=None):
        return assess(CFG, params, NOMINAL,
                      train_costs or {"nominal": dict(SIG), "best": dict(SIG)},
                      {"nominal": val_nominal, "best": val_best},
                      n_val_steps, accel_weight=accel_weight)

    def test_pass_for_good_identification(self):
        # best 明显优于 nominal、参数在物理域内、accel RMS 达标
        good = mk_params()
        vn = dict(SIG, quat=200.0, q=50.0)          # 250
        vb = dict(SIG, quat=40.0, q=10.0,
                  accel=1.0 * 1000 * 3 * 0.2 ** 2)  # 120, rms=0.346 -> 170/250=0.68
        r = self._run(good, vn, vb)
        self.assertEqual(r["verdict"], "PASS", r)
        self.assertEqual(r["exit_code"], 0)
        ok_ids = {c["id"] for c in r["checks"] if c["ok"]}
        self.assertEqual(ok_ids, {"EFFECTIVENESS", "PHYSICAL", "ACCEL", "ACTUATOR"})

    def test_fail_when_no_holdout_improvement(self):
        good = mk_params()
        vn = dict(SIG, quat=100.0)
        vb = dict(SIG, quat=90.0)  # ratio 0.9 > 0.7
        r = self._run(good, vn, vb)
        self.assertEqual(r["verdict"], "FAIL")
        eff = next(c for c in r["checks"] if c["id"] == "EFFECTIVENESS")
        self.assertFalse(eff["ok"])

    def test_fail_when_physically_implausible(self):
        # 评审场景复现：旧首轮结果 6.97 kg / com 0.19 / 惯量 1.5-2.0
        bad = mk_params(mass=6.97)
        bad["bodies"]["base"]["com"] = np.array([0.06, 0.19, -0.20])
        bad["bodies"]["base"]["inertia"] = np.diag([1.5, 1.2, 2.0])
        vn = dict(SIG, quat=100.0)
        vb = dict(SIG, quat=10.0, accel=1.0 * 1000 * 3 * 0.5 ** 2)
        r = self._run(bad, vn, vb)
        self.assertEqual(r["verdict"], "FAIL")
        phys = next(c for c in r["checks"] if c["id"] == "PHYSICAL")
        self.assertFalse(phys["ok"])
        self.assertIn("com_y", phys["detail"])
        # 可信度分级：质量偏差 +62% -> 低
        self.assertEqual(r["credibility"]["base.mass"], "低")
        self.assertEqual(r["credibility"]["base.inertia"], "低")

    def test_fail_when_accel_rms_too_high(self):
        good = mk_params()
        vn = dict(SIG, quat=100.0)
        vb = dict(SIG, quat=10.0,
                  accel=1.0 * 1000 * 20.0 ** 2)   # norm-rms 20 > max(15)
        r = self._run(good, vn, vb)
        self.assertEqual(r["verdict"], "FAIL")
        acc = next(c for c in r["checks"] if c["id"] == "ACCEL")
        self.assertFalse(acc["ok"])

    def test_fail_when_accel_not_improving_enough(self):
        # nominal 高于地板时仍要求 >=65% 改善：25 -> 13.9 超出
        # bar=min(15, max(13.5, 0.35*25=8.75))=13.5 -> FAIL
        good = mk_params()
        r = self._run(good, dict(SIG, quat=100.0,
                                  accel=1.0 * 1000 * 25.0 ** 2),
                       dict(SIG, quat=10.0, accel=1.0 * 1000 * 13.9 ** 2))
        acc = next(c for c in r["checks"] if c["id"] == "ACCEL")
        self.assertFalse(acc["ok"])   # 13.9 > bar 13.5
        # 低于地板（v13/v14 实测场景：nominal 20.2, best 12.9~13.0）-> PASS
        vb2 = dict(SIG, quat=10.0, accel=1.0 * 1000 * 12.9 ** 2)
        r2 = self._run(good, dict(SIG, quat=100.0,
                                  accel=1.0 * 1000 * 20.23 ** 2), vb2)
        acc2 = next(c for c in r2["checks"] if c["id"] == "ACCEL")
        self.assertTrue(acc2["ok"])

    def test_accel_floor_configurable_and_strict(self):
        # 地板可配置：抬高到 14 时 v13 场景（best 12.9）仍过；设 0 时要求纯 65% 改善
        good = mk_params()
        vn = dict(SIG, quat=100.0, accel=1.0 * 1000 * 20.23 ** 2)
        vb = dict(SIG, quat=10.0, accel=1.0 * 1000 * 12.9 ** 2)
        cfg = dict(CFG, validation={"accel_rms_floor": 0.0})
        r = assess(cfg, good, NOMINAL, {"nominal": dict(SIG), "best": dict(SIG)},
                   {"nominal": vn, "best": vb}, 1000, accel_weight=1.0)
        acc = next(c for c in r["checks"] if c["id"] == "ACCEL")
        self.assertFalse(acc["ok"])   # 12.9 > 0.35*20.23=7.08, floor disabled

    def test_accel_disabled_skips_check(self):
        good = mk_params()
        vn = dict(SIG, quat=100.0)
        vb = dict(SIG, quat=10.0)
        r = self._run(good, vn, vb, accel_weight=0.0)
        self.assertEqual(r["verdict"], "PASS")
        self.assertNotIn("ACCEL", {c["id"] for c in r["checks"]})

    def test_config_thresholds_apply(self):
        # config validation.accel_rms_max / effective_ratio 生效
        cfg = dict(CFG, validation={"accel_rms_max": 0.4, "effective_ratio": 0.5,
                                    "accel_rms_floor": 0.0})
        good = mk_params()
        vn = dict(SIG, quat=100.0)
        # best 0.68*250=170 但 ratio 0.68 > 0.5 -> EFFECTIVENESS FAIL
        vb = dict(SIG, quat=40.0, q=10.0, accel=1.0 * 1000 * 3 * 0.2 ** 2)
        r = assess(cfg, good, NOMINAL,
                   {"nominal": dict(SIG), "best": dict(SIG)},
                   {"nominal": vn, "best": vb}, 1000, accel_weight=1.0)
        self.assertEqual(r["verdict"], "FAIL")
        eff = next(c for c in r["checks"] if c["id"] == "EFFECTIVENESS")
        self.assertFalse(eff["ok"])
        # accel rms 0.346 <= 0.4 -> ACCEL ok
        acc = next(c for c in r["checks"] if c["id"] == "ACCEL")
        self.assertTrue(acc["ok"])
        self.assertEqual(r["criteria"]["accel_rms_max"], 0.4)

    def test_boundary_warning_on_kappa_s(self):
        p = mk_params()
        p["kappa_s"] = 0.306  # 贴盒底 (range [0.3, 0.8])
        warns = boundary_warnings(p, CFG)
        self.assertTrue(any("kappa_s" in w for w in warns))
        p2 = mk_params()
        self.assertEqual(boundary_warnings(p2, CFG), [])

    def test_fail_when_kappa_s_outside_actuator_band(self):
        # 完成标准 4（ACTUATOR）：kappa_s 落在搜索盒内但超出阶跃回归证据带
        # [0.34, 0.71] -> FAIL（防 kappa_s 吸收未建模误差）
        out = mk_params()
        out["kappa_s"] = 0.79  # in search box [0.3, 0.8], outside band
        vn = dict(SIG, quat=200.0, q=50.0)
        vb = dict(SIG, quat=40.0, q=10.0,
                  accel=1.0 * 1000 * 3 * 0.2 ** 2)
        r = self._run(out, vn, vb)
        self.assertEqual(r["verdict"], "FAIL")
        act = next(c for c in r["checks"] if c["id"] == "ACTUATOR")
        self.assertFalse(act["ok"])
        self.assertIn("0.790", act["detail"])

    def test_actuator_band_from_config(self):
        # validation.actuator_kappa_s_band 可配置
        cfg = dict(CFG, validation={"actuator_kappa_s_band": [0.4, 0.6]})
        p = mk_params()  # kappa_s = 0.55 in band -> ok
        vn = dict(SIG, quat=200.0, q=50.0)
        vb = dict(SIG, quat=40.0, q=10.0, accel=1.0 * 1000 * 3 * 0.2 ** 2)
        r = assess(cfg, p, NOMINAL, {"nominal": dict(SIG), "best": dict(SIG)},
                   {"nominal": vn, "best": vb}, 1000, accel_weight=1.0)
        act = next(c for c in r["checks"] if c["id"] == "ACTUATOR")
        self.assertTrue(act["ok"])
        self.assertEqual(r["criteria"]["actuator_kappa_s_band"], [0.4, 0.6])


class TestCredibility(unittest.TestCase):
    def test_grades(self):
        g = credibility_grade(NOMINAL, NOMINAL, CFG["bodies"],
                              CFG["motor_groups"], 0.55)
        self.assertEqual(g["base.mass"], "高")
        self.assertEqual(g["kappa.knee"], "高")
        self.assertEqual(g["kappa_s"], "高")
        mid = mk_params(mass=M0 * 1.3)
        g2 = credibility_grade(mid, NOMINAL, CFG["bodies"], CFG["motor_groups"], 0.55)
        self.assertEqual(g2["base.mass"], "中")


if __name__ == "__main__":
    unittest.main()
