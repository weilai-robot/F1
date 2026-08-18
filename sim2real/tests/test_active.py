import sys, unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from active.bezier import bezier_matrix, denormalize, sample_curve
from active.fim import a_optimality_objective, fd_jacobian, fim_from_jacobians


class TestBezier(unittest.TestCase):
    def test_endpoints(self):
        ctrl = np.array([0.1, 0.5, 0.9, 0.3])
        c = sample_curve(ctrl, 50)
        self.assertAlmostEqual(c[0], ctrl[0], places=9)
        self.assertAlmostEqual(c[-1], ctrl[-1], places=9)
        self.assertTrue(np.all((c >= -1e-9) & (c <= 1.0 + 1e-9)))

    def test_partition_of_unity(self):
        B = bezier_matrix(5, 20)
        np.testing.assert_allclose(B.sum(axis=1), 1.0, atol=1e-12)

    def test_denormalize(self):
        ranges = np.array([[-1.0, 1.0], [0.0, 0.5]])
        u = np.array([[0.0, 1.0], [0.0, 1.0]])  # (n_dim, n_points)
        d = denormalize(u, ranges)
        np.testing.assert_allclose(d[:, 0], [-1.0, 0.0])
        np.testing.assert_allclose(d[:, 1], [1.0, 0.5])


class TestFIM(unittest.TestCase):
    def test_fd_jacobian_linear(self):
        # f(theta) = A theta  =>  J = A
        A = np.array([[1.0, 2.0], [3.0, 4.0], [0.5, -1.0]])
        theta = np.array([0.3, -0.2])
        f = lambda th: A @ th
        J = fd_jacobian(lambda th, s: f(th), theta, delta=0.05)
        np.testing.assert_allclose(J, A, atol=1e-6)

    def test_fim_psd_and_a_opt(self):
        rng = np.random.default_rng(0)
        Js = [rng.normal(size=(6, 3)) for _ in range(10)]
        F = fim_from_jacobians(Js)
        self.assertEqual(F.shape, (3, 3))
        self.assertGreaterEqual(np.linalg.eigvalsh(F).min(), -1e-12)
        v = a_optimality_objective(F)
        self.assertTrue(np.isfinite(v) and v > 0)
        # more information -> smaller trace of inverse
        F2 = fim_from_jacobians(Js + Js)
        self.assertLessEqual(a_optimality_objective(F2), v + 1e-12)

    def test_singular_fim_fallback(self):
        F = np.zeros((3, 3))
        v = a_optimality_objective(F)
        self.assertTrue(np.isfinite(v))


if __name__ == "__main__":
    unittest.main()
