"""SPI-Active stage 2: FIM-optimal active exploration."""

from .bezier import bezier_matrix, denormalize, sample_curve
from .fim import a_optimality_objective, fd_jacobian, fim_from_jacobians
from .command_opt import command_objective, optimize_commands

__all__ = ["bezier_matrix", "denormalize", "sample_curve",
           "a_optimality_objective", "fd_jacobian", "fim_from_jacobians",
           "command_objective", "optimize_commands"]
