"""Versioned, shared success contract for F1 navigation evaluation."""

POSITION_SUCCESS_THRESHOLD_M = 0.35
SUCCESS_CONTRACT = {
    "fall": False,
    "collisions": 0,
    "position_error_m": {
        "operator": "<",
        "value": POSITION_SUCCESS_THRESHOLD_M,
    },
}
SUCCESS_CONTRACT_VERSION = 1
