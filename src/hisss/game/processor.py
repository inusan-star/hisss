import numpy as np

from hisss.cpp.lib import CPP_LIB


def process_state(
    partial_state_json: str,
    perfect_state_json: str | None = None,
    features_flag: bool = False,
) -> np.ndarray | tuple[np.ndarray, np.ndarray] | tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Processes the game state"""
    return CPP_LIB.process_state(
        partial_state_json,
        perfect_state_json=perfect_state_json,
        features_flag=features_flag,
    )
