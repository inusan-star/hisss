import numpy as np

from hisss.cpp.lib import CPP_LIB


def process_state(
    state_json: str,
    features_flag: bool = False,
) -> list[int] | tuple[list[int], np.ndarray]:
    """Processes the game state."""
    return CPP_LIB.process_state(state_json, features_flag=features_flag)
