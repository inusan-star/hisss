from hisss.cpp.lib import CPP_LIB


def get_safe_moves(state_json: str) -> list[int]:
    """Returns a list of safe moves as integer constants."""
    return CPP_LIB.get_safe_moves(state_json)
