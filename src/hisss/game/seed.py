from hisss.cpp.lib import CPP_LIB


def set_seed(seed: int):
    """Set the random seed for the C++ engine."""
    CPP_LIB.lib.set_seed(seed)
