import random

from hisss.cpp.lib import CPP_LIB


class ClassicalEngine:
    """Classical agent engine."""

    def __init__(self):
        """Initialize."""
        self.ptr = CPP_LIB.lib.create_classical_agent_cpp()

    def start(self, state_json: str):
        """Start game."""
        if self.ptr:
            CPP_LIB.lib.start_classical_agent_cpp(self.ptr, state_json.encode("utf-8"))

    def move(self, state_json: str) -> int:
        """Generate move."""
        if self.ptr:
            return CPP_LIB.lib.move_classical_agent_cpp(self.ptr, state_json.encode("utf-8"))

        return random.randint(0, 3)

    def end(self, state_json: str):
        """End game."""
        if self.ptr:
            CPP_LIB.lib.end_classical_agent_cpp(self.ptr, state_json.encode("utf-8"))

    def __del__(self):
        """Destroy."""
        if hasattr(self, "ptr") and self.ptr:
            CPP_LIB.lib.destroy_classical_agent_cpp(self.ptr)
            self.ptr = None
