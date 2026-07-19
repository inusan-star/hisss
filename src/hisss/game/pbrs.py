import numpy as np

from hisss.cpp.lib import CPP_LIB
from hisss.game.state import BattleSnakeState


def compute_pbrs(state: BattleSnakeState) -> dict[int, float]:
    """Computes PBRS."""
    num_snakes = len(state.snakes_alive)

    # Setup.
    alives = np.array(state.snakes_alive, dtype=bool)
    lengths = np.array(state.snake_len, dtype=np.int32)
    heads_x = np.zeros(num_snakes, dtype=np.int32)
    heads_y = np.zeros(num_snakes, dtype=np.int32)
    obstacle_grid = np.zeros(225, dtype=bool)
    pbrs_out = np.zeros(num_snakes, dtype=np.float32)

    # Parse.
    for i, alive in enumerate(state.snakes_alive):
        if alive:
            body = state.snake_pos[i]

            heads_x[i] = body[0][0]
            heads_y[i] = body[0][1]

            for x, y in body:
                obstacle_grid[y * 15 + x] = True

    # Process.
    CPP_LIB.lib.compute_pbrs_cpp(num_snakes, lengths, heads_x, heads_y, alives, obstacle_grid, pbrs_out)

    # Return.
    return {i: val for i, val in enumerate(pbrs_out.tolist())}
