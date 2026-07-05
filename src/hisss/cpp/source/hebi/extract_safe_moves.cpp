#include <algorithm>
#include <cmath>
#include <vector>

#include "../../header/hebi/processor.h"

namespace hebi {

void StateProcessor::extract_safe_moves(bool* safe_moves_out) const {
  // Reset outputs.
  for (int i = 0; i < 4; ++i) {
    safe_moves_out[i] = false;
  }

  // Abort if the player is eliminated.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state_.you.head.value();

  // Init obstacle map.
  std::vector<std::vector<bool>> obstacles(BOARD_SIZE, std::vector<bool>(BOARD_SIZE, false));
  std::vector<std::vector<int>> self_clear_time_grid(BOARD_SIZE, std::vector<int>(BOARD_SIZE, 0));
  bool is_enemy_tail[4] = {false, false, false, false};

  // Process all snakes.
  for (const auto& snake : game_state_.board.snakes) {
    // Skip dead snakes.
    if (snake.elimination_event.has_value()) {
      continue;
    }

    int body_size = static_cast<int>(snake.body.size());
    bool is_stacked = (body_size < snake.length);

    // Determine tail persistence.
    int check_len = body_size;
    if (snake.id == game_state_.you.id && (!is_stacked && body_size > 1)) {
      check_len = body_size - 1;
    }

    // Mark body obstacles.
    for (int i = 0; i < check_len; ++i) {
      if (snake.body[i].has_value()) {
        hebi::Point p = snake.body[i].value();

        if (p.x >= 0 && p.x < BOARD_SIZE && p.y >= 0 && p.y < BOARD_SIZE) {
          obstacles[p.y][p.x] = true;

          if (snake.id == game_state_.you.id) {
            self_clear_time_grid[p.y][p.x] = snake.length - i;

          } else if (i == body_size - 1 && snake.body[body_size - 1].has_value()) {
            for (int d = 0; d < 4; ++d) {
              if (head.x + hebi::dx(static_cast<hebi::Direction>(d)) == p.x && head.y + hebi::dy(static_cast<hebi::Direction>(d)) == p.y) {
                is_enemy_tail[d] = true;
              }
            }
          }
        }
      }
    }
  }

  // Prep fast BFS grid.
  std::vector<std::vector<int>> visited(BOARD_SIZE, std::vector<int>(BOARD_SIZE, 0));
  std::vector<std::vector<int>> step_grid(BOARD_SIZE, std::vector<int>(BOARD_SIZE, 0));
  std::vector<hebi::Point> queue;
  queue.reserve(BOARD_SIZE * BOARD_SIZE);
  int visit_id = 0;

  bool immediate_safe[4] = {false, false, false, false};
  bool any_safe = false;

  // Eval each move direction.
  for (int i = 0; i < 4; ++i) {
    hebi::Direction dir = static_cast<hebi::Direction>(i);
    int nx = head.x + hebi::dx(dir);
    int ny = head.y + hebi::dy(dir);

    // Check bounds and collision.
    if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE && !obstacles[ny][nx]) {
      immediate_safe[i] = true;
      visit_id++;
      queue.clear();

      // Start flood fill.
      queue.push_back({nx, ny});
      visited[ny][nx] = visit_id;
      step_grid[ny][nx] = 1;
      int reachable_count = 0;
      size_t queue_index = 0;

      // Expand space count.
      while (queue_index < queue.size() && reachable_count < game_state_.you.length) {
        hebi::Point current = queue[queue_index++];
        int current_step = step_grid[current.y][current.x];
        reachable_count++;

        // Explore four neighbors.
        for (int d = 0; d < 4; ++d) {
          int fx = current.x + hebi::dx(static_cast<hebi::Direction>(d));
          int fy = current.y + hebi::dy(static_cast<hebi::Direction>(d));

          // Check grid bounds.
          if (fx >= 0 && fx < BOARD_SIZE && fy >= 0 && fy < BOARD_SIZE && visited[fy][fx] != visit_id) {
            bool time_blocked = (current_step < self_clear_time_grid[fy][fx]);

            // Push unvisited space.
            if (!time_blocked) {
              visited[fy][fx] = visit_id;
              step_grid[fy][fx] = current_step + 1;
              queue.push_back({fx, fy});
            }
          }
        }
      }

      // Confirm safe space.
      if (reachable_count >= game_state_.you.length) {
        safe_moves_out[i] = true;
        any_safe = true;
      }
    }
  }

  // Restore fallback moves.
  if (!any_safe) {
    bool enemy_tail_found = false;

    for (int i = 0; i < 4; ++i) {
      if (is_enemy_tail[i]) {
        safe_moves_out[i] = true;
        enemy_tail_found = true;
      }
    }

    if (!enemy_tail_found) {
      for (int i = 0; i < 4; ++i) {
        if (immediate_safe[i]) {
          safe_moves_out[i] = true;
        }
      }
    }
  }
}

}  // namespace hebi
