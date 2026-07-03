#include "../../header/hebi/processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hebi {

StateProcessor::StateProcessor(const char* state_json) {
  auto j = nlohmann::json::parse(state_json);
  game_state_ = j.get<hebi::GameState>();
}

void StateProcessor::extract_safe_moves(bool* safe_moves_out) const {
  // Reset outputs.
  for (int i = 0; i < 4; ++i) {
    safe_moves_out[i] = false;
  }

  // Abort if dead.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state_.you.head.value();

  // Init obstacle map.
  std::vector<std::vector<bool>> obstacles(BOARD_SIZE, std::vector<bool>(BOARD_SIZE, false));
  std::vector<std::vector<int>> clear_time_grid(BOARD_SIZE, std::vector<int>(BOARD_SIZE, 0));

  // Process all snakes.
  for (const auto& snake : game_state_.board.snakes) {
    // Skip dead snakes.
    if (snake.elimination_event.has_value()) {
      continue;
    }

    int body_size = static_cast<int>(snake.body.size());
    bool is_stacked = (body_size < snake.length);

    // Determine tail persistence.
    int check_len = (!is_stacked && body_size > 1) ? body_size - 1 : body_size;

    // Mark body obstacles.
    for (int i = 0; i < check_len; ++i) {
      if (snake.body[i].has_value()) {
        hebi::Point p = snake.body[i].value();

        if (p.x >= 0 && p.x < BOARD_SIZE && p.y >= 0 && p.y < BOARD_SIZE) {
          obstacles[p.y][p.x] = true;
        }
      }
    }

    for (int b = 0; b < body_size; ++b) {
      if (snake.body[b].has_value()) {
        hebi::Point p = snake.body[b].value();

        if (p.x >= 0 && p.x < BOARD_SIZE && p.y >= 0 && p.y < BOARD_SIZE) {
          int turns_to_clear = body_size - b;
          if (turns_to_clear > clear_time_grid[p.y][p.x]) {
            clear_time_grid[p.y][p.x] = turns_to_clear;
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
  int reachable_counts[4] = {0, 0, 0, 0};
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
            bool time_blocked = (current_step < clear_time_grid[fy][fx]);

            // Push unvisited space.
            if (!time_blocked && !obstacles[fy][fx]) {
              visited[fy][fx] = visit_id;
              step_grid[fy][fx] = current_step + 1;
              queue.push_back({fx, fy});
            }
          }
        }
      }

      reachable_counts[i] = reachable_count;

      // Confirm safe space.
      if (reachable_count >= game_state_.you.length) {
        safe_moves_out[i] = true;
        any_safe = true;
      }
    }
  }

  // Restore longest surviving move if trapped.
  if (!any_safe) {
    int max_reachable = -1;
    for (int i = 0; i < 4; ++i) {
      if (immediate_safe[i] && reachable_counts[i] > max_reachable) {
        max_reachable = reachable_counts[i];
      }
    }
    if (max_reachable >= 0) {
      for (int i = 0; i < 4; ++i) {
        if (immediate_safe[i] && reachable_counts[i] == max_reachable) {
          safe_moves_out[i] = true;
        }
      }
    }
  }
}

void StateProcessor::encode_features(float* features_out) const {
  int plane_size = BOARD_SIZE * BOARD_SIZE;

  // Reset output array.
  std::fill(features_out, features_out + 9 * plane_size, 0.0f);

  // Abort if the player is eliminated.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state_.you.head.value();

  auto is_visible = [&](int x, int y) { return std::abs(x - head.x) + std::abs(y - head.y) <= VIEW_RADIUS; };

  auto set_features = [&](int c, int x, int y, float val) {
    if (x >= 0 && x < BOARD_SIZE && y >= 0 && y < BOARD_SIZE) {
      features_out[c * plane_size + y * BOARD_SIZE + x] = val;
    }
  };

  // Channel-0, Channel-1, Channel-2: You (Head, Body, Tail)
  int you_len = game_state_.you.body.size();
  for (int i = 0; i < you_len; ++i) {
    if (!game_state_.you.body[i].has_value()) continue;
    hebi::Point p = game_state_.you.body[i].value();

    if (i == 0) {
      set_features(0, p.x, p.y, 1.0f);
    }
    if (i > 0 && i < you_len - 1) {
      set_features(1, p.x, p.y, 1.0f);
    }
    if (i == you_len - 1) {
      set_features(2, p.x, p.y, 1.0f);
    }
  }

  // Channel-3, Channel-4, Channel-5: Enemies (Head, Body, Tail)
  for (const auto& snake : game_state_.board.snakes) {
    if (snake.elimination_event.has_value() || snake.id == game_state_.you.id) {
      continue;
    }

    int body_len = snake.body.size();

    for (int i = 0; i < body_len; ++i) {
      if (!snake.body[i].has_value()) continue;
      hebi::Point p = snake.body[i].value();

      if (is_visible(p.x, p.y)) {
        if (i == 0) {
          set_features(3, p.x, p.y, 1.0f);
        }
        if (i > 0 && i < body_len - 1) {
          set_features(4, p.x, p.y, 1.0f);
        }
        if (i == body_len - 1) {
          set_features(5, p.x, p.y, 1.0f);
        }
      }
    }
  }

  // Channel-6: Food
  for (const auto& food : game_state_.board.food) {
    set_features(6, food.x, food.y, 1.0f);
  }

  // Channel-7: You Health
  float health_val = static_cast<float>(game_state_.you.health.value_or(0)) / 100.0f;
  for (int y = 0; y < BOARD_SIZE; ++y) {
    for (int x = 0; x < BOARD_SIZE; ++x) {
      set_features(7, x, y, health_val);
    }
  }

  // Channel-8: Visibility
  for (int y = 0; y < BOARD_SIZE; ++y) {
    for (int x = 0; x < BOARD_SIZE; ++x) {
      if (is_visible(x, y)) {
        set_features(8, x, y, 1.0f);
      }
    }
  }
}

extern "C" {
int get_board_size_cpp() { return StateProcessor::BOARD_SIZE; }

void process_state_cpp(const char* state_json, bool* safe_moves_out, float* features_out) {
  StateProcessor processor(state_json);

  if (safe_moves_out != nullptr) {
    processor.extract_safe_moves(safe_moves_out);
  }

  if (features_out != nullptr) {
    processor.encode_features(features_out);
  }
}
}

}  // namespace hebi