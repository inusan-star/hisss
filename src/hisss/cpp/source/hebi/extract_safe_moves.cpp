#include <algorithm>
#include <cmath>
#include <vector>

#include "../../header/hebi/processor.h"

namespace hebi {

// Internal search state.
struct SearchState {
  hebi::Point position;
  int food_eaten;
};

// Internal evaluation state.
struct MoveEvaluation {
  bool is_accessible;
  bool is_space_sufficient;
  bool is_enemy_head;
  bool is_enemy_tail;
  int reachable_max_space;
};

void StateProcessor::extract_safe_moves(bool* safe_moves_out) const {
  // Reset outputs.
  for (int i = 0; i < 4; ++i) {
    safe_moves_out[i] = false;
  }

  // Elimination check.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  const hebi::Point head = game_state_.you.head.value();

  // Distance check helper.
  auto get_distance = [](int source_x, int source_y, int dest_x, int dest_y) { return std::abs(source_x - dest_x) + std::abs(source_y - dest_y); };

  // Bounds check helper.
  auto is_in_bounds = [](int target_x, int target_y) { return target_x >= 0 && target_x < BOARD_SIZE && target_y >= 0 && target_y < BOARD_SIZE; };

  // Visibility check helper.
  auto is_visible = [&](int target_x, int target_y) { return get_distance(target_x, target_y, head.x, head.y) <= VIEW_RADIUS; };

  // Grid layer initialization.
  alignas(16) bool food_grid[BOARD_SIZE * BOARD_SIZE] = {false};
  alignas(16) bool player_body_grid[BOARD_SIZE * BOARD_SIZE] = {false};
  alignas(16) bool enemy_head_grid[BOARD_SIZE * BOARD_SIZE] = {false};
  alignas(16) bool enemy_tail_grid[BOARD_SIZE * BOARD_SIZE] = {false};
  alignas(16) bool obstacles[BOARD_SIZE * BOARD_SIZE] = {false};
  alignas(16) int clear_time_grid[BOARD_SIZE * BOARD_SIZE] = {0};

  // Static food mapping.
  for (const auto& food : game_state_.board.food) {
    if (is_in_bounds(food.x, food.y)) {
      food_grid[food.y * BOARD_SIZE + food.x] = true;
    }
  }

  // Snake body mapping.
  for (const auto& snake : game_state_.board.snakes) {
    // Elimination check.
    if (snake.elimination_event.has_value()) {
      continue;
    }

    // Tail persistence determination.
    const int body_size = static_cast<int>(snake.body.size());
    const bool is_stacked = (body_size < snake.length);

    if (snake.id == game_state_.you.id) {
      // Body segment mapping for player.
      for (int i = 0; i < body_size; ++i) {
        if (snake.body[i].has_value()) {
          const hebi::Point p = snake.body[i].value();

          if (is_in_bounds(p.x, p.y)) {
            const int idx = p.y * BOARD_SIZE + p.x;
            player_body_grid[idx] = true;

            if ((i < body_size - 1) || is_stacked) {
              obstacles[idx] = true;
              clear_time_grid[idx] = std::max(clear_time_grid[idx], snake.length - i);
            }
          }
        }
      }
    } else {
      // Check visibility or disadvantage status.
      bool has_null_segment = false;

      for (const auto& segment : snake.body) {
        if (!segment.has_value()) {
          has_null_segment = true;
          break;
        }
      }

      const bool is_disadvantaged = (snake.length >= game_state_.you.length);

      // Enemy head neighbors restriction.
      if (snake.body.front().has_value() && (has_null_segment || is_disadvantaged)) {
        const hebi::Point enemy_head = snake.body.front().value();

        for (int d = 0; d < 4; ++d) {
          int hx = enemy_head.x + hebi::dx(static_cast<hebi::Direction>(d));
          int hy = enemy_head.y + hebi::dy(static_cast<hebi::Direction>(d));

          if (is_in_bounds(hx, hy)) {
            enemy_head_grid[hy * BOARD_SIZE + hx] = true;
          }
        }
      }

      // Valid segment detection for enemies.
      int first_valid_idx = -1;
      int last_valid_idx = -1;

      for (int i = 0; i < body_size; ++i) {
        if (snake.body[i].has_value()) {
          if (first_valid_idx == -1) first_valid_idx = i;
          last_valid_idx = i;
        }
      }

      if (first_valid_idx != -1) {
        hebi::Point first_valid_pos = snake.body[first_valid_idx].value();
        hebi::Point last_valid_pos = snake.body[last_valid_idx].value();
        hebi::Point current_pos = first_valid_pos;

        // Buffer initialization.
        alignas(16) hebi::Point segments_to_write[BOARD_SIZE * BOARD_SIZE];
        alignas(16) int logical_indices[BOARD_SIZE * BOARD_SIZE];
        int write_count = 0;
        int current_logical_idx = 0;

        // Virtual head assignment.
        if (!snake.body.front().has_value()) {
          int min_dist_to_you = 999;
          hebi::Point valid_heads[4];
          int head_count = 0;

          for (int d = 0; d < 4; ++d) {
            int hx = first_valid_pos.x + hebi::dx(static_cast<hebi::Direction>(d));
            int hy = first_valid_pos.y + hebi::dy(static_cast<hebi::Direction>(d));

            // Combined bounds, visibility, and player duplicate check.
            if (is_in_bounds(hx, hy) && !is_visible(hx, hy) && !player_body_grid[hy * BOARD_SIZE + hx]) {
              int current_dist = get_distance(hx, hy, head.x, head.y);

              if (current_dist < min_dist_to_you) {
                min_dist_to_you = current_dist;
                head_count = 0;
                valid_heads[head_count++] = {hx, hy};
              } else if (current_dist == min_dist_to_you) {
                valid_heads[head_count++] = {hx, hy};
              }
            }
          }

          for (int head_idx = 0; head_idx < head_count; ++head_idx) {
            segments_to_write[write_count] = valid_heads[head_idx];
            logical_indices[write_count] = current_logical_idx;
            write_count++;
          }

          current_pos = valid_heads[0];
        } else {
          segments_to_write[write_count] = current_pos;
          logical_indices[write_count] = current_logical_idx;
          write_count++;
        }

        // Body interpolation.
        for (int i = first_valid_idx; i <= last_valid_idx; ++i) {
          if (snake.body[i].has_value()) {
            hebi::Point next_target = snake.body[i].value();

            // Validation helper for interpolation steps.
            auto is_valid_step = [&](const hebi::Point& step_pos) {
              return is_in_bounds(step_pos.x, step_pos.y) &&
                     (!is_visible(step_pos.x, step_pos.y) || (step_pos.x == next_target.x && step_pos.y == next_target.y)) &&
                     !player_body_grid[step_pos.y * BOARD_SIZE + step_pos.x];
            };

            // Gap interpolation.
            while (current_pos.x != next_target.x || current_pos.y != next_target.y) {
              hebi::Point candidate_pos = current_pos;
              bool is_valid_pos = false;

              // X-axis interpolation.
              if (current_pos.x != next_target.x) {
                candidate_pos.x += (current_pos.x < next_target.x) ? 1 : -1;
                is_valid_pos = is_valid_step(candidate_pos);
              }

              // Y-axis interpolation.
              if (current_pos.y != next_target.y && !is_valid_pos) {
                candidate_pos = current_pos;
                candidate_pos.y += (current_pos.y < next_target.y) ? 1 : -1;
              }

              current_pos = candidate_pos;

              // Dynamic duplicate check.
              bool is_duplicate = false;

              for (int check_idx = 0; check_idx < write_count; ++check_idx) {
                if (current_pos.x == segments_to_write[check_idx].x && current_pos.y == segments_to_write[check_idx].y) {
                  is_duplicate = true;
                  break;
                }
              }

              if (!is_duplicate) {
                current_logical_idx++;
                segments_to_write[write_count] = current_pos;
                logical_indices[write_count] = current_logical_idx;
                write_count++;
              }
            }
          }
        }

        // Virtual tail assignment.
        if (!snake.body.back().has_value()) {
          int min_dist_to_you = 999;
          hebi::Point valid_tails[4];
          int tail_count = 0;

          for (int d = 0; d < 4; ++d) {
            int tx = last_valid_pos.x + hebi::dx(static_cast<hebi::Direction>(d));
            int ty = last_valid_pos.y + hebi::dy(static_cast<hebi::Direction>(d));

            // Combined bounds, visibility, and player duplicate check.
            if (is_in_bounds(tx, ty) && !is_visible(tx, ty) && !player_body_grid[ty * BOARD_SIZE + tx]) {
              // Dynamic duplicate check.
              bool is_duplicate = false;

              for (int check_idx = 0; check_idx < write_count; ++check_idx) {
                if (tx == segments_to_write[check_idx].x && ty == segments_to_write[check_idx].y) {
                  is_duplicate = true;
                  break;
                }
              }

              if (!is_duplicate) {
                int current_dist = get_distance(tx, ty, head.x, head.y);

                if (current_dist < min_dist_to_you) {
                  min_dist_to_you = current_dist;
                  tail_count = 0;
                  valid_tails[tail_count++] = {tx, ty};
                } else if (current_dist == min_dist_to_you) {
                  valid_tails[tail_count++] = {tx, ty};
                }
              }
            }
          }

          current_logical_idx++;

          for (int tail_idx = 0; tail_idx < tail_count; ++tail_idx) {
            segments_to_write[write_count] = valid_tails[tail_idx];
            logical_indices[write_count] = current_logical_idx;
            write_count++;
          }
        }

        // Grid batch optimization.
        const int total_logical_steps = current_logical_idx + 1;

        for (int i = 0; i < write_count; ++i) {
          const hebi::Point p = segments_to_write[i];
          const int idx = p.y * BOARD_SIZE + p.x;
          obstacles[idx] = true;

          const int time_to_clear = total_logical_steps - logical_indices[i];
          clear_time_grid[idx] = std::max(clear_time_grid[idx], time_to_clear);

          if (logical_indices[i] == current_logical_idx) {
            enemy_tail_grid[idx] = true;
          }
        }
      }
    }
  }

  // Prep fast BFS grid.
  alignas(16) int visited[BOARD_SIZE * BOARD_SIZE] = {0};
  alignas(16) int step_grid[BOARD_SIZE * BOARD_SIZE] = {0};
  SearchState queue[BOARD_SIZE * BOARD_SIZE];
  int visit_id = 0;

  // Track strategy conditions.
  MoveEvaluation evaluations[4];

  for (int i = 0; i < 4; ++i) {
    evaluations[i] = {false, false, false, false, 0};
  }

  // Direction evaluation.
  for (int i = 0; i < 4; ++i) {
    const hebi::Direction dir = static_cast<hebi::Direction>(i);
    const int nx = head.x + hebi::dx(dir);
    const int ny = head.y + hebi::dy(dir);

    // Collision check.
    if (is_in_bounds(nx, ny)) {
      const int target_idx = ny * BOARD_SIZE + nx;
      evaluations[i].is_accessible = !obstacles[target_idx];
      evaluations[i].is_enemy_head = enemy_head_grid[target_idx];
      evaluations[i].is_enemy_tail = enemy_tail_grid[target_idx];

      if (evaluations[i].is_accessible) {
        visit_id++;
        int queue_start = 0;
        int queue_end = 0;

        // Start flood fill.
        int initial_food = food_grid[target_idx] ? 1 : 0;
        queue[queue_end++] = {{nx, ny}, initial_food};
        visited[target_idx] = visit_id;
        step_grid[target_idx] = 1;
        int reachable_count = 0;

        // Tail chase check.
        bool can_tail_chase = player_body_grid[target_idx];

        // Expand space count.
        while (queue_start < queue_end && reachable_count < game_state_.you.length) {
          SearchState current = queue[queue_start++];
          const int current_idx = current.position.y * BOARD_SIZE + current.position.x;
          int current_step = step_grid[current_idx];
          reachable_count++;

          // Explore four neighbors.
          for (int d = 0; d < 4; ++d) {
            int fx = current.position.x + hebi::dx(static_cast<hebi::Direction>(d));
            int fy = current.position.y + hebi::dy(static_cast<hebi::Direction>(d));

            if (is_in_bounds(fx, fy)) {
              const int neighbor_idx = fy * BOARD_SIZE + fx;

              if (visited[neighbor_idx] != visit_id) {
                // Dynamic tail persistence calculation.
                int effective_clear_time = clear_time_grid[neighbor_idx];

                if (player_body_grid[neighbor_idx]) {
                  effective_clear_time += current.food_eaten;
                }

                bool time_blocked = ((current_step + 1) < effective_clear_time);

                // Push unvisited space.
                if (!time_blocked) {
                  // Tail chase check.
                  if (player_body_grid[neighbor_idx]) {
                    can_tail_chase = true;
                  }

                  visited[neighbor_idx] = visit_id;
                  step_grid[neighbor_idx] = current_step + 1;
                  int next_food_count = current.food_eaten + (food_grid[neighbor_idx] ? 1 : 0);
                  queue[queue_end++] = {{fx, fy}, next_food_count};
                }
              }
            }
          }
        }

        evaluations[i].reachable_max_space = reachable_count;

        // Space sufficiency check.
        if (reachable_count >= game_state_.you.length || can_tail_chase) {
          evaluations[i].is_space_sufficient = true;
        }

        // Confirm safe space.
        if (evaluations[i].is_space_sufficient && !evaluations[i].is_enemy_head) {
          safe_moves_out[i] = true;
        }
      }
    }
  }

  // Check if any perfect safe move exists.
  bool has_perfect_move = false;

  for (int i = 0; i < 4; ++i) {
    if (safe_moves_out[i]) {
      has_perfect_move = true;
    }
  }

  // Execute multi-tier fallback decision tree.
  if (!has_perfect_move) {
    bool fallback_found = false;

    // Fallback: Allow space-sufficient enemy heads or enemy tails.
    for (int i = 0; i < 4; ++i) {
      if ((evaluations[i].is_accessible && evaluations[i].is_space_sufficient && evaluations[i].is_enemy_head) || evaluations[i].is_enemy_tail) {
        safe_moves_out[i] = true;
        fallback_found = true;
      }
    }

    // Fallback: Allow all accessible routes.
    if (!fallback_found) {
      for (int i = 0; i < 4; ++i) {
        if (evaluations[i].is_accessible) {
          safe_moves_out[i] = true;
          fallback_found = true;
        }
      }

      // Fallback: Force activation across all moves.
      if (!fallback_found) {
        for (int i = 0; i < 4; ++i) {
          safe_moves_out[i] = true;
        }
      }
    }
  }
}

}  // namespace hebi
