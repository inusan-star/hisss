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
  bool is_enemy_head;
  bool is_enemy_tail;
  int space_strict;
  int space_normal;
  int space_desperate;
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

  std::vector<hebi::Point> active_enemy_heads;

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

        // Track valid enemy head for Voronoi dominance.
        active_enemy_heads.push_back(first_valid_pos);

        // Buffer initialization.
        alignas(16) hebi::Point segments_to_write[BOARD_SIZE * BOARD_SIZE];
        alignas(16) int logical_indices[BOARD_SIZE * BOARD_SIZE];
        int write_count = 0;
        int current_logical_idx = 0;

        // Virtual head assignment.
        if (!snake.body.front().has_value()) {
          int min_dist_to_you = 9999;
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

            // Gap interpolation.
            if (current_pos.x != next_target.x || current_pos.y != next_target.y) {
              alignas(16) int parent_x[BOARD_SIZE * BOARD_SIZE] = {0};
              alignas(16) int parent_y[BOARD_SIZE * BOARD_SIZE] = {0};
              alignas(16) bool path_visited[BOARD_SIZE * BOARD_SIZE] = {false};

              hebi::Point path_queue[BOARD_SIZE * BOARD_SIZE];
              int path_queue_start = 0;
              int path_queue_end = 0;

              path_queue[path_queue_end++] = current_pos;
              path_visited[current_pos.y * BOARD_SIZE + current_pos.x] = true;

              bool target_found = false;

              // Start BFS traversal.
              while (path_queue_start < path_queue_end && !target_found) {
                hebi::Point search_pos = path_queue[path_queue_start++];

                for (int d = 0; d < 4; ++d) {
                  hebi::Point neighbor_pos = {search_pos.x + hebi::dx(static_cast<hebi::Direction>(d)),
                                              search_pos.y + hebi::dy(static_cast<hebi::Direction>(d))};

                  if (neighbor_pos.x == next_target.x && neighbor_pos.y == next_target.y) {
                    parent_x[neighbor_pos.y * BOARD_SIZE + neighbor_pos.x] = search_pos.x;
                    parent_y[neighbor_pos.y * BOARD_SIZE + neighbor_pos.x] = search_pos.y;
                    target_found = true;
                    break;
                  }

                  if (is_in_bounds(neighbor_pos.x, neighbor_pos.y)) {
                    int neighbor_idx = neighbor_pos.y * BOARD_SIZE + neighbor_pos.x;

                    if (!path_visited[neighbor_idx] && !is_visible(neighbor_pos.x, neighbor_pos.y) && !player_body_grid[neighbor_idx]) {
                      path_visited[neighbor_idx] = true;
                      parent_x[neighbor_idx] = search_pos.x;
                      parent_y[neighbor_idx] = search_pos.y;
                      path_queue[path_queue_end++] = neighbor_pos;
                    }
                  }
                }
              }

              if (target_found) {
                // Reconstruct path.
                hebi::Point trace_pos = next_target;
                hebi::Point reverse_path[BOARD_SIZE * BOARD_SIZE];
                int path_length = 0;

                while (trace_pos.x != current_pos.x || trace_pos.y != current_pos.y) {
                  reverse_path[path_length++] = trace_pos;
                  int trace_idx = trace_pos.y * BOARD_SIZE + trace_pos.x;
                  trace_pos = {parent_x[trace_idx], parent_y[trace_idx]};
                }

                for (int p = path_length - 1; p >= 0; --p) {
                  current_pos = reverse_path[p];
                  current_logical_idx++;
                  segments_to_write[write_count] = current_pos;
                  logical_indices[write_count] = current_logical_idx;
                  write_count++;
                }
              }
            }
          }
        }

        // Virtual tail assignment.
        if (!snake.body.back().has_value()) {
          int min_dist_to_you = 9999;
          hebi::Point valid_tails[4];
          int tail_count = 0;

          for (int d = 0; d < 4; ++d) {
            int tx = last_valid_pos.x + hebi::dx(static_cast<hebi::Direction>(d));
            int ty = last_valid_pos.y + hebi::dy(static_cast<hebi::Direction>(d));

            // Combined bounds, visibility, and player duplicate check.
            if (is_in_bounds(tx, ty) && !is_visible(tx, ty) && !player_body_grid[ty * BOARD_SIZE + tx]) {
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
    evaluations[i] = {false, false, false, 0, 0, 0};
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
        // Multi-tier space evaluation (0: strict, 1: normal, 2: desperate)
        for (int mode = 0; mode < 3; ++mode) {
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
                    bool mode_blocked = false;

                    if (mode == 0) {  // space_strict
                      for (const auto& eh : active_enemy_heads) {
                        int enemy_arrival_turn = get_distance(fx, fy, eh.x, eh.y);
                        int my_arrival_turn = current_step + 1;
                        if (enemy_arrival_turn <= 2 && enemy_arrival_turn <= my_arrival_turn) {
                          mode_blocked = true;
                          break;
                        }
                      }
                      if (!mode_blocked && enemy_head_grid[neighbor_idx]) {
                        mode_blocked = true;
                      }
                    } else if (mode == 1) {  // space_normal
                      if (enemy_head_grid[neighbor_idx]) {
                        mode_blocked = true;
                      }
                    }

                    if (!mode_blocked) {
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
          }

          // Merge tail chase logic into final reachable count.
          if (can_tail_chase) {
            reachable_count = std::max(reachable_count, game_state_.you.length);
          }

          if (mode == 0)
            evaluations[i].space_strict = reachable_count;
          else if (mode == 1)
            evaluations[i].space_normal = reachable_count;
          else if (mode == 2)
            evaluations[i].space_desperate = reachable_count;
        }
      }
    }
  }

  // Execute multi-tier fallback decision tree.

  // Tier 1: Perfect safe (Strict Safe)
  bool has_tier1 = false;
  for (int i = 0; i < 4; ++i) {
    if (evaluations[i].is_accessible && evaluations[i].space_strict >= game_state_.you.length && !evaluations[i].is_enemy_head) {
      safe_moves_out[i] = true;
      has_tier1 = true;
    }
  }

  if (!has_tier1) {
    // Tier 2: Normal safe
    bool has_tier2 = false;
    for (int i = 0; i < 4; ++i) {
      if (evaluations[i].is_accessible && evaluations[i].space_normal >= game_state_.you.length && !evaluations[i].is_enemy_head) {
        safe_moves_out[i] = true;
        has_tier2 = true;
      }
    }

    if (!has_tier2) {
      // Tier 3: Best depth survival
      int max_space = -1;
      for (int i = 0; i < 4; ++i) {
        if (evaluations[i].is_accessible) {
          int space = std::max(evaluations[i].space_strict, evaluations[i].space_normal);
          if (space > max_space) {
            max_space = space;
          }
        }
      }

      bool has_tier3 = false;
      if (max_space > 0) {
        for (int i = 0; i < 4; ++i) {
          if (evaluations[i].is_accessible && std::max(evaluations[i].space_strict, evaluations[i].space_normal) == max_space) {
            safe_moves_out[i] = true;
            has_tier3 = true;
          }
        }
      }

      if (!has_tier3) {
        // Tier 4: Risk take / desperate survival
        bool has_tier4 = false;
        for (int i = 0; i < 4; ++i) {
          if (evaluations[i].is_accessible && evaluations[i].space_desperate > 0) {
            safe_moves_out[i] = true;
            has_tier4 = true;
          }
        }

        // Ultimate Fallback: Allow all accessible routes, or force activation if completely boxed in.
        if (!has_tier4) {
          bool any_set = false;
          for (int i = 0; i < 4; ++i) {
            if (evaluations[i].is_accessible) {
              safe_moves_out[i] = true;
              any_set = true;
            }
          }
          if (!any_set) {
            for (int i = 0; i < 4; ++i) {
              safe_moves_out[i] = true;
            }
          }
        }
      }
    }
  }
}

}  // namespace hebi
