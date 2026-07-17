#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "../../header/hebi/processor.h"

namespace hebi {

// Board dimensions.
static constexpr int BOARD_SIZE = 15;
static constexpr unsigned U_BOARD_SIZE = static_cast<unsigned>(BOARD_SIZE);
static constexpr int VIEW_RADIUS = 5;
static constexpr int CELLS_COUNT = BOARD_SIZE * BOARD_SIZE;

// Spatial grid.
struct alignas(32) Bitboard {
  uint64_t blocks[4] = {0, 0, 0, 0};

  inline void set(int flat_idx) { blocks[flat_idx >> 6] |= (1ULL << (flat_idx & 63)); }
  inline bool get(int flat_idx) const { return (blocks[flat_idx >> 6] & (1ULL << (flat_idx & 63))) != 0; }
};

// Distance maps.
struct LookupTables {
  uint8_t distance[CELLS_COUNT][CELLS_COUNT];
  uint64_t visibility_bits[CELLS_COUNT][4];

  constexpr LookupTables() : distance{}, visibility_bits{} {
    for (int source = 0; source < CELLS_COUNT; ++source) {
      const int y1 = source / BOARD_SIZE;
      const int x1 = source % BOARD_SIZE;

      for (int target = 0; target < CELLS_COUNT; ++target) {
        const int y2 = target / BOARD_SIZE;
        const int x2 = target % BOARD_SIZE;

        const int dist = (x1 > x2 ? x1 - x2 : x2 - x1) + (y1 > y2 ? y1 - y2 : y2 - y1);
        distance[source][target] = static_cast<uint8_t>(dist);

        if (dist <= VIEW_RADIUS) {
          visibility_bits[source][target >> 6] |= (1ULL << (target & 63));
        }
      }
    }
  }
};

static constexpr LookupTables LUTS = LookupTables();

// Search state.
struct SearchState {
  hebi::Point position;
  int food_eaten;
};

// Move evaluation.
struct MoveEvaluation {
  bool is_accessible;
  bool is_space_sufficient;
  bool is_enemy_head;
  bool is_enemy_tail;
  int reachable_max_space;
};

void StateProcessor::extract_safe_moves(int32_t* memory_map_out, bool* safe_moves_out) const {
  // Reset move outputs.
  safe_moves_out[0] = false;
  safe_moves_out[1] = false;
  safe_moves_out[2] = false;
  safe_moves_out[3] = false;

  // Check player elimination.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  // Cache head position.
  const hebi::Point head = game_state_.you.head.value();
  const int head_flat = head.y * BOARD_SIZE + head.x;

  // Initialize layer grids.
  Bitboard food_grid;
  Bitboard player_body_grid;
  Bitboard enemy_head_grid;
  Bitboard enemy_tail_grid;
  Bitboard obstacles;
  alignas(32) int clear_time_grid[CELLS_COUNT] = {0};

  // Map food items.
  for (const auto& food : game_state_.board.food) {
    if (static_cast<unsigned>(food.x) < U_BOARD_SIZE && static_cast<unsigned>(food.y) < U_BOARD_SIZE) {
      food_grid.set(food.y * BOARD_SIZE + food.x);
    }
  }

  // Map player body for masking.
  {
    const auto& you = game_state_.you;
    const int body_size = static_cast<int>(you.body.size());

    for (int segment_index = 0; segment_index < body_size; ++segment_index) {
      if (you.body[segment_index].has_value()) {
        const hebi::Point segment_pos = you.body[segment_index].value();

        if (static_cast<unsigned>(segment_pos.x) < U_BOARD_SIZE && static_cast<unsigned>(segment_pos.y) < U_BOARD_SIZE) {
          const int flat_index = segment_pos.y * BOARD_SIZE + segment_pos.x;
          player_body_grid.set(flat_index);
        }
      }
    }
  }

  // Define inaccessible cells.
  Bitboard invalid_mask;
  invalid_mask.blocks[0] = LUTS.visibility_bits[head_flat][0] | player_body_grid.blocks[0];
  invalid_mask.blocks[1] = LUTS.visibility_bits[head_flat][1] | player_body_grid.blocks[1];
  invalid_mask.blocks[2] = LUTS.visibility_bits[head_flat][2] | player_body_grid.blocks[2];
  invalid_mask.blocks[3] = LUTS.visibility_bits[head_flat][3] | player_body_grid.blocks[3];

  // Evaluate opponents.
  for (const auto& snake : game_state_.board.snakes) {
    if (snake.id == game_state_.you.id || snake.elimination_event.has_value()) {
      continue;
    }

    // Detect missing segments.
    const int body_size = static_cast<int>(snake.body.size());
    bool has_null_segment = false;

    for (const auto& segment : snake.body) {
      if (!segment.has_value()) {
        has_null_segment = true;
        break;
      }
    }

    const bool is_disadvantaged = (snake.length >= game_state_.you.length);

    // Restrict enemy head neighbors.
    if (snake.body.front().has_value() && (has_null_segment || is_disadvantaged)) {
      const hebi::Point enemy_head = snake.body.front().value();

      const auto add_enemy_head_grid = [&](int direction) __attribute__((always_inline)) {
        const int hx = enemy_head.x + hebi::dx(static_cast<hebi::Direction>(direction));
        const int hy = enemy_head.y + hebi::dy(static_cast<hebi::Direction>(direction));

        if (static_cast<unsigned>(hx) < U_BOARD_SIZE && static_cast<unsigned>(hy) < U_BOARD_SIZE) {
          enemy_head_grid.set(hy * BOARD_SIZE + hx);
        }
      };

      add_enemy_head_grid(0);
      add_enemy_head_grid(1);
      add_enemy_head_grid(2);
      add_enemy_head_grid(3);
    }

    // Locate visible segment bounds.
    int first_valid_index = -1;
    int last_valid_index = -1;

    for (int segment_index = 0; segment_index < body_size; ++segment_index) {
      if (snake.body[segment_index].has_value()) {
        if (first_valid_index == -1) {
          first_valid_index = segment_index;
        }

        last_valid_index = segment_index;
      }
    }

    if (first_valid_index != -1) {
      const hebi::Point first_valid_pos = snake.body[first_valid_index].value();
      hebi::Point current_pos = first_valid_pos;

      // Interpolation state.
      alignas(16) hebi::Point segments_to_write[CELLS_COUNT];
      alignas(16) int logical_indices[CELLS_COUNT];
      int write_count = 0;
      int current_logical_index = 0;

      // Predict hidden enemy head.
      if (!snake.body.front().has_value()) {
        int min_dist_to_you = 9999;
        hebi::Point valid_heads[4];
        int head_count = 0;

        const auto check_virtual_head = [&](int direction) __attribute__((always_inline)) {
          const int hx = first_valid_pos.x + hebi::dx(static_cast<hebi::Direction>(direction));
          const int hy = first_valid_pos.y + hebi::dy(static_cast<hebi::Direction>(direction));

          if (static_cast<unsigned>(hx) < U_BOARD_SIZE && static_cast<unsigned>(hy) < U_BOARD_SIZE) {
            const int flat_index = hy * BOARD_SIZE + hx;

            if (!invalid_mask.get(flat_index)) {
              const int current_dist = LUTS.distance[head_flat][flat_index];

              if (current_dist < min_dist_to_you) {
                min_dist_to_you = current_dist;
                head_count = 0;
                valid_heads[head_count++] = {hx, hy};
              } else if (current_dist == min_dist_to_you) {
                valid_heads[head_count++] = {hx, hy};
              }
            }
          }
        };

        check_virtual_head(0);
        check_virtual_head(1);
        check_virtual_head(2);
        check_virtual_head(3);

        for (int head_index = 0; head_index < head_count; ++head_index) {
          segments_to_write[write_count] = valid_heads[head_index];
          logical_indices[write_count] = current_logical_index;
          write_count++;
        }

        if (head_count > 0) current_pos = valid_heads[0];
      } else {
        segments_to_write[write_count] = current_pos;
        logical_indices[write_count] = current_logical_index;
        write_count++;
      }

      // Pathfinding state.
      Bitboard path_visited;
      alignas(32) int parent_idx[CELLS_COUNT];
      hebi::Point path_queue[CELLS_COUNT];
      alignas(16) hebi::Point reverse_path[CELLS_COUNT];

      // Reconstruct body gaps.
      for (int segment_index = first_valid_index; segment_index <= last_valid_index; ++segment_index) {
        if (snake.body[segment_index].has_value()) {
          const hebi::Point next_target = snake.body[segment_index].value();

          // Check spatial gap.
          if (current_pos.x != next_target.x || current_pos.y != next_target.y) {
            // Reset search state.
            path_visited.blocks[0] = 0;
            path_visited.blocks[1] = 0;
            path_visited.blocks[2] = 0;
            path_visited.blocks[3] = 0;

            int path_queue_start = 0;
            int path_queue_end = 0;

            const int start_flat = current_pos.y * BOARD_SIZE + current_pos.x;
            const int target_flat = next_target.y * BOARD_SIZE + next_target.x;

            path_queue[path_queue_end++] = current_pos;
            path_visited.set(start_flat);

            bool target_found = false;

            // Search adjacent segments.
            while (path_queue_start < path_queue_end && !target_found) {
              const hebi::Point search_pos = path_queue[path_queue_start++];
              const int search_flat = search_pos.y * BOARD_SIZE + search_pos.x;

              // Evaluate neighbors.
              for (int direction = 0; direction < 4; ++direction) {
                const int hx = search_pos.x + hebi::dx(static_cast<hebi::Direction>(direction));
                const int hy = search_pos.y + hebi::dy(static_cast<hebi::Direction>(direction));

                if (static_cast<unsigned>(hx) < U_BOARD_SIZE && static_cast<unsigned>(hy) < U_BOARD_SIZE) {
                  const int neighbor_flat = hy * BOARD_SIZE + hx;

                  if (neighbor_flat == target_flat) {
                    parent_idx[neighbor_flat] = search_flat;
                    target_found = true;
                    break;
                  }

                  if (!path_visited.get(neighbor_flat) && !invalid_mask.get(neighbor_flat)) {
                    path_visited.set(neighbor_flat);
                    parent_idx[neighbor_flat] = search_flat;
                    path_queue[path_queue_end++] = {hx, hy};
                  }
                }
              }
            }

            if (target_found) {
              // Reconstruct gap path.
              hebi::Point trace_pos = next_target;
              int path_length = 0;

              while (trace_pos.x != current_pos.x || trace_pos.y != current_pos.y) {
                reverse_path[path_length++] = trace_pos;
                const int trace_flat = trace_pos.y * BOARD_SIZE + trace_pos.x;
                const int parent_flat = parent_idx[trace_flat];
                trace_pos = {parent_flat % BOARD_SIZE, parent_flat / BOARD_SIZE};
              }

              for (int path_index = path_length - 1; path_index >= 0; --path_index) {
                current_pos = reverse_path[path_index];
                current_logical_index++;
                segments_to_write[write_count] = current_pos;
                logical_indices[write_count] = current_logical_index;
                write_count++;
              }
            }
          }
        }
      }

      // Predict hidden enemy tail.
      if (!snake.body.back().has_value()) {
        int min_dist_to_you = 9999;
        hebi::Point valid_tails[4];
        int tail_count = 0;

        // Evaluate tail candidates.
        const auto check_virtual_tail = [&](int direction) __attribute__((always_inline)) {
          const int tx = current_pos.x + hebi::dx(static_cast<hebi::Direction>(direction));
          const int ty = current_pos.y + hebi::dy(static_cast<hebi::Direction>(direction));

          // Check boundaries.
          if (static_cast<unsigned>(tx) < U_BOARD_SIZE && static_cast<unsigned>(ty) < U_BOARD_SIZE) {
            const int flat_index = ty * BOARD_SIZE + tx;

            // Check cell validity.
            if (!invalid_mask.get(flat_index)) {
              const int current_dist = LUTS.distance[head_flat][flat_index];

              if (current_dist < min_dist_to_you) {
                min_dist_to_you = current_dist;
                tail_count = 0;
                valid_tails[tail_count++] = {tx, ty};
              } else if (current_dist == min_dist_to_you) {
                valid_tails[tail_count++] = {tx, ty};
              }
            }
          }
        };

        check_virtual_tail(0);
        check_virtual_tail(1);
        check_virtual_tail(2);
        check_virtual_tail(3);

        current_logical_index++;

        for (int tail_idx = 0; tail_idx < tail_count; ++tail_idx) {
          segments_to_write[write_count] = valid_tails[tail_idx];
          logical_indices[write_count] = current_logical_index;
          write_count++;
        }
      }

      // Map reconstructed body.
      const int total_logical_steps = current_logical_index + 1;

      for (int write_index = 0; write_index < write_count; ++write_index) {
        const hebi::Point segment_pos = segments_to_write[write_index];

        // Check boundaries.
        if (static_cast<unsigned>(segment_pos.x) < U_BOARD_SIZE && static_cast<unsigned>(segment_pos.y) < U_BOARD_SIZE) {
          const int flat_index = segment_pos.y * BOARD_SIZE + segment_pos.x;
          obstacles.set(flat_index);

          const int time_to_clear = total_logical_steps - logical_indices[write_index];

          // Track segment expiration.
          if (time_to_clear > clear_time_grid[flat_index]) {
            clear_time_grid[flat_index] = time_to_clear;
          }

          if (logical_indices[write_index] == current_logical_index) {
            enemy_tail_grid.set(flat_index);
          }
        }
      }

      // Propagate memory map.
      if (write_count > 1) {
        hebi::Point current_position = segments_to_write[0];
        int current_flat_index = current_position.y * BOARD_SIZE + current_position.x;
        bool is_current_visible = (LUTS.visibility_bits[head_flat][current_flat_index >> 6] & (1ULL << (current_flat_index & 63))) != 0;

        for (int write_index = 0; write_index < write_count - 1; ++write_index) {
          const hebi::Point next_position = segments_to_write[write_index + 1];
          const int next_flat_index = next_position.y * BOARD_SIZE + next_position.x;
          const bool is_next_visible = (LUTS.visibility_bits[head_flat][next_flat_index >> 6] & (1ULL << (next_flat_index & 63))) != 0;

          // Detect transition .
          if (is_current_visible && !is_next_visible) {
            int source_memory = memory_map_out[next_flat_index];

            if (source_memory == 0) {
              // Evaluate adjacent cells.
              const hebi::Point check_position = segments_to_write[write_index];
              const auto check_hidden_memory = [&](int direction) __attribute__((always_inline)) {
                const int nx = check_position.x + hebi::dx(static_cast<hebi::Direction>(direction));
                const int ny = check_position.y + hebi::dy(static_cast<hebi::Direction>(direction));

                if (static_cast<unsigned>(nx) < U_BOARD_SIZE && static_cast<unsigned>(ny) < U_BOARD_SIZE) {
                  const int n_flat = ny * BOARD_SIZE + nx;
                  const bool n_visible = (LUTS.visibility_bits[head_flat][n_flat >> 6] & (1ULL << (n_flat & 63))) != 0;

                  if (!n_visible && memory_map_out[n_flat] > source_memory) {
                    source_memory = memory_map_out[n_flat];
                  }
                }
              };

              check_hidden_memory(0);
              check_hidden_memory(1);
              check_hidden_memory(2);
              check_hidden_memory(3);
            }

            // Determine propagation time.
            int propagation_time = source_memory;

            // Propagate through the segment chain.
            for (int trace_index = write_index; trace_index >= 0; --trace_index) {
              const hebi::Point trace_position = segments_to_write[trace_index];
              const int trace_flat_index = trace_position.y * BOARD_SIZE + trace_position.x;

              if (propagation_time > clear_time_grid[trace_flat_index]) {
                clear_time_grid[trace_flat_index] = propagation_time;
              } else {
                break;
              }

              propagation_time++;
            }
          }

          is_current_visible = is_next_visible;
        }
      }
    }
  }

  // Sync memory map.
  {
    for (int cell_index = 0; cell_index < CELLS_COUNT; ++cell_index) {
      const bool is_visible = (LUTS.visibility_bits[head_flat][cell_index >> 6] & (1ULL << (cell_index & 63))) != 0;
      int final_time = clear_time_grid[cell_index];

      if (!is_visible) {
        int past_memory = memory_map_out[cell_index];
        past_memory = (past_memory > 0) ? past_memory - 1 : 0;

        if (past_memory > final_time) {
          final_time = past_memory;
        }
      }

      memory_map_out[cell_index] = final_time;

      if (final_time > 0) {
        obstacles.set(cell_index);
        clear_time_grid[cell_index] = final_time;
      }
    }
  }

  // Map player body for obstacles.
  {
    const auto& you = game_state_.you;
    const int body_size = static_cast<int>(you.body.size());
    const bool is_stacked = (body_size < you.length);

    for (int segment_index = 0; segment_index < body_size; ++segment_index) {
      if (you.body[segment_index].has_value()) {
        const hebi::Point segment_pos = you.body[segment_index].value();

        if (static_cast<unsigned>(segment_pos.x) < U_BOARD_SIZE && static_cast<unsigned>(segment_pos.y) < U_BOARD_SIZE) {
          const int flat_index = segment_pos.y * BOARD_SIZE + segment_pos.x;

          if ((segment_index < body_size - 1) || is_stacked) {
            obstacles.set(flat_index);
            clear_time_grid[flat_index] = you.length - segment_index;
          }
        }
      }
    }
  }

  // Move evaluation state.
  alignas(32) int step_grid[CELLS_COUNT] = {0};
  SearchState queue[CELLS_COUNT];

  // Reset evaluations.
  MoveEvaluation evaluations[4];

  for (int move_idx = 0; move_idx < 4; ++move_idx) {
    evaluations[move_idx] = {false, false, false, false, 0};
  }

  // Evaluate directional moves.
  for (int move_idx = 0; move_idx < 4; ++move_idx) {
    const hebi::Direction dir = static_cast<hebi::Direction>(move_idx);
    const int next_x = head.x + hebi::dx(dir);
    const int next_y = head.y + hebi::dy(dir);

    // Check basic collisions.
    if (static_cast<unsigned>(next_x) < U_BOARD_SIZE && static_cast<unsigned>(next_y) < U_BOARD_SIZE) {
      const int target_idx = next_y * BOARD_SIZE + next_x;
      evaluations[move_idx].is_accessible = !obstacles.get(target_idx);
      evaluations[move_idx].is_enemy_head = enemy_head_grid.get(target_idx);
      evaluations[move_idx].is_enemy_tail = enemy_tail_grid.get(target_idx);

      if (evaluations[move_idx].is_accessible) {
        // Reset search state.
        Bitboard path_visited;
        path_visited.blocks[0] = 0;
        path_visited.blocks[1] = 0;
        path_visited.blocks[2] = 0;
        path_visited.blocks[3] = 0;

        int queue_start = 0;
        int queue_end = 0;

        // Initialize spatial search.
        int initial_food = food_grid.get(target_idx) ? 1 : 0;
        queue[queue_end++] = {{next_x, next_y}, initial_food};
        path_visited.set(target_idx);
        step_grid[target_idx] = 1;
        int reachable_count = 0;

        // Detect tail chase.
        bool can_tail_chase = player_body_grid.get(target_idx);

        // Calculate accessible space.
        while (queue_start < queue_end && reachable_count < game_state_.you.length) {
          SearchState current = queue[queue_start++];
          const int current_idx = current.position.y * BOARD_SIZE + current.position.x;
          int current_step = step_grid[current_idx];
          reachable_count++;

          // Explore neighbors.
          for (int neighbor_dir = 0; neighbor_dir < 4; ++neighbor_dir) {
            int fill_x = current.position.x + hebi::dx(static_cast<hebi::Direction>(neighbor_dir));
            int fill_y = current.position.y + hebi::dy(static_cast<hebi::Direction>(neighbor_dir));

            if (static_cast<unsigned>(fill_x) < U_BOARD_SIZE && static_cast<unsigned>(fill_y) < U_BOARD_SIZE) {
              const int neighbor_idx = fill_y * BOARD_SIZE + fill_x;

              if (!path_visited.get(neighbor_idx)) {
                // Calculate segment expiration.
                int effective_clear_time = clear_time_grid[neighbor_idx];

                if (player_body_grid.get(neighbor_idx)) {
                  effective_clear_time += current.food_eaten;
                }

                bool time_blocked = ((current_step + 1) < effective_clear_time);

                // Expand search.
                if (!time_blocked) {
                  // Detect tail chase.
                  if (player_body_grid.get(neighbor_idx)) {
                    can_tail_chase = true;
                  }

                  path_visited.set(neighbor_idx);
                  step_grid[neighbor_idx] = current_step + 1;
                  int next_food_count = current.food_eaten + (food_grid.get(neighbor_idx) ? 1 : 0);
                  queue[queue_end++] = {{fill_x, fill_y}, next_food_count};
                }
              }
            }
          }
        }

        evaluations[move_idx].is_space_sufficient = false;
        evaluations[move_idx].reachable_max_space = reachable_count;

        // Evaluate space sufficiency.
        if (reachable_count >= game_state_.you.length || can_tail_chase) {
          evaluations[move_idx].is_space_sufficient = true;
        }

        // Confirm safe direction.
        if (evaluations[move_idx].is_space_sufficient && !evaluations[move_idx].is_enemy_head) {
          safe_moves_out[move_idx] = true;
        }
      }
    }
  }

  // Check absolute safe moves.
  const bool has_perfect_move = safe_moves_out[0] | safe_moves_out[1] | safe_moves_out[2] | safe_moves_out[3];

  // Execute fallback logic.
  if (!has_perfect_move) {
    bool fallback_found = false;

    // Fallback: Allow risky heads or tails.
    for (int move_idx = 0; move_idx < 4; ++move_idx) {
      const bool condition =
          (evaluations[move_idx].is_accessible && evaluations[move_idx].is_space_sufficient && evaluations[move_idx].is_enemy_head) ||
          evaluations[move_idx].is_enemy_tail;
      safe_moves_out[move_idx] = condition;
      fallback_found |= condition;
    }

    // Fallback: Allow any accessible cell.
    if (!fallback_found) {
      for (int move_idx = 0; move_idx < 4; ++move_idx) {
        const bool condition = evaluations[move_idx].is_accessible;
        safe_moves_out[move_idx] = condition;
        fallback_found |= condition;
      }

      // Fallback: Force all directions.
      if (!fallback_found) {
        safe_moves_out[0] = true;
        safe_moves_out[1] = true;
        safe_moves_out[2] = true;
        safe_moves_out[3] = true;
      }
    }
  }
}

}  // namespace hebi
