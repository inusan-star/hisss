#include "../../header/hebi/pbrs.h"

#include <algorithm>
#include <cmath>

namespace hebi {

extern "C" {

void compute_pbrs_cpp(int num_snakes, const int32_t* snake_lengths, const int32_t* snake_heads_x, const int32_t* snake_heads_y,
                      const bool* snake_alive, const bool* obstacle_grid, float* pbrs_out) {
  constexpr int BOARD_SIZE = 15;
  constexpr int CELLS_COUNT = BOARD_SIZE * BOARD_SIZE;
  constexpr int MAX_SNAKES = 4;
  constexpr int UNREACHABLE = 9999;

  // Initialize outputs.
  for (int snake_index = 0; snake_index < num_snakes; ++snake_index) {
    pbrs_out[snake_index] = 0.0f;
  }

  if (num_snakes <= 0 || num_snakes > MAX_SNAKES) {
    return;
  }

  // Initialize reach times.
  int reach_times[MAX_SNAKES][CELLS_COUNT];

  for (int snake_index = 0; snake_index < MAX_SNAKES; ++snake_index) {
    std::fill_n(reach_times[snake_index], CELLS_COUNT, UNREACHABLE);
  }

  // Compute independent reach times.
  for (int snake_index = 0; snake_index < num_snakes; ++snake_index) {
    if (!snake_alive[snake_index]) {
      continue;
    }

    const int head_x = snake_heads_x[snake_index];
    const int head_y = snake_heads_y[snake_index];

    if (head_x < 0 || head_x >= BOARD_SIZE || head_y < 0 || head_y >= BOARD_SIZE) {
      continue;
    }

    int queue[CELLS_COUNT];
    int queue_start = 0;
    int queue_end = 0;

    const int head_flat = head_y * BOARD_SIZE + head_x;
    reach_times[snake_index][head_flat] = 0;
    queue[queue_end++] = head_flat;

    while (queue_start < queue_end) {
      const int current_flat = queue[queue_start++];
      const int current_x = current_flat % BOARD_SIZE;
      const int current_y = current_flat / BOARD_SIZE;
      const int next_time = reach_times[snake_index][current_flat] + 1;

      if (current_x < BOARD_SIZE - 1) {
        const int neighbor_flat = current_flat + 1;

        if (!obstacle_grid[neighbor_flat] && reach_times[snake_index][neighbor_flat] == UNREACHABLE) {
          reach_times[snake_index][neighbor_flat] = next_time;
          queue[queue_end++] = neighbor_flat;
        }
      }

      if (current_x > 0) {
        const int neighbor_flat = current_flat - 1;

        if (!obstacle_grid[neighbor_flat] && reach_times[snake_index][neighbor_flat] == UNREACHABLE) {
          reach_times[snake_index][neighbor_flat] = next_time;
          queue[queue_end++] = neighbor_flat;
        }
      }

      if (current_y < BOARD_SIZE - 1) {
        const int neighbor_flat = current_flat + BOARD_SIZE;

        if (!obstacle_grid[neighbor_flat] && reach_times[snake_index][neighbor_flat] == UNREACHABLE) {
          reach_times[snake_index][neighbor_flat] = next_time;
          queue[queue_end++] = neighbor_flat;
        }
      }

      if (current_y > 0) {
        const int neighbor_flat = current_flat - BOARD_SIZE;

        if (!obstacle_grid[neighbor_flat] && reach_times[snake_index][neighbor_flat] == UNREACHABLE) {
          reach_times[snake_index][neighbor_flat] = next_time;
          queue[queue_end++] = neighbor_flat;
        }
      }
    }
  }

  // Count unique Voronoi ownership.
  int voronoi_areas[MAX_SNAKES] = {0, 0, 0, 0};

  for (int cell_index = 0; cell_index < CELLS_COUNT; ++cell_index) {
    int minimum_time = UNREACHABLE;
    int minimum_owner = -1;
    bool is_tied = false;

    for (int snake_index = 0; snake_index < num_snakes; ++snake_index) {
      if (!snake_alive[snake_index]) {
        continue;
      }

      const int arrival_time = reach_times[snake_index][cell_index];

      if (arrival_time < minimum_time) {
        minimum_time = arrival_time;
        minimum_owner = snake_index;
        is_tied = false;
      } else if (arrival_time == minimum_time && arrival_time < UNREACHABLE) {
        is_tied = true;
      }
    }

    if (minimum_owner >= 0 && !is_tied) {
      voronoi_areas[minimum_owner]++;
    }
  }

  // Compute potentials.
  for (int snake_index = 0; snake_index < num_snakes; ++snake_index) {
    if (!snake_alive[snake_index]) {
      continue;
    }

    const float length = static_cast<float>(std::max(1, snake_lengths[snake_index]));
    const float voronoi_area = static_cast<float>(voronoi_areas[snake_index]);

    pbrs_out[snake_index] = std::log1p(voronoi_area) - std::log(length);
  }
}
}

}  // namespace hebi
