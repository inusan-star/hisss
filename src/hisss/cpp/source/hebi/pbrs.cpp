#include "../../header/hebi/pbrs.h"

#include <algorithm>
#include <cmath>

namespace hebi {

extern "C" {

void compute_pbrs_cpp(int num_snakes, const int32_t* snake_lengths, const int32_t* snake_heads_x, const int32_t* snake_heads_y,
                      const bool* snake_alive, const bool* obstacle_grid, float* pbrs_out) {
  constexpr int BOARD_SIZE = 15;
  constexpr int CELLS_COUNT = 225;

  // Init.
  int owner_grid[CELLS_COUNT];
  int reach_time[CELLS_COUNT];
  std::fill_n(owner_grid, CELLS_COUNT, -1);
  std::fill_n(reach_time, CELLS_COUNT, 9999);

  int queue_flat[CELLS_COUNT];
  int queue_id[CELLS_COUNT];
  int queue_time[CELLS_COUNT];

  int q_start = 0;
  int q_end = 0;

  // Setup heads.
  for (int i = 0; i < num_snakes; ++i) {
    if (snake_alive[i]) {
      int flat_idx = snake_heads_y[i] * BOARD_SIZE + snake_heads_x[i];
      queue_flat[q_end] = flat_idx;
      queue_id[q_end] = i;
      queue_time[q_end] = 0;
      q_end++;

      reach_time[flat_idx] = 0;
      owner_grid[flat_idx] = i;
    }
  }

  // BFS.
  while (q_start < q_end) {
    int c_flat = queue_flat[q_start];
    int cid = queue_id[q_start];
    int ctime = queue_time[q_start];
    q_start++;

    int cx = c_flat % BOARD_SIZE;
    int cy = c_flat / BOARD_SIZE;
    int ntime = ctime + 1;

    // RIGHT.
    if (cx < BOARD_SIZE - 1) {
      int n_flat = c_flat + 1;

      if (!obstacle_grid[n_flat]) {
        if (ntime < reach_time[n_flat]) {
          reach_time[n_flat] = ntime;
          owner_grid[n_flat] = cid;
          queue_flat[q_end] = n_flat;
          queue_id[q_end] = cid;
          queue_time[q_end] = ntime;
          q_end++;
        } else if (ntime == reach_time[n_flat] && owner_grid[n_flat] != cid) {
          owner_grid[n_flat] = -2;
        }
      }
    }

    // LEFT.
    if (cx > 0) {
      int n_flat = c_flat - 1;

      if (!obstacle_grid[n_flat]) {
        if (ntime < reach_time[n_flat]) {
          reach_time[n_flat] = ntime;
          owner_grid[n_flat] = cid;
          queue_flat[q_end] = n_flat;
          queue_id[q_end] = cid;
          queue_time[q_end] = ntime;
          q_end++;
        } else if (ntime == reach_time[n_flat] && owner_grid[n_flat] != cid) {
          owner_grid[n_flat] = -2;
        }
      }
    }

    // DOWN.
    if (cy < BOARD_SIZE - 1) {
      int n_flat = c_flat + BOARD_SIZE;

      if (!obstacle_grid[n_flat]) {
        if (ntime < reach_time[n_flat]) {
          reach_time[n_flat] = ntime;
          owner_grid[n_flat] = cid;
          queue_flat[q_end] = n_flat;
          queue_id[q_end] = cid;
          queue_time[q_end] = ntime;
          q_end++;
        } else if (ntime == reach_time[n_flat] && owner_grid[n_flat] != cid) {
          owner_grid[n_flat] = -2;
        }
      }
    }

    // UP.
    if (cy > 0) {
      int n_flat = c_flat - BOARD_SIZE;

      if (!obstacle_grid[n_flat]) {
        if (ntime < reach_time[n_flat]) {
          reach_time[n_flat] = ntime;
          owner_grid[n_flat] = cid;
          queue_flat[q_end] = n_flat;
          queue_id[q_end] = cid;
          queue_time[q_end] = ntime;
          q_end++;
        } else if (ntime == reach_time[n_flat] && owner_grid[n_flat] != cid) {
          owner_grid[n_flat] = -2;
        }
      }
    }
  }

  // Count.
  int voronoi_areas[4] = {0, 0, 0, 0};

  for (int i = 0; i < CELLS_COUNT; ++i) {
    int oid = owner_grid[i];
    if (oid >= 0 && oid < num_snakes) {
      voronoi_areas[oid]++;
    }
  }

  // Compute.
  for (int i = 0; i < num_snakes; ++i) {
    if (snake_alive[i]) {
      float length = static_cast<float>(std::max(1, snake_lengths[i]));
      float v_area = static_cast<float>(voronoi_areas[i]);
      pbrs_out[i] = std::log(1.0f + v_area) - std::log(length);
    } else {
      pbrs_out[i] = 0.0f;
    }
  }
}
}

}  // namespace hebi
