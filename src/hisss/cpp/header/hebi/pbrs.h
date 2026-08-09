#ifndef HEBI_PBRS_H_
#define HEBI_PBRS_H_

#include <cstdint>

namespace hebi {

extern "C" {
// Compute PBRS potentials.
void compute_pbrs_cpp(int num_snakes, const int32_t* snake_lengths, const int32_t* snake_heads_x, const int32_t* snake_heads_y,
                      const bool* snake_alive, const bool* obstacle_grid, float* pbrs_out);
}

}  // namespace hebi

#endif
