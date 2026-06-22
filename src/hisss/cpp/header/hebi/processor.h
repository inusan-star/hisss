#ifndef HEBI_PROCESSOR_H_
#define HEBI_PROCESSOR_H_

#include <nlohmann/json.hpp>

#include "../../header/hebi/types.h"

namespace hebi {

class StateProcessor {
 public:
  static constexpr int BOARD_SIZE = 15;
  static constexpr int VIEW_RADIUS = 5;

  explicit StateProcessor(const char* state_json);

  void extract_safe_moves(bool* safe_moves_out) const;
  void encode_features(float* features_out) const;

 private:
  hebi::GameState game_state_;
};

extern "C" {
// Get the board size.
int get_board_size_cpp();

// Process the game state.
void process_state_cpp(const char* state_json, bool* safe_moves_out, float* features_out);
}

}  // namespace hebi

#endif
