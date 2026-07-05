#include "../../header/hebi/processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hebi {

StateProcessor::StateProcessor(const char* state_json) {
  auto j = nlohmann::json::parse(state_json);
  game_state_ = j.get<hebi::GameState>();
}

extern "C" {
int get_board_size_cpp() { return StateProcessor::BOARD_SIZE; }

void process_state_cpp(const char* state_json, bool* safe_moves_out, float* features_out) {
  StateProcessor processor(state_json);

  if (safe_moves_out != nullptr) {
    processor.extract_safe_moves(safe_moves_out);
  }

  if (features_out != nullptr) {
    processor.encode_policy_features(features_out);
  }
}
}

}  // namespace hebi
