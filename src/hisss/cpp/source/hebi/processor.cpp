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

void process_state_cpp(int32_t* memory_map_out, const char* partial_state_json, const char* perfect_state_json, bool* safe_moves_out,
                       float* policy_features_out, float* value_features_out) {
  StateProcessor partial_processor(partial_state_json);

  if (memory_map_out != nullptr && safe_moves_out != nullptr) {
    partial_processor.extract_safe_moves(memory_map_out, safe_moves_out);
  }

  if (memory_map_out != nullptr && policy_features_out != nullptr && safe_moves_out != nullptr) {
    partial_processor.encode_policy_features(memory_map_out, policy_features_out, safe_moves_out);
  }

  if (value_features_out != nullptr) {
    StateProcessor perfect_processor(perfect_state_json);
    perfect_processor.encode_value_features(value_features_out);
  }
}
}

}  // namespace hebi
