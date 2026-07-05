#ifndef HEBI_CLASSICAL_H_
#define HEBI_CLASSICAL_H_

#include <nlohmann/json.hpp>
#include <random>

#include "../../header/hebi/types.h"

namespace hebi {

class ClassicalAgent {
 public:
  static constexpr int BOARD_SIZE = 15;
  static constexpr int VIEW_RADIUS = 5;

  ClassicalAgent();
  ~ClassicalAgent();

  void start(const char* state_json);
  int move(const char* state_json);
  void end(const char* state_json);

 private:
  int turn_;
  std::mt19937 rng_;
  hebi::GameState game_state_;
};

extern "C" {
// Create classical agent.
ClassicalAgent* create_classical_agent_cpp();

// Destroy classical agent.
void destroy_classical_agent_cpp(ClassicalAgent* agent);

// Start game.
void start_classical_agent_cpp(ClassicalAgent* agent, const char* state_json);

// Generate move.
int move_classical_agent_cpp(ClassicalAgent* agent, const char* state_json);

// End game.
void end_classical_agent_cpp(ClassicalAgent* agent, const char* state_json);
}

}  // namespace hebi

#endif
