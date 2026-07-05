#include "../../header/hebi/classical.h"

#include <algorithm>
#include <vector>

#include "../../header/hebi/processor.h"

namespace hebi {

ClassicalAgent::ClassicalAgent() : turn_(0), rng_(42) {}

ClassicalAgent::~ClassicalAgent() {}

void ClassicalAgent::start(const char* state_json) { turn_ = 0; }

int ClassicalAgent::move(const char* state_json) {
  turn_++;

  // Update current game state.
  auto j = nlohmann::json::parse(state_json);
  game_state_ = j.get<hebi::GameState>();

  return 0;
}

void ClassicalAgent::end(const char* state_json) { turn_ = 0; }

extern "C" {
ClassicalAgent* create_classical_agent_cpp() { return new ClassicalAgent(); }

void destroy_classical_agent_cpp(ClassicalAgent* agent) {
  if (agent != nullptr) {
    delete agent;
  }
}

void start_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    agent->start(state_json);
  }
}

int move_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    return agent->move(state_json);
  }
  return 0;
}

void end_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    agent->end(state_json);
  }
}
}

}  // namespace hebi