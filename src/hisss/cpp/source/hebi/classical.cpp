#include "../../header/hebi/classical.h"

#include <algorithm>
#include <vector>

namespace hebi {

ClassicalAgent::ClassicalAgent() : turn_(0), rng_(42) {}

ClassicalAgent::~ClassicalAgent() {}

void ClassicalAgent::start(const char* state_json) { turn_ = 0; }

ClassicalAgent::Board ClassicalAgent::parse_board() const {
  Board board;

  for (int system_x = 0; system_x < SYSTEM_BOARD_SIZE; ++system_x) {
    board.grid[0][system_x] = Cell(Category::WALL, Owner::NONE, Segment::NONE);
    board.grid[SYSTEM_BOARD_SIZE - 1][system_x] = Cell(Category::WALL, Owner::NONE, Segment::NONE);
  }

  for (int system_y = 1; system_y <= GAME_BOARD_SIZE; ++system_y) {
    board.grid[system_y][0] = Cell(Category::WALL, Owner::NONE, Segment::NONE);
    board.grid[system_y][SYSTEM_BOARD_SIZE - 1] = Cell(Category::WALL, Owner::NONE, Segment::NONE);
  }

  if (!game_state_.you.elimination_event.has_value()) {
    hebi::Point head_point = game_state_.you.head.value();

    for (int system_y = 1; system_y <= GAME_BOARD_SIZE; ++system_y) {
      int game_y = system_y - 1;
      int delta_y = std::abs(game_y - head_point.y);

      for (int system_x = 1; system_x <= GAME_BOARD_SIZE; ++system_x) {
        int game_x = system_x - 1;
        int delta_x = std::abs(game_x - head_point.x);

        if (delta_x + delta_y <= VIEW_RADIUS) {
          board.grid[system_y][system_x] = Cell(Category::EMPTY, Owner::NONE, Segment::NONE);
        }
      }
    }
  }

  for (const auto& food_item : game_state_.board.food) {
    int system_x = food_item.x + 1;
    int system_y = food_item.y + 1;
    board.grid[system_y][system_x].category = Category::FOOD;
  }

  for (const auto& snake_item : game_state_.board.snakes) {
    if (snake_item.elimination_event.has_value()) {
      continue;
    }

    Owner owner_type = (snake_item.id == game_state_.you.id) ? Owner::YOU : Owner::ENEMY;
    int segments_count = static_cast<int>(snake_item.body.size());

    if (segments_count == 0) {
      continue;
    }

    for (int segment_index = 0; segment_index < segments_count; ++segment_index) {
      const auto& segment = snake_item.body[segment_index];

      if (!segment.has_value()) {
        continue;
      }

      hebi::Point segment_point = segment.value();
      int system_x = segment_point.x + 1;
      int system_y = segment_point.y + 1;

      Segment segment_type = (segment_index == 0) ? Segment::HEAD : (segment_index == segments_count - 1) ? Segment::TAIL : Segment::BODY;

      board.grid[system_y][system_x] = Cell(Category::SNAKE, owner_type, segment_type);
    }
  }

  return board;
}

int ClassicalAgent::move(const char* state_json) {
  turn_++;

  auto j = nlohmann::json::parse(state_json);
  game_state_ = j.get<hebi::GameState>();

  Board board = parse_board();

  return 0;
}

void ClassicalAgent::end(const char* state_json) { turn_ = 0; }

extern "C" {
// Create classical agent.
ClassicalAgent* create_classical_agent_cpp() { return new ClassicalAgent(); }

// Destroy classical agent.
void destroy_classical_agent_cpp(ClassicalAgent* agent) {
  if (agent != nullptr) {
    delete agent;
  }
}

// Start game.
void start_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    agent->start(state_json);
  }
}

// Generate move.
int move_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    return agent->move(state_json);
  }
  return 0;
}

// End game.
void end_classical_agent_cpp(ClassicalAgent* agent, const char* state_json) {
  if (agent != nullptr) {
    agent->end(state_json);
  }
}
}

}  // namespace hebi
