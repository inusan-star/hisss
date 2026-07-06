#ifndef HEBI_CLASSICAL_H_
#define HEBI_CLASSICAL_H_

#include <nlohmann/json.hpp>
#include <random>
#include <vector>

#include "../../header/hebi/types.h"

namespace hebi {

enum class Category : uint8_t { EMPTY = 0, WALL = 1, FOOD = 2, SNAKE = 3, UNKNOWN = 4 };
enum class Owner : uint8_t { NONE = 0, YOU = 1, ENEMY = 2 };
enum class Segment : uint8_t { NONE = 0, HEAD = 1, BODY = 2, TAIL = 3 };

struct Cell {
  Category category : 3;
  Owner owner : 2;
  Segment segment : 3;

  constexpr Cell() : category(Category::UNKNOWN), owner(Owner::NONE), segment(Segment::NONE) {}
  constexpr Cell(Category category, Owner owner, Segment segment) : category(category), owner(owner), segment(segment) {}
};

class ClassicalAgent {
 public:
  static constexpr int GAME_BOARD_SIZE = 15;
  static constexpr int SYSTEM_BOARD_SIZE = 17;
  static constexpr int VIEW_RADIUS = 5;

  struct Board {
    Cell grid[SYSTEM_BOARD_SIZE][SYSTEM_BOARD_SIZE];
  };

  ClassicalAgent();
  ~ClassicalAgent();

  void start(const char* state_json);
  int move(const char* state_json);
  void end(const char* state_json);

 private:
  Board parse_board() const;

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
