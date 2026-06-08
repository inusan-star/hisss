#pragma once

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace hebi {

// --------------------------------------------------
// Misc Models
// --------------------------------------------------

// 2D coordinate on the board.
struct Point {
  int x;
  int y;
};

// Board food item.
struct Food : public Point {
  int spawn_turn;
};

// Reason for snake elimination.
enum class EliminatedCause {
  EliminatedByCollision,
  EliminatedBySelfCollision,
  EliminatedByOutOfHealth,
  EliminatedByHeadToHeadCollision,
  EliminatedByOutOfBounds
};

// Elimination event details.
struct EliminationEvent {
  EliminatedCause cause;
  int turn;
  std::optional<std::string> by = std::nullopt;
};

// --------------------------------------------------
// Snake Model
// --------------------------------------------------

// Snake visual styling.
struct SnakeCustomizations {
  std::tuple<int, int, int> color;
  std::optional<std::string> head = std::nullopt;
  std::optional<std::string> tail = std::nullopt;
};

// Snake state details.
struct Snake {
  std::string id;
  std::string name;
  int length;
  std::optional<std::string> latency = std::nullopt;
  std::optional<std::string> squad = std::nullopt;
  std::optional<int> health = std::nullopt;
  std::optional<Point> head = std::nullopt;
  std::vector<std::optional<Point>> body;
  SnakeCustomizations customizations;
  std::optional<EliminationEvent> elimination_event = std::nullopt;
};

// --------------------------------------------------
// Game & Ruleset Models
// --------------------------------------------------

// Royale mode configuration.
struct RoyaleSettings {
  int shrinkEveryNTurns;
};

// Squad mode configuration.
struct SquadSettings {
  bool allowBodyCollisions;
  bool sharedElimination;
  bool sharedHealth;
  bool sharedLength;
};

// Game rules configuration.
struct RulesetSettings {
  int foodSpawnChance;
  int hazardDamagePerTurn;
  int minimumFood;
  std::optional<int> viewRadius = std::nullopt;
  RoyaleSettings royale;
  SquadSettings squad;
};

// Ruleset metadata.
struct Ruleset {
  std::string name;
  std::string version;
  RulesetSettings settings;
};

// Game instance metadata.
struct Game {
  std::string id;
  std::string source;
  int timeout;
  Ruleset ruleset;
};

// --------------------------------------------------
// Board & Root GameState Models
// --------------------------------------------------

// Current board state.
struct Board {
  int height;
  int width;
  std::vector<Food> food;
  std::vector<Point> hazards;
  std::vector<Snake> snakes;
};

// Full game state for a turn.
struct GameState {
  int turn;
  Game game;
  Board board;
  Snake you;
};

// --------------------------------------------------
// Snake Action Model
// --------------------------------------------------

// Movement directions.
enum class Direction { UP, RIGHT, DOWN, LEFT };

// Get coordinate shift for direction.
inline std::pair<int, int> board_delta(Direction dir) {
  switch (dir) {
    case Direction::DOWN:
      return {0, -1};
    case Direction::UP:
      return {0, 1};
    case Direction::LEFT:
      return {-1, 0};
    case Direction::RIGHT:
      return {1, 0};
  }
  return {0, 0};
}

// Create direction from coordinate shift.
inline std::optional<Direction> from_board_delta(int dx, int dy) {
  if (dx == 0 && dy == -1) return Direction::DOWN;
  if (dx == 0 && dy == 1) return Direction::UP;
  if (dx == -1 && dy == 0) return Direction::LEFT;
  if (dx == 1 && dy == 0) return Direction::RIGHT;
  return std::nullopt;
}

// Get X-axis delta.
inline int dx(Direction dir) { return board_delta(dir).first; }

// Get Y-axis delta.
inline int dy(Direction dir) { return board_delta(dir).second; }

// Action to perform.
struct MoveAction {
  Direction move;
};

// --------------------------------------------------
// JSON Serialization
// --------------------------------------------------

NLOHMANN_JSON_SERIALIZE_ENUM(EliminatedCause, {{EliminatedCause::EliminatedByCollision, "snake-collision"},
                                               {EliminatedCause::EliminatedBySelfCollision, "snake-self-collision"},
                                               {EliminatedCause::EliminatedByOutOfHealth, "out-of-health"},
                                               {EliminatedCause::EliminatedByHeadToHeadCollision, "head-collision"},
                                               {EliminatedCause::EliminatedByOutOfBounds, "wall-collision"}})

NLOHMANN_JSON_SERIALIZE_ENUM(Direction, {{Direction::UP, "up"}, {Direction::RIGHT, "right"}, {Direction::DOWN, "down"}, {Direction::LEFT, "left"}})

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Food, x, y, spawn_turn)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EliminationEvent, cause, turn, by)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SnakeCustomizations, color, head, tail)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Snake, id, name, length, latency, squad, health, head, body, customizations, elimination_event)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RoyaleSettings, shrinkEveryNTurns)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SquadSettings, allowBodyCollisions, sharedElimination, sharedHealth, sharedLength)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RulesetSettings, foodSpawnChance, hazardDamagePerTurn, minimumFood, viewRadius, royale, squad)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Ruleset, name, version, settings)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Game, id, source, timeout, ruleset)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Board, height, width, food, hazards, snakes)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GameState, turn, game, board, you)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MoveAction, move)

}  // namespace hebi