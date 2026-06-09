#include "../../header/hebi/mask.h"

#include <nlohmann/json.hpp>

#include "../../header/hebi/types.h"

extern "C" {

void get_safe_moves_cpp(const char* state_json, bool* safe_moves_out) {
  // Reset output array.
  for (int i = 0; i < 4; ++i) {
    safe_moves_out[i] = false;
  }

  auto j = nlohmann::json::parse(state_json);
  hebi::GameState game_state = j.get<hebi::GameState>();

  // Abort if the player is eliminated.
  if (game_state.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state.you.head.value();
  int width = game_state.board.width;
  int height = game_state.board.height;

  // Identify all occupied grid cells.
  std::vector<std::vector<bool>> obstacles(height, std::vector<bool>(width, false));
  for (const auto& snake : game_state.board.snakes) {
    // Skip eliminated snakes.
    if (snake.elimination_event.has_value()) {
      continue;
    }

    // Mark body segments as obstacles.
    int check_len = snake.body.size() > 1 ? snake.body.size() - 1 : snake.body.size();
    for (int i = 0; i < check_len; ++i) {
      if (snake.body[i].has_value()) {
        hebi::Point p = snake.body[i].value();
        // Skip invalid coordinates.
        if (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height) {
          obstacles[p.y][p.x] = true;
        }
      }
    }
  }

  // Check safety of adjacent cells.
  for (int i = 0; i < 4; ++i) {
    hebi::Direction dir = static_cast<hebi::Direction>(i);
    int nx = head.x + hebi::dx(dir);
    int ny = head.y + hebi::dy(dir);

    // Verify bounds and occupancy.
    if (nx >= 0 && nx < width && ny >= 0 && ny < height && !obstacles[ny][nx]) {
      safe_moves_out[i] = true;
    }
  }
}

}  // extern "C"