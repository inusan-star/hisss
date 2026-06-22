#include "../../header/hebi/processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hebi {

StateProcessor::StateProcessor(const char* state_json) {
  auto j = nlohmann::json::parse(state_json);
  game_state_ = j.get<hebi::GameState>();
}

void StateProcessor::extract_safe_moves(bool* safe_moves_out) const {
  // Reset output array.
  for (int i = 0; i < 4; ++i) {
    safe_moves_out[i] = false;
  }

  // Abort if the player is eliminated.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state_.you.head.value();

  // Identify all occupied grid cells.
  std::vector<std::vector<bool>> obstacles(BOARD_SIZE, std::vector<bool>(BOARD_SIZE, false));
  for (const auto& snake : game_state_.board.snakes) {
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
        if (p.x >= 0 && p.x < BOARD_SIZE && p.y >= 0 && p.y < BOARD_SIZE) {
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
    if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE && !obstacles[ny][nx]) {
      safe_moves_out[i] = true;
    }
  }
}

void StateProcessor::encode_features(float* features_out) const {
  int plane_size = BOARD_SIZE * BOARD_SIZE;

  // Reset output array.
  std::fill(features_out, features_out + 9 * plane_size, 0.0f);

  // Abort if the player is eliminated.
  if (game_state_.you.elimination_event.has_value()) {
    return;
  }

  hebi::Point head = game_state_.you.head.value();

  auto is_visible = [&](int x, int y) { return std::abs(x - head.x) + std::abs(y - head.y) <= VIEW_RADIUS; };

  auto set_features = [&](int c, int x, int y, float val) {
    if (x >= 0 && x < BOARD_SIZE && y >= 0 && y < BOARD_SIZE) {
      features_out[c * plane_size + y * BOARD_SIZE + x] = val;
    }
  };

  // Channel-0, Channel-1, Channel-2: You (Head, Body, Tail)
  int you_len = game_state_.you.body.size();
  for (int i = 0; i < you_len; ++i) {
    if (!game_state_.you.body[i].has_value()) continue;
    hebi::Point p = game_state_.you.body[i].value();

    if (i == 0) {
      set_features(0, p.x, p.y, 1.0f);
    }
    if (i > 0 && i < you_len - 1) {
      set_features(1, p.x, p.y, 1.0f);
    }
    if (i == you_len - 1) {
      set_features(2, p.x, p.y, 1.0f);
    }
  }

  // Channel-3, Channel-4, Channel-5: Enemies (Head, Body, Tail)
  for (const auto& snake : game_state_.board.snakes) {
    if (snake.elimination_event.has_value() || snake.id == game_state_.you.id) {
      continue;
    }

    int body_len = snake.body.size();

    for (int i = 0; i < body_len; ++i) {
      if (!snake.body[i].has_value()) continue;
      hebi::Point p = snake.body[i].value();

      if (is_visible(p.x, p.y)) {
        if (i == 0) {
          set_features(3, p.x, p.y, 1.0f);
        }
        if (i > 0 && i < body_len - 1) {
          set_features(4, p.x, p.y, 1.0f);
        }
        if (i == body_len - 1) {
          set_features(5, p.x, p.y, 1.0f);
        }
      }
    }
  }

  // Channel-6: Food
  for (const auto& food : game_state_.board.food) {
    set_features(6, food.x, food.y, 1.0f);
  }

  // Channel-7: Visibility
  for (int y = 0; y < BOARD_SIZE; ++y) {
    for (int x = 0; x < BOARD_SIZE; ++x) {
      if (is_visible(x, y)) {
        set_features(7, x, y, 1.0f);
      }
    }
  }

  // Channel-8: You Health
  float health_val = static_cast<float>(game_state_.you.health.value_or(0)) / 100.0f;
  for (int y = 0; y < BOARD_SIZE; ++y) {
    for (int x = 0; x < BOARD_SIZE; ++x) {
      set_features(8, x, y, health_val);
    }
  }
}

extern "C" {
int get_board_size_cpp() { return StateProcessor::BOARD_SIZE; }

void process_state_cpp(const char* state_json, bool* safe_moves_out, float* features_out) {
  StateProcessor processor(state_json);

  if (safe_moves_out != nullptr) {
    processor.extract_safe_moves(safe_moves_out);
  }

  if (features_out != nullptr) {
    processor.encode_features(features_out);
  }
}
}

}  // namespace hebi
