#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <optional>
#include <vector>

#include "../../header/hebi/processor.h"

namespace hebi {

static constexpr uint32_t MEMORY_DATA_MASK = 0x0001FFFFu;
static constexpr uint32_t ENEMY_SLOT_CHUNK_MASK = 0x00007FFFu;
static constexpr uint32_t ENEMY_SLOT_KEY_MASK = 0x3FFFFFFFu;
static constexpr int ENEMY_SLOT_SHIFT = 17;
static constexpr int ENEMY_SLOT_COUNT = 3;
static constexpr int ENEMY_SLOT_CELLS = 2;

static uint32_t get_enemy_slot_key(const hebi::Snake& snake) {
  uint32_t hash = 2166136261u;

  for (const unsigned char character : snake.id) {
    hash ^= character;
    hash *= 16777619u;
  }

  hash &= ENEMY_SLOT_KEY_MASK;

  return hash == 0 ? 1u : hash;
}

static uint32_t read_enemy_slot_key(const int32_t* memory_map, int slot) {
  const int first_cell = slot * ENEMY_SLOT_CELLS;
  const uint32_t lower_chunk = (static_cast<uint32_t>(memory_map[first_cell]) >> ENEMY_SLOT_SHIFT) & ENEMY_SLOT_CHUNK_MASK;
  const uint32_t upper_chunk = (static_cast<uint32_t>(memory_map[first_cell + 1]) >> ENEMY_SLOT_SHIFT) & ENEMY_SLOT_CHUNK_MASK;

  return lower_chunk | (upper_chunk << 15);
}

static void write_enemy_slot_key(int32_t* memory_map, int slot, uint32_t key) {
  const int first_cell = slot * ENEMY_SLOT_CELLS;

  const uint32_t lower_chunk = key & ENEMY_SLOT_CHUNK_MASK;
  const uint32_t upper_chunk = (key >> 15) & ENEMY_SLOT_CHUNK_MASK;

  memory_map[first_cell] =
      static_cast<int32_t>((static_cast<uint32_t>(memory_map[first_cell]) & MEMORY_DATA_MASK) | (lower_chunk << ENEMY_SLOT_SHIFT));
  memory_map[first_cell + 1] =
      static_cast<int32_t>((static_cast<uint32_t>(memory_map[first_cell + 1]) & MEMORY_DATA_MASK) | (upper_chunk << ENEMY_SLOT_SHIFT));
}

static std::array<const hebi::Snake*, ENEMY_SLOT_COUNT> resolve_enemy_slots(const std::vector<const hebi::Snake*>& enemies, int32_t* memory_map,
                                                                            int turn) {
  std::array<const hebi::Snake*, ENEMY_SLOT_COUNT> enemy_slots = {nullptr, nullptr, nullptr};
  uint32_t enemy_slot_keys[ENEMY_SLOT_COUNT] = {
      read_enemy_slot_key(memory_map, 0),
      read_enemy_slot_key(memory_map, 1),
      read_enemy_slot_key(memory_map, 2),
  };

  bool has_initialized_slot = false;

  for (int slot = 0; slot < ENEMY_SLOT_COUNT; ++slot) {
    if (enemy_slot_keys[slot] != 0) {
      has_initialized_slot = true;
      break;
    }
  }

  if (turn == 0 || !has_initialized_slot) {
    for (int slot = 0; slot < ENEMY_SLOT_COUNT; ++slot) {
      const uint32_t key = slot < static_cast<int>(enemies.size()) ? get_enemy_slot_key(*enemies[slot]) : 0u;
      write_enemy_slot_key(memory_map, slot, key);
      enemy_slot_keys[slot] = key;
    }
  }

  for (const hebi::Snake* enemy : enemies) {
    const uint32_t enemy_key = get_enemy_slot_key(*enemy);

    for (int slot = 0; slot < ENEMY_SLOT_COUNT; ++slot) {
      if (enemy_slot_keys[slot] == enemy_key) {
        enemy_slots[slot] = enemy;
        break;
      }
    }
  }

  return enemy_slots;
}

void StateProcessor::encode_policy_features(int32_t* memory_map, float* features_out, const bool* safe_moves) {
  try {
    if (game_state_.you.elimination_event.has_value()) {
      return;
    }

    const int plane_size = BOARD_SIZE * BOARD_SIZE;

    auto set_val = [](float* channel_ptr, int x, int y, float v = 1.0f) {
      if (x >= 0 && x < BOARD_SIZE && y >= 0 && y < BOARD_SIZE) {
        channel_ptr[y * BOARD_SIZE + x] = v;
      }
    };

    int ch = 0;

    // Self (8 Channels)
    float* self_head_ptr = features_out + (ch + 0) * plane_size;
    float* self_body_ptr = features_out + (ch + 1) * plane_size;
    float* self_tail_ptr = features_out + (ch + 2) * plane_size;
    float* self_len_ptr = features_out + (ch + 3) * plane_size;
    float* self_hp_ptr = features_out + (ch + 4) * plane_size;
    float* self_dir_x_ptr = features_out + (ch + 5) * plane_size;
    float* self_dir_y_ptr = features_out + (ch + 6) * plane_size;
    float* self_body_age_ptr = features_out + (ch + 7) * plane_size;

    int self_body_len = game_state_.you.body.size();
    float self_dx = 0.0f;
    float self_dy = 0.0f;
    hebi::Point self_head_p;

    for (int i = 0; i < self_body_len; ++i) {
      if (!game_state_.you.body[i].has_value()) continue;
      hebi::Point p = game_state_.you.body[i].value();

      float age = (self_body_len > 1) ? 1.0f - static_cast<float>(i) / (self_body_len - 1) : 1.0f;
      set_val(self_body_age_ptr, p.x, p.y, age);

      if (i == 0) {
        set_val(self_head_ptr, p.x, p.y);
        self_head_p = p;
      }
      if (i == 1) {
        self_dx = static_cast<float>(self_head_p.x - p.x);
        self_dy = static_cast<float>(self_head_p.y - p.y);
      }
      if (i > 0 && i < self_body_len - 1) {
        set_val(self_body_ptr, p.x, p.y);
      }
      if (i == self_body_len - 1) {
        set_val(self_tail_ptr, p.x, p.y);
      }
    }

    std::fill_n(self_len_ptr, plane_size, static_cast<float>(game_state_.you.length) / 100.0f);
    std::fill_n(self_hp_ptr, plane_size, static_cast<float>(game_state_.you.health.value_or(0)) / 100.0f);
    std::fill_n(self_dir_x_ptr, plane_size, self_dx);
    std::fill_n(self_dir_y_ptr, plane_size, self_dy);

    ch += 8;

    // Enemies (24 Channels)
    std::vector<const hebi::Snake*> enemies;
    enemies.reserve(3);

    for (const auto& snake : game_state_.board.snakes) {
      if (snake.id != game_state_.you.id) {
        enemies.push_back(&snake);
      }
    }

    std::sort(enemies.begin(), enemies.end(), [](const hebi::Snake* a, const hebi::Snake* b) { return a->id < b->id; });

    const std::array<const hebi::Snake*, ENEMY_SLOT_COUNT> enemy_slots = resolve_enemy_slots(enemies, memory_map, game_state_.turn);

    float alive_count = 1.0f;
    std::optional<hebi::Point> enemy_heads[3] = {std::nullopt, std::nullopt, std::nullopt};
    bool enemy_fully_visible[3] = {false, false, false};
    bool enemy_tail_visible[3] = {false, false, false};
    bool enemy_body_visible[3] = {false, false, false};

    for (int e = 0; e < 3; ++e) {
      int base_ch = ch + (e * 8);

      float* en_head_ptr = features_out + (base_ch + 0) * plane_size;
      float* en_body_ptr = features_out + (base_ch + 1) * plane_size;
      float* en_tail_ptr = features_out + (base_ch + 2) * plane_size;
      float* en_len_ptr = features_out + (base_ch + 3) * plane_size;
      float* en_dir_x_ptr = features_out + (base_ch + 4) * plane_size;
      float* en_dir_y_ptr = features_out + (base_ch + 5) * plane_size;
      float* en_body_age_ptr = features_out + (base_ch + 6) * plane_size;
      float* en_alive_ptr = features_out + (base_ch + 7) * plane_size;

      const hebi::Snake* enemy = enemy_slots[e];

      if (enemy == nullptr || enemy->elimination_event.has_value()) {
        std::fill_n(en_head_ptr, plane_size, 0.0f);
        std::fill_n(en_body_ptr, plane_size, 0.0f);
        std::fill_n(en_tail_ptr, plane_size, 0.0f);
        std::fill_n(en_len_ptr, plane_size, 0.0f);
        std::fill_n(en_dir_x_ptr, plane_size, 0.0f);
        std::fill_n(en_dir_y_ptr, plane_size, 0.0f);
        std::fill_n(en_body_age_ptr, plane_size, 0.0f);
        std::fill_n(en_alive_ptr, plane_size, 0.0f);
        continue;
      }

      alive_count += 1.0f;

      int en_body_len = enemy->body.size();
      float en_dx = 0.0f;
      float en_dy = 0.0f;
      bool has_head = false;
      hebi::Point en_head_p;
      bool fully_visible = (en_body_len > 0);

      for (int i = 0; i < en_body_len; ++i) {
        if (!enemy->body[i].has_value()) {
          fully_visible = false;
          continue;
        }
        hebi::Point p = enemy->body[i].value();

        float age = (en_body_len > 1) ? 1.0f - static_cast<float>(i) / (en_body_len - 1) : 1.0f;
        set_val(en_body_age_ptr, p.x, p.y, age);

        if (i == 0) {
          set_val(en_head_ptr, p.x, p.y);
          en_head_p = p;
          has_head = true;
          enemy_heads[e] = p;
        }
        if (i == 1) {
          if (has_head) {
            en_dx = static_cast<float>(en_head_p.x - p.x);
            en_dy = static_cast<float>(en_head_p.y - p.y);
          }
        }
        if (i > 0 && i < en_body_len - 1) {
          set_val(en_body_ptr, p.x, p.y);
          enemy_body_visible[e] = true;
        }
        if (i == en_body_len - 1) {
          set_val(en_tail_ptr, p.x, p.y);
          enemy_tail_visible[e] = true;
        }
      }

      enemy_fully_visible[e] = fully_visible;

      std::fill_n(en_len_ptr, plane_size, static_cast<float>(enemy->length) / 100.0f);
      std::fill_n(en_dir_x_ptr, plane_size, en_dx);
      std::fill_n(en_dir_y_ptr, plane_size, en_dy);
      std::fill_n(en_alive_ptr, plane_size, 1.0f);
    }

    ch += 24;

    // Total Alive Count (1 Channel)
    float* total_alive_ptr = features_out + ch * plane_size;
    std::fill_n(total_alive_ptr, plane_size, alive_count / 4.0f);

    ch += 1;

    // Enemy Head Distances (15 Channels)
    for (int e = 0; e < 3; ++e) {
      for (int d = 1; d <= 5; ++d) {
        float* dist_ptr = features_out + ch * plane_size;
        std::fill_n(dist_ptr, plane_size, 0.0f);

        if (enemy_heads[e].has_value()) {
          hebi::Point head = enemy_heads[e].value();

          for (int y = 0; y < BOARD_SIZE; ++y) {
            for (int x = 0; x < BOARD_SIZE; ++x) {
              if (std::abs(x - head.x) + std::abs(y - head.y) <= d) {
                dist_ptr[y * BOARD_SIZE + x] = 1.0f;
              }
            }
          }
        }

        ch += 1;
      }
    }

    // Enemy Visible (3 Channels)
    for (int e = 0; e < 3; ++e) {
      float* visible_ptr = features_out + ch * plane_size;
      float is_visible = enemy_heads[e].has_value() ? 1.0f : 0.0f;
      std::fill_n(visible_ptr, plane_size, is_visible);
      ch += 1;
    }

    // Enemy Fully Visible (3 Channels)
    for (int e = 0; e < 3; ++e) {
      float* fully_visible_ptr = features_out + ch * plane_size;
      float is_fully_visible = enemy_fully_visible[e] ? 1.0f : 0.0f;
      std::fill_n(fully_visible_ptr, plane_size, is_fully_visible);
      ch += 1;
    }

    // Enemy Tail Visible (3 Channels)
    for (int e = 0; e < 3; ++e) {
      float* tail_visible_ptr = features_out + ch * plane_size;
      float is_tail_visible = enemy_tail_visible[e] ? 1.0f : 0.0f;
      std::fill_n(tail_visible_ptr, plane_size, is_tail_visible);
      ch += 1;
    }

    // Enemy Body Visible (3 Channels)
    for (int e = 0; e < 3; ++e) {
      float* body_visible_ptr = features_out + ch * plane_size;
      float is_body_visible = enemy_body_visible[e] ? 1.0f : 0.0f;
      std::fill_n(body_visible_ptr, plane_size, is_body_visible);
      ch += 1;
    }

    // Self Head Distances (5 Channels)
    for (int d = 1; d <= 5; ++d) {
      float* dist_ptr = features_out + ch * plane_size;
      std::fill_n(dist_ptr, plane_size, 0.0f);

      for (int y = 0; y < BOARD_SIZE; ++y) {
        for (int x = 0; x < BOARD_SIZE; ++x) {
          if (std::abs(x - self_head_p.x) + std::abs(y - self_head_p.y) <= d) {
            dist_ptr[y * BOARD_SIZE + x] = 1.0f;
          }
        }
      }
      ch += 1;
    }

    // Wall Distance (1 Channel)
    float* wall_dist_ptr = features_out + ch * plane_size;

    for (int y = 0; y < BOARD_SIZE; ++y) {
      for (int x = 0; x < BOARD_SIZE; ++x) {
        int dist_x = std::min(x, BOARD_SIZE - 1 - x);
        int dist_y = std::min(y, BOARD_SIZE - 1 - y);
        int wall_dist = std::min(dist_x, dist_y);
        wall_dist_ptr[y * BOARD_SIZE + x] = std::max(0.0f, 1.0f - static_cast<float>(wall_dist) / 7.0f);
      }
    }

    ch += 1;

    // Food and Clear Time (2 Channels)
    float* food_ptr = features_out + (ch + 0) * plane_size;
    float* clear_time_ptr = features_out + (ch + 1) * plane_size;

    for (int y = 0; y < BOARD_SIZE; ++y) {
      for (int x = 0; x < BOARD_SIZE; ++x) {
        int cell_index = y * BOARD_SIZE + x;
        int packed_data = memory_map[cell_index];

        bool has_food = (packed_data & (1 << 16)) != 0;
        int clear_time = packed_data & 0xFFFF;

        food_ptr[cell_index] = has_food ? 1.0f : 0.0f;
        clear_time_ptr[cell_index] = std::max(0.0f, 1.0f - static_cast<float>(clear_time) / 100.0f);
      }
    }

    ch += 2;

    // Self safe moves (1 Channel)
    float* self_safe_ptr = features_out + ch * plane_size;
    std::fill_n(self_safe_ptr, plane_size, 0.0f);

    if (safe_moves != nullptr) {
      int dirs[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
      for (int d = 0; d < 4; ++d) {
        if (safe_moves[d]) {
          int nx = self_head_p.x + dirs[d][0];
          int ny = self_head_p.y + dirs[d][1];
          if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE) {
            self_safe_ptr[ny * BOARD_SIZE + nx] = 1.0f;
          }
        }
      }
    }

    ch += 1;
  } catch (...) {
  }
}

}  // namespace hebi
