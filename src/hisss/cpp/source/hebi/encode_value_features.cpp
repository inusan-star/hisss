#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

void StateProcessor::encode_value_features(int32_t* memory_map, float* features_out) {
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

    // Enemies (27 Channels)
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
    bool is_longer_than_enemy[3] = {false, false, false};
    std::optional<hebi::Point> enemy_heads[3] = {std::nullopt, std::nullopt, std::nullopt};

    for (int e = 0; e < 3; ++e) {
      int base_ch = ch + (e * 9);

      float* en_head_ptr = features_out + (base_ch + 0) * plane_size;
      float* en_body_ptr = features_out + (base_ch + 1) * plane_size;
      float* en_tail_ptr = features_out + (base_ch + 2) * plane_size;
      float* en_len_ptr = features_out + (base_ch + 3) * plane_size;
      float* en_hp_ptr = features_out + (base_ch + 4) * plane_size;
      float* en_dir_x_ptr = features_out + (base_ch + 5) * plane_size;
      float* en_dir_y_ptr = features_out + (base_ch + 6) * plane_size;
      float* en_body_age_ptr = features_out + (base_ch + 7) * plane_size;
      float* en_alive_ptr = features_out + (base_ch + 8) * plane_size;

      const hebi::Snake* enemy = enemy_slots[e];

      if (enemy == nullptr || enemy->elimination_event.has_value()) {
        std::fill_n(en_head_ptr, plane_size, 0.0f);
        std::fill_n(en_body_ptr, plane_size, 0.0f);
        std::fill_n(en_tail_ptr, plane_size, 0.0f);
        std::fill_n(en_len_ptr, plane_size, 0.0f);
        std::fill_n(en_hp_ptr, plane_size, 0.0f);
        std::fill_n(en_dir_x_ptr, plane_size, 0.0f);
        std::fill_n(en_dir_y_ptr, plane_size, 0.0f);
        std::fill_n(en_body_age_ptr, plane_size, 0.0f);
        std::fill_n(en_alive_ptr, plane_size, 0.0f);
        continue;
      }

      alive_count += 1.0f;

      is_longer_than_enemy[e] = (game_state_.you.length > enemy->length);

      int en_body_len = enemy->body.size();
      float en_dx = 0.0f;
      float en_dy = 0.0f;
      hebi::Point en_head_p;

      for (int i = 0; i < en_body_len; ++i) {
        if (!enemy->body[i].has_value()) continue;
        hebi::Point p = enemy->body[i].value();

        float age = (en_body_len > 1) ? 1.0f - static_cast<float>(i) / (en_body_len - 1) : 1.0f;
        set_val(en_body_age_ptr, p.x, p.y, age);

        if (i == 0) {
          set_val(en_head_ptr, p.x, p.y);
          en_head_p = p;
          enemy_heads[e] = p;
        }
        if (i == 1) {
          en_dx = static_cast<float>(en_head_p.x - p.x);
          en_dy = static_cast<float>(en_head_p.y - p.y);
        }
        if (i > 0 && i < en_body_len - 1) {
          set_val(en_body_ptr, p.x, p.y);
        }
        if (i == en_body_len - 1) {
          set_val(en_tail_ptr, p.x, p.y);
        }
      }

      std::fill_n(en_len_ptr, plane_size, static_cast<float>(enemy->length) / 100.0f);
      std::fill_n(en_hp_ptr, plane_size, static_cast<float>(enemy->health.value_or(0)) / 100.0f);
      std::fill_n(en_dir_x_ptr, plane_size, en_dx);
      std::fill_n(en_dir_y_ptr, plane_size, en_dy);
      std::fill_n(en_alive_ptr, plane_size, 1.0f);
    }

    ch += 27;

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

    // Head Advantage (4 Channels)
    bool is_longest = true;

    for (int e = 0; e < 3; ++e) {
      const hebi::Snake* enemy = enemy_slots[e];

      if (enemy != nullptr && !enemy->elimination_event.has_value() && game_state_.you.length <= enemy->length) {
        is_longest = false;
      }
    }

    float* head_adv_ptr = features_out + ch * plane_size;
    std::fill_n(head_adv_ptr, plane_size, is_longest ? 1.0f : 0.0f);
    ch += 1;

    for (int e = 0; e < 3; ++e) {
      float* ind_head_adv_ptr = features_out + ch * plane_size;
      std::fill_n(ind_head_adv_ptr, plane_size, is_longer_than_enemy[e] ? 1.0f : 0.0f);
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

    // ==================================================
    // BFS features (22 Channels)
    // ==================================================

    std::vector<int> arrival_0(plane_size, 999);
    std::vector<int> arrival_1(plane_size, 999);
    std::vector<int> arrival_2(plane_size, 999);
    std::vector<int> arrival_3(plane_size, 999);
    std::vector<int>* arrivals[4] = {&arrival_0, &arrival_1, &arrival_2, &arrival_3};

    std::vector<bool> obs(plane_size, false);
    for (const auto& snake : game_state_.board.snakes) {
      if (snake.elimination_event.has_value()) continue;
      int len = snake.body.size();

      for (int i = 0; i < len - 1; ++i) {
        if (snake.body[i].has_value()) {
          hebi::Point p = snake.body[i].value();

          if (p.x >= 0 && p.x < BOARD_SIZE && p.y >= 0 && p.y < BOARD_SIZE) {
            obs[p.y * BOARD_SIZE + p.x] = true;
          }
        }
      }
    }

    std::vector<int> q(plane_size);
    int dx[4] = {0, 0, -1, 1};
    int dy[4] = {-1, 1, 0, 0};

    auto do_bfs = [&](int idx, hebi::Point start_p) {
      if (start_p.x < 0 || start_p.x >= BOARD_SIZE || start_p.y < 0 || start_p.y >= BOARD_SIZE) return;
      int head = 0, tail = 0;
      int start_idx = start_p.y * BOARD_SIZE + start_p.x;

      q[tail++] = start_idx;
      (*arrivals[idx])[start_idx] = 0;

      while (head < tail) {
        int curr = q[head++];
        int cx = curr % BOARD_SIZE;
        int cy = curr / BOARD_SIZE;
        int c_dist = (*arrivals[idx])[curr];

        for (int d = 0; d < 4; ++d) {
          int nx = cx + dx[d];
          int ny = cy + dy[d];

          if (nx >= 0 && nx < BOARD_SIZE && ny >= 0 && ny < BOARD_SIZE) {
            int n_idx = ny * BOARD_SIZE + nx;

            if (!obs[n_idx] && (*arrivals[idx])[n_idx] == 999) {
              (*arrivals[idx])[n_idx] = c_dist + 1;
              q[tail++] = n_idx;
            }
          }
        }
      }
    };

    do_bfs(0, self_head_p);
    float lengths[4] = {static_cast<float>(game_state_.you.length), 1.0f, 1.0f, 1.0f};

    for (int e = 0; e < 3; ++e) {
      const hebi::Snake* enemy = enemy_slots[e];

      if (enemy != nullptr && enemy_heads[e].has_value()) {
        do_bfs(e + 1, enemy_heads[e].value());
        lengths[e + 1] = static_cast<float>(enemy->length);
      }
    }

    // Arrival Time (4 Channels), First Reach (4 Channels), Arrival Margin (3 Channels)
    float* arr_ptr[4] = {features_out + (ch + 0) * plane_size, features_out + (ch + 1) * plane_size, features_out + (ch + 2) * plane_size,
                         features_out + (ch + 3) * plane_size};
    float* fr_ptr[4] = {features_out + (ch + 4) * plane_size, features_out + (ch + 5) * plane_size, features_out + (ch + 6) * plane_size,
                        features_out + (ch + 7) * plane_size};
    float* am_ptr[3] = {features_out + (ch + 8) * plane_size, features_out + (ch + 9) * plane_size, features_out + (ch + 10) * plane_size};

    int voronoi_area[4] = {0, 0, 0, 0};

    for (int i = 0; i < plane_size; ++i) {
      int a0 = arrival_0[i];
      int a1 = arrival_1[i];
      int a2 = arrival_2[i];
      int a3 = arrival_3[i];

      // Arrival Time (4 Channels)
      arr_ptr[0][i] = std::max(0.0f, 1.0f - a0 / 50.0f);
      arr_ptr[1][i] = std::max(0.0f, 1.0f - a1 / 50.0f);
      arr_ptr[2][i] = std::max(0.0f, 1.0f - a2 / 50.0f);
      arr_ptr[3][i] = std::max(0.0f, 1.0f - a3 / 50.0f);

      // First Reach (4 Channels)
      int min_arr = std::min({a0, a1, a2, a3});
      fr_ptr[0][i] = 0.0f;
      fr_ptr[1][i] = 0.0f;
      fr_ptr[2][i] = 0.0f;
      fr_ptr[3][i] = 0.0f;

      if (min_arr < 999) {
        if (a0 == min_arr) {
          fr_ptr[0][i] = 1.0f;
          voronoi_area[0]++;
        }
        if (a1 == min_arr) {
          fr_ptr[1][i] = 1.0f;
          voronoi_area[1]++;
        }
        if (a2 == min_arr) {
          fr_ptr[2][i] = 1.0f;
          voronoi_area[2]++;
        }
        if (a3 == min_arr) {
          fr_ptr[3][i] = 1.0f;
          voronoi_area[3]++;
        }
      }

      // Arrival Margin (3 Channels)
      am_ptr[0][i] = std::clamp((a1 - a0) / 50.0f, -1.0f, 1.0f);
      am_ptr[1][i] = std::clamp((a2 - a0) / 50.0f, -1.0f, 1.0f);
      am_ptr[2][i] = std::clamp((a3 - a0) / 50.0f, -1.0f, 1.0f);
    }

    ch += 11;

    float v_self_per_len = (voronoi_area[0] > 0) ? std::min(1.0f, lengths[0] / static_cast<float>(voronoi_area[0])) : 1.0f;
    float v_e0_per_len = (voronoi_area[1] > 0) ? std::min(1.0f, lengths[1] / static_cast<float>(voronoi_area[1])) : 1.0f;
    float v_e1_per_len = (voronoi_area[2] > 0) ? std::min(1.0f, lengths[2] / static_cast<float>(voronoi_area[2])) : 1.0f;
    float v_e2_per_len = (voronoi_area[3] > 0) ? std::min(1.0f, lengths[3] / static_cast<float>(voronoi_area[3])) : 1.0f;

    float v_margin_e0 = static_cast<float>(voronoi_area[0] - voronoi_area[1]) / 255.0f;
    float v_margin_e1 = static_cast<float>(voronoi_area[0] - voronoi_area[2]) / 255.0f;
    float v_margin_e2 = static_cast<float>(voronoi_area[0] - voronoi_area[3]) / 255.0f;

    float v_self = static_cast<float>(voronoi_area[0]) / 255.0f;
    float v_e0 = static_cast<float>(voronoi_area[1]) / 255.0f;
    float v_e1 = static_cast<float>(voronoi_area[2]) / 255.0f;
    float v_e2 = static_cast<float>(voronoi_area[3]) / 255.0f;

    // Voronoi per Length (4 Channels)
    std::fill_n(features_out + (ch + 0) * plane_size, plane_size, v_self_per_len);
    std::fill_n(features_out + (ch + 1) * plane_size, plane_size, v_e0_per_len);
    std::fill_n(features_out + (ch + 2) * plane_size, plane_size, v_e1_per_len);
    std::fill_n(features_out + (ch + 3) * plane_size, plane_size, v_e2_per_len);

    // Margin Voronoi (3 Channels)
    std::fill_n(features_out + (ch + 4) * plane_size, plane_size, v_margin_e0);
    std::fill_n(features_out + (ch + 5) * plane_size, plane_size, v_margin_e1);
    std::fill_n(features_out + (ch + 6) * plane_size, plane_size, v_margin_e2);

    // Voronoi (4 Channels)
    std::fill_n(features_out + (ch + 7) * plane_size, plane_size, v_self);
    std::fill_n(features_out + (ch + 8) * plane_size, plane_size, v_e0);
    std::fill_n(features_out + (ch + 9) * plane_size, plane_size, v_e1);
    std::fill_n(features_out + (ch + 10) * plane_size, plane_size, v_e2);

    ch += 11;

    // Food (1 Channel)
    float* food_ptr = features_out + ch * plane_size;
    std::fill_n(food_ptr, plane_size, 0.0f);

    for (const auto& f : game_state_.board.food) {
      set_val(food_ptr, f.x, f.y, 1.0f);
    }

    ch += 1;

    // Food Remain Turn (1 Channel)
    float* food_remain_ptr = features_out + ch * plane_size;
    std::fill_n(food_remain_ptr, plane_size, 0.0f);

    for (const auto& f : game_state_.board.food) {
      float elapsed_turns = static_cast<float>(game_state_.turn - f.spawn_turn);
      float normalized_val = std::min(1.0f, elapsed_turns / 100.0f);
      set_val(food_remain_ptr, f.x, f.y, normalized_val);
    }
  } catch (...) {
  }
}

}  // namespace hebi
