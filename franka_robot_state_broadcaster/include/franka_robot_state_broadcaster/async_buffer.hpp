// Copyright (c) 2026 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file contains code adapted from Intrinsic Innovation LLC (Copyright 2023),
// originally licensed under Apache 2.0:
// https://github.com/intrinsic-ai/sdk/blob/main/intrinsic/icon/utils/async_buffer.h
// Modifications: Removed absl dependency, replaced DCHECK with assert, C++17 compat.

#pragma once

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

/**
 * @brief A real-time safe producer/consumer triple buffer.
 *
 * Writes by one producer thread and reads by one consumer thread may be
 * concurrent -- anything else is not thread-safe.
 * Both reads and writes are lock-free and do not copy data.
 *
 * Terminology:
 *   Active  -- the buffer currently owned by the consumer.
 *   Free    -- the buffer available for the producer to write into.
 *   Mailbox -- a buffer not owned by either side; may be "full" (recently
 *              committed by the producer) or "empty" (recently fetched by
 *              the consumer).
 *
 * Producer usage:
 *   T& buffer = async_buffer.get_free_buffer();
 *   buffer = new_data;
 *   async_buffer.commit_free_buffer();
 *
 * Consumer usage:
 *   bool has_new_data = false;
 *   T& active = async_buffer.get_active_buffer(has_new_data);
 *
 * @tparam T The type of data stored in the buffer
 */
template <typename T>
class AsyncBuffer {
 public:
  /**
   * @brief Construct an AsyncBuffer with internally allocated storage
   *
   * @param args Arguments forwarded to the constructor of T for all three buffer slots
   */
  template <typename... InitArgs>
  explicit AsyncBuffer(const InitArgs&... args) {
    for (auto& buf : buffers_) {
      buf = std::make_unique<T>(args...);
    }
  }

  /**
   * @brief Get the most recently committed buffer for the consumer
   *
   * If the mailbox is full (producer committed since last call), swaps the
   * active and mailbox buffers and returns the new data. Otherwise returns
   * the same active buffer as before.
   *
   * @param[out] has_new_data Set to true if new data was available, false otherwise
   * @return Reference to the active buffer (always valid)
   */
  T& get_active_buffer(bool& has_new_data) {
    State current = state_.load(std::memory_order_acquire);
    State next = current;

    while (true) {
      next = current;
      if (next.mailbox_full) {
        std::swap(next.active_index, next.mailbox_index);
        next.mailbox_full = false;
      }
      if (state_.compare_exchange_weak(current, next, std::memory_order_release,
                                       std::memory_order_acquire)) {
        break;
      }
    }

    assert(current.free_index == next.free_index);
    assert(!next.mailbox_full);
    assert(next.is_consistent());

    has_new_data = current.mailbox_full;
    return *buffers_[next.active_index];
  }

  /**
   * @brief Get a free buffer that the producer can write into
   *
   * The returned reference is valid until the next call to commit_free_buffer().
   *
   * @return Reference to the free buffer
   */
  T& get_free_buffer() {
    free_buffer_checked_out_ = true;
    return *buffers_[state_.load(std::memory_order_acquire).free_index];
  }

  /**
   * @brief Commit the free buffer by swapping it with the mailbox
   *
   * Must be preceded by a call to get_free_buffer().
   *
   * @return true if the commit succeeded, false if get_free_buffer() was not called first
   */
  bool commit_free_buffer() {
    if (!free_buffer_checked_out_) {
      return false;
    }

    State current = state_.load(std::memory_order_acquire);
    State next;

    while (true) {
      next.active_index = current.active_index;
      next.mailbox_index = current.free_index;
      next.free_index = current.mailbox_index;
      next.mailbox_full = true;
      if (state_.compare_exchange_weak(current, next, std::memory_order_release,
                                       std::memory_order_acquire)) {
        break;
      }
    }

    assert(current.active_index == next.active_index);
    assert(next.mailbox_full);
    assert(next.is_consistent());
    free_buffer_checked_out_ = false;
    return true;
  }

 private:
  struct State {
    [[nodiscard]] bool is_consistent() const {
      return active_index < 3 && mailbox_index < 3 && free_index < 3 &&
             (active_index != mailbox_index) && (mailbox_index != free_index) &&
             (free_index != active_index);
    }
    uint8_t active_index = 0;   // NOLINT(misc-non-private-member-variables-in-classes)
    uint8_t mailbox_index = 1;  // NOLINT(misc-non-private-member-variables-in-classes)
    uint8_t free_index = 2;     // NOLINT(misc-non-private-member-variables-in-classes)
    bool mailbox_full = false;  // NOLINT(misc-non-private-member-variables-in-classes)
  };

  std::atomic<State> state_{State{0, 1, 2, false}};
  static_assert(std::atomic<State>::is_always_lock_free,
                "AsyncBuffer::State must be lock-free atomically swappable");
  // Only accessed by the producer thread -- not atomic by design (single-producer contract).
  bool free_buffer_checked_out_ = false;
  std::array<std::unique_ptr<T>, 3> buffers_;
};
