/**
 * @file lockfree_ringbuffer.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 无锁环形缓冲区
 * @version 0.1
 * @date 2026-04-19
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace Algorithm {

template <typename T, size_t Capacity>
class alignas(32) SpscLockFreeRingBuffer {
private:
  static_assert(Capacity >= 2, "Capacity must be >= 2");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be power of two");
  static_assert(std::is_copy_assignable<T>::value ||
                    std::is_move_assignable<T>::value,
                "Data must be copy-assignable or move-assignable");

  SpscLockFreeRingBuffer(const SpscLockFreeRingBuffer &) = delete;
  SpscLockFreeRingBuffer &operator=(const SpscLockFreeRingBuffer &) = delete;

  void Push(const T &item) {
    uint32_t tail = tail_.load(std::memory_order_relaxed);
    uint32_t next_tail = Inc(tail);
    uint32_t head = head_.load(std::memory_order_acquire);

    // 满时覆盖最旧元素
    if (next_tail == head) {
      head_.store(Inc(head), std::memory_order_release);
    }

    items_[tail] = item;
    tail_.store(next_tail, std::memory_order_release);
  }

  bool Pop(T *out) {
    if (out == nullptr)
      return false;
    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head == tail)
      return false;
    *out = items_[head];
    head_.store(Inc(head), std::memory_order_release);
    return true;
  }

  void Reset() {
    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
  }

private:
  uint32_t Inc(uint32_t index) const { return (index + 1U) % Capacity; }
  T items_[Capacity]{};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

} // namespace Algorithm