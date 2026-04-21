/**
 * @file lockfree_queue.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 轻量级有界 MPSC 无锁队列
 * @version 0.1
 * @date 2026-04-18
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

enum class QueueError : uint8_t { OK = 0, FULL, EMPTY };

/**
 * @brief 固定容量 MPSC 无锁队列（多生产者，单消费者）
 *
 * 设计目标：
 * 1) 无动态内存分配（适合 MCU）
 * 2) 无锁，生产者/消费者互不阻塞
 * 3) Try 接口：不会无限等待，便于实时任务使用
 *
 * @tparam Data 元素类型（需要可拷贝或可移动赋值）
 * @tparam Capacity 队列容量，必须是 2 的幂
 */
template <typename Data, size_t Capacity> class alignas(32) MpscQueue {
  static_assert(Capacity >= 2, "Capacity must be >= 2");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "Capacity must be power of two");
  static_assert(std::is_copy_assignable<Data>::value ||
                    std::is_move_assignable<Data>::value,
                "Data must be copy-assignable or move-assignable");

private:
  // 环形槽位数组
  struct alignas(32) Slot {
    std::atomic<size_t> sequence;
    Data data;
  };

public:
  MpscQueue() noexcept : enqueue_pos_(0), dequeue_pos_(0) {
    for (size_t i = 0; i < Capacity; ++i) {
      slots_[i].sequence.store(i, std::memory_order_relaxed);
    }
  }

  MpscQueue(const MpscQueue &) = delete;
  MpscQueue &operator=(const MpscQueue &) = delete;

  template <typename T> QueueError TryPush(T &&item) noexcept {
    size_t pos = enqueue_pos_.load(std::memory_order_relaxed);

    for (;;) {
      Slot &slot = slots_[pos & kMask];
      const size_t seq = slot.sequence.load(std::memory_order_acquire);
      const intptr_t diff =
          static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);

      if (diff == 0) {
        if (enqueue_pos_.compare_exchange_weak(pos, pos + 1,
                                               std::memory_order_relaxed,
                                               std::memory_order_relaxed)) {
          slot.data = std::forward<T>(item);
          slot.sequence.store(pos + 1, std::memory_order_release);
          return QueueError::OK;
        }
      } else if (diff < 0) {
        return QueueError::FULL;
      } else {
        pos = enqueue_pos_.load(std::memory_order_relaxed);
      }
    }
  }

  QueueError TryPop(Data &out) noexcept {
    const size_t pos = dequeue_pos_.load(std::memory_order_relaxed);
    Slot &slot = slots_[pos & kMask];
    const size_t seq = slot.sequence.load(std::memory_order_acquire);
    const intptr_t diff =
        static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);

    if (diff == 0) {
      out = std::move(slot.data);
      slot.sequence.store(pos + Capacity, std::memory_order_release);
      dequeue_pos_.store(pos + 1, std::memory_order_relaxed);
      return QueueError::OK;
    }

    if (diff < 0) {
      return QueueError::EMPTY;
    }

    return QueueError::EMPTY;
  }

  size_t Size() const noexcept {
    const size_t enq = enqueue_pos_.load(std::memory_order_acquire);
    const size_t deq = dequeue_pos_.load(std::memory_order_acquire);
    return enq - deq;
  }

  static constexpr size_t GetCapacity() noexcept { return Capacity; }

private:
  static constexpr size_t kMask = Capacity - 1;

  alignas(32) Slot slots_[Capacity];
  alignas(32) std::atomic<size_t> enqueue_pos_;
  alignas(32) std::atomic<size_t> dequeue_pos_;
};

} // namespace Algorithm
