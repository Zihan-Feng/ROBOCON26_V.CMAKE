/**
 * @file topics.cpp
 * @author Keten (2863861004@qq.com)
 * @brief Lock-free pub-sub mechanism for inter-task communication.
 * @version 0.3
 * @date 2024-09-26
 */
#include "topics.hpp"

#include <atomic>
#include <cstdlib>
#include <cstring>

#include "lockfree_ringbuffer.hpp"

template <typename T> class SpscOverwriteRing {
public:
  SpscOverwriteRing() = default;

  ~SpscOverwriteRing() {
    if (items_ != nullptr) {
      std::free(items_);
      items_ = nullptr;
    }
  }

  bool Init(uint32_t capacity) {
    if (capacity == 0U) {
      capacity = 1U;
    }
    items_ = static_cast<T *>(std::calloc(capacity, sizeof(T)));
    if (items_ == nullptr) {
      capacity_ = 0U;
      return false;
    }

    capacity_ = capacity;
    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
    return true;
  }

  void Push(const T &item) {
    if (items_ == nullptr || capacity_ == 0U) {
      return;
    }

    uint32_t tail = tail_.load(std::memory_order_relaxed);
    uint32_t next_tail = Inc(tail);
    uint32_t head = head_.load(std::memory_order_acquire);

    // 满时覆盖最旧元素。
    if (next_tail == head) {
      head_.store(Inc(head), std::memory_order_release);
    }

    items_[tail] = item;
    tail_.store(next_tail, std::memory_order_release);
  }

  bool Pop(T *out) {
    if (out == nullptr || items_ == nullptr || capacity_ == 0U) {
      return false;
    }

    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }

    *out = items_[head];
    head_.store(Inc(head), std::memory_order_release);
    return true;
  }

private:
  uint32_t Inc(uint32_t index) const { return (index + 1U) % capacity_; }

  T *items_{nullptr};
  uint32_t capacity_{0};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

struct topic_queue_t {
  SpscOverwriteRing<publish_data> ring;
};

typedef struct subscriber_node_t {
  Subscriber *sub;
  topic_queue_t queue;
  struct subscriber_node_t *next;
} subscriber_node_t;

struct internal_topic {
  Publisher *pub;
  subscriber_node_t *subs;
  const char *topic_str;
  struct internal_topic *next;
};

class TopicBus {
public:
  static TopicBus &Instance() {
    static TopicBus instance;
    return instance;
  }

  void Init() { topics_ = nullptr; }

  struct internal_topic *RegisterTopic(const char *topic) {
    struct internal_topic *now = nullptr;

    if (topic == nullptr) {
      return nullptr;
    }

    for (now = topics_; now != nullptr; now = now->next) {
      if (now->topic_str != nullptr &&
          std::strcmp(now->topic_str, topic) == 0) {
        return now;
      }
    }

    now = static_cast<struct internal_topic *>(
        std::calloc(1, sizeof(struct internal_topic)));
    if (now == nullptr) {
      return nullptr;
    }

    now->topic_str = topic;
    now->next = topics_;
    topics_ = now;
    return now;
  }

private:
  TopicBus() = default;
  struct internal_topic *topics_{nullptr};
};

static void pub_commit(Publisher *pub, publish_data data);
static publish_data sub_get(Subscriber *sub);
static void queue_init(topic_queue_t *queue, uint32_t capacity);
static void queue_push(topic_queue_t *queue, publish_data data);
static int queue_pop(topic_queue_t *queue, publish_data *out);
static void queue_free(topic_queue_t *queue);

void SubPub_Init(void) { TopicBus::Instance().Init(); }

static void queue_init(topic_queue_t *queue, uint32_t capacity) {
  if (queue == nullptr) {
    return;
  }
  (void)queue->ring.Init(capacity);
}

static void queue_push(topic_queue_t *queue, publish_data data) {
  if (queue == nullptr) {
    return;
  }
  queue->ring.Push(data);
}

static int queue_pop(topic_queue_t *queue, publish_data *out) {
  if (queue == nullptr || out == nullptr) {
    return 0;
  }
  return queue->ring.Pop(out) ? 1 : 0;
}

static void queue_free(topic_queue_t *queue) { (void)queue; }

static void pub_commit(Publisher *pub, publish_data data) {
  subscriber_node_t *node;

  if (pub == nullptr || pub->topic == nullptr) {
    return;
  }

  node = pub->topic->subs;
  while (node != nullptr) {
    queue_push(&node->queue, data);
    node = node->next;
  }
}

static publish_data sub_get(Subscriber *sub) {
  publish_data now;

  now.data = nullptr;
  now.len = -1;

  if (sub == nullptr || sub->queue == nullptr) {
    return now;
  }

  if (queue_pop(static_cast<topic_queue_t *>(sub->queue), &now) == 0) {
    now.data = nullptr;
    now.len = -1;
  }

  return now;
}

Publisher *register_pub(const char *topic) {
  struct internal_topic *now_topic;
  Publisher *obj;

  now_topic = TopicBus::Instance().RegisterTopic(topic);
  if (now_topic == nullptr) {
    return nullptr;
  }

  if (now_topic->pub != nullptr) {
    return now_topic->pub;
  }

  obj = static_cast<Publisher *>(calloc(1, sizeof(Publisher)));
  if (obj == nullptr) {
    return nullptr;
  }

  obj->pub_topic = topic;
  obj->topic = now_topic;
  obj->publish = pub_commit;
  now_topic->pub = obj;
  return obj;
}

Subscriber *register_sub(const char *topic, uint32_t buffer_len) {
  struct internal_topic *now_topic;
  subscriber_node_t *node;
  Subscriber *obj;

  now_topic = TopicBus::Instance().RegisterTopic(topic);
  if (now_topic == nullptr) {
    return nullptr;
  }

  obj = static_cast<Subscriber *>(calloc(1, sizeof(Subscriber)));
  if (obj == nullptr) {
    return nullptr;
  }

  node = static_cast<subscriber_node_t *>(calloc(1, sizeof(subscriber_node_t)));
  if (node == nullptr) {
    free(obj);
    return nullptr;
  }

  queue_init(&node->queue, buffer_len);

  obj->sub_topic = topic;
  obj->topic = now_topic;
  obj->queue = &node->queue;
  obj->get_data = sub_get;

  node->sub = obj;
  node->next = now_topic->subs;
  now_topic->subs = node;

  return obj;
}
