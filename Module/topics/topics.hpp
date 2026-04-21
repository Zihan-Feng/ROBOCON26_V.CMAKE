#pragma once

#include <cstdint>
#include <cstring>

extern "C" {

typedef struct publish_data_t {
  uint8_t *data;
  int len;
} publish_data;

struct internal_topic;

typedef struct publisher_t Publisher;
typedef struct subscriber_t Subscriber;

typedef void (*publish_fn_t)(Publisher *pub, publish_data data);
typedef publish_data (*get_data_fn_t)(Subscriber *sub);

typedef struct publisher_t {
  const char *pub_topic;
  struct internal_topic *topic;
  publish_fn_t publish;
} Publisher;

typedef struct subscriber_t {
  const char *sub_topic;
  struct internal_topic *topic;
  void *queue;
  get_data_fn_t get_data;
} Subscriber;

void SubPub_Init(void);
Publisher *register_pub(const char *topic);
Subscriber *register_sub(const char *topic, uint32_t buffer_len);

} // extern "C"

class TopicPublisher {
public:
  explicit TopicPublisher(const char *topic) : pub_(register_pub(topic)) {}

  bool IsValid() const { return pub_ != nullptr; }

  bool Publish(uint8_t *data, int len) const {
    if (pub_ == nullptr || data == nullptr || len < 0) {
      return false;
    }
    publish_data packet{data, len};
    pub_->publish(pub_, packet);
    return true;
  }

private:
  Publisher *pub_{nullptr};
};

class TopicSubscriber {
public:
  TopicSubscriber(const char *topic, uint32_t buffer_len)
      : sub_(register_sub(topic, buffer_len)) {}

  bool IsValid() const { return sub_ != nullptr; }

  bool TryGet(publish_data *out) const {
    if (sub_ == nullptr || out == nullptr) {
      return false;
    }

    publish_data packet = sub_->get_data(sub_);
    if (packet.data == nullptr || packet.len < 0) {
      return false;
    }

    *out = packet;
    return true;
  }

private:
  Subscriber *sub_{nullptr};
};

template <typename T> class TypedTopicPublisher {
public:
  explicit TypedTopicPublisher(const char *topic) : pub_(topic) {}

  bool IsValid() const { return pub_.IsValid(); }

  bool Publish(const T &value) const {
    return pub_.Publish(
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(&value)),
        static_cast<int>(sizeof(T)));
  }

private:
  TopicPublisher pub_;
};

template <typename T> class TypedTopicSubscriber {
public:
  TypedTopicSubscriber(const char *topic, uint32_t buffer_len)
      : sub_(topic, buffer_len) {}

  bool IsValid() const { return sub_.IsValid(); }

  bool TryGet(T *out) const {
    if (out == nullptr) {
      return false;
    }

    publish_data packet{};
    if (!sub_.TryGet(&packet)) {
      return false;
    }

    if (packet.len != static_cast<int>(sizeof(T))) {
      return false;
    }

    std::memcpy(out, packet.data, sizeof(T));
    return true;
  }

private:
  TopicSubscriber sub_;
};
