/**
 * @file bsp_topics.h
 * @author Keten (2863861004@qq.com)
 * @brief Lightweight pub-sub module for inter-task communication.
 * @version 0.1
 * @date 2024-09-26
 */
#ifndef TOPICS_H
#define TOPICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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

#ifdef __cplusplus
}
#endif

#endif /* TOPICS_H */
