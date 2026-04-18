/**
 * @file bsp_topics.c
 * @author Keten (2863861004@qq.com)
 * @brief Lightweight pub-sub mechanism for inter-task communication.
 * @version 0.1
 * @date 2024-09-26
 */
#include "topics.h"

#include <stdlib.h>
#include <string.h>

typedef struct topic_queue_t {
    publish_data *items;
    uint32_t capacity;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} topic_queue_t;

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

static struct internal_topic *g_topics = NULL;

static struct internal_topic *register_topic(const char *topic);
static void pub_commit(Publisher *pub, publish_data data);
static publish_data sub_get(Subscriber *sub);
static void queue_init(topic_queue_t *queue, uint32_t capacity);
static void queue_push(topic_queue_t *queue, publish_data data);
static int queue_pop(topic_queue_t *queue, publish_data *out);
static void queue_free(topic_queue_t *queue);

void SubPub_Init(void)
{
    g_topics = NULL;
}

static void queue_init(topic_queue_t *queue, uint32_t capacity)
{
    if (capacity == 0U)
    {
        capacity = 1U;
    }

    queue->items = (publish_data *)calloc(capacity, sizeof(publish_data));
    queue->capacity = (queue->items != NULL) ? capacity : 0U;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
}

static void queue_push(topic_queue_t *queue, publish_data data)
{
    if (queue == NULL || queue->items == NULL || queue->capacity == 0U)
    {
        return;
    }

    if (queue->count == queue->capacity)
    {
        queue->head = (queue->head + 1U) % queue->capacity;
        queue->count--;
    }

    queue->items[queue->tail] = data;
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count++;
}

static int queue_pop(topic_queue_t *queue, publish_data *out)
{
    if (queue == NULL || out == NULL || queue->items == NULL || queue->count == 0U)
    {
        return 0;
    }

    *out = queue->items[queue->head];
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    return 1;
}

static void queue_free(topic_queue_t *queue)
{
    if (queue == NULL)
    {
        return;
    }

    free(queue->items);
    queue->items = NULL;
    queue->capacity = 0U;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
}

static struct internal_topic *register_topic(const char *topic)
{
    struct internal_topic *now;

    if (topic == NULL)
    {
        return NULL;
    }

    for (now = g_topics; now != NULL; now = now->next)
    {
        if ((now->topic_str != NULL) && (strcmp(now->topic_str, topic) == 0))
        {
            return now;
        }
    }

    now = (struct internal_topic *)calloc(1, sizeof(struct internal_topic));
    if (now == NULL)
    {
        return NULL;
    }

    now->topic_str = topic;
    now->next = g_topics;
    g_topics = now;
    return now;
}

static void pub_commit(Publisher *pub, publish_data data)
{
    subscriber_node_t *node;

    if (pub == NULL || pub->topic == NULL)
    {
        return;
    }

    node = pub->topic->subs;
    while (node != NULL)
    {
        queue_push(&node->queue, data);
        node = node->next;
    }
}

static publish_data sub_get(Subscriber *sub)
{
    publish_data now;

    now.data = NULL;
    now.len = -1;

    if (sub == NULL || sub->queue == NULL)
    {
        return now;
    }

    if (queue_pop((topic_queue_t *)sub->queue, &now) == 0)
    {
        now.data = NULL;
        now.len = -1;
    }

    return now;
}

Publisher *register_pub(const char *topic)
{
    struct internal_topic *now_topic;
    Publisher *obj;

    now_topic = register_topic(topic);
    if (now_topic == NULL)
    {
        return NULL;
    }

    if (now_topic->pub != NULL)
    {
        return now_topic->pub;
    }

    obj = (Publisher *)calloc(1, sizeof(Publisher));
    if (obj == NULL)
    {
        return NULL;
    }

    obj->pub_topic = topic;
    obj->topic = now_topic;
    obj->publish = pub_commit;
    now_topic->pub = obj;
    return obj;
}

Subscriber *register_sub(const char *topic, uint32_t buffer_len)
{
    struct internal_topic *now_topic;
    subscriber_node_t *node;
    Subscriber *obj;

    now_topic = register_topic(topic);
    if (now_topic == NULL)
    {
        return NULL;
    }

    obj = (Subscriber *)calloc(1, sizeof(Subscriber));
    if (obj == NULL)
    {
        return NULL;
    }

    node = (subscriber_node_t *)calloc(1, sizeof(subscriber_node_t));
    if (node == NULL)
    {
        free(obj);
        return NULL;
    }

    queue_init(&node->queue, buffer_len);
    if (node->queue.items == NULL)
    {
        free(node);
        free(obj);
        return NULL;
    }

    obj->sub_topic = topic;
    obj->topic = now_topic;
    obj->queue = &node->queue;
    obj->get_data = sub_get;

    node->sub = obj;
    node->next = now_topic->subs;
    now_topic->subs = node;

    return obj;
}
