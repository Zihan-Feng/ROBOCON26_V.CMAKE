#include "control_task.h"
#include "data_type.h"
#include "topics.h"
#include <cstdint>

Uart_Instance_t *xbox_uart_instance = NULL;
extern uart_package_t xbox_uart_package;
osThreadId_t Control_TaskHandle;

/* 控制信息发布 */
Publisher *ctrl_pub;
pub_Control_Data ctrl_data;

/* 订阅xbox遥控控制信息 */
Subscriber *xbox_data_sub;
pub_Xbox_Data xbox_chassis_data;

void Xbox_Data_Process()
{
  ctrl_data.linear_x = (int)(xbox_chassis_data.joyLHori - 32767) / 32767.0f * MAX_VELOCITY;
  ctrl_data.linear_y = -(int)(xbox_chassis_data.joyLVert - 32767) / 32767.0f * MAX_VELOCITY;
  ctrl_data.Omega = (int)(xbox_chassis_data.joyRHori - 32767) / 32767.0f * MAX_VELOCITY;
  ctrl_data.Status = 1;
  ctrl_data.Move = 0;
  ctrl_data.ctrl = 0;
}

void Control_Task(void *argument)
{
  /* 创建发布者 */
  ctrl_pub = register_pub("ctrl_topic");
  publish_data ctrl_publish_data;
  /* 创建订阅者 */
  publish_data xbox_data;
  xbox_data_sub = register_sub("xbox", 1);
  portTickType currentTime;
  currentTime = xTaskGetTickCount();
    for (;;)
    {
        /* 从xbox数据订阅者中获取数据 */
        xbox_data = xbox_data_sub->get_data(xbox_data_sub);
        if (xbox_data.len != -1)
        {
            /* 将void*数据转换为pub_Xbox_Data类型 */
            xbox_chassis_data = *(pub_Xbox_Data *)xbox_data.data;
            /* 处理xbox数据，生成控制数据 */
            Xbox_Data_Process();
            /* 发布控制数据 */
            ctrl_publish_data.data = (uint8_t *)&ctrl_data;
            ctrl_publish_data.len = sizeof(pub_Control_Data);
            ctrl_pub->publish(ctrl_pub,ctrl_publish_data);
        }
        vTaskDelayUntil(&currentTime, 5);
        // vTaskDelay(5);
    }
}