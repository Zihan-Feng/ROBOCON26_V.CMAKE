#pragma once

/* RTOS层及mcu main接口 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* bsp 层接口头文件 */
#include "bsp_dwt.h"
/* app 层接口头文件 一般是extern了任务函数才会在这里include */
#include "com_config.h"
#include "debug_task.h"

/* module层接口头文件 */

/* Definitions for TaskHand */
extern osThreadId_t CAN1_Send_TaskHandle;
extern osThreadId_t CAN2_Send_TaskHandle;
extern osThreadId_t CAN3_Send_TaskHandle;
extern osThreadId_t Debug_TaskHandle;
extern osThreadId_t Control_TaskHandle;

/* Definitions for TaskFunc */
void CAN1_Send_Task(void *argument);
void CAN2_Send_Task(void *argument);
void CAN3_Send_Task(void *argument);
void Debug_Task(void *argument);
void Control_Task(void *argument);

static inline void osTaskInit(void)
{
    const osThreadAttr_t CAN1_SendTaskHandle_attributes = {
    .name = "CAN1_Send_TaskHandle",
    .stack_size = 128*4 ,
    .priority = (osPriority_t) osPriorityNormal,
    };
    CAN1_Send_TaskHandle = osThreadNew(CAN1_Send_Task, NULL, &CAN1_SendTaskHandle_attributes);

    const osThreadAttr_t CAN2_SendTaskHandle_attributes = {
    .name = "CAN2_Send_TaskHandle",
    .stack_size = 128*4 ,
    .priority = (osPriority_t) osPriorityNormal,
    };
    CAN2_Send_TaskHandle = osThreadNew(CAN2_Send_Task, NULL, &CAN2_SendTaskHandle_attributes);
    
    const osThreadAttr_t CAN3_SendTaskHandle_attributes = {
    .name = "CAN3_Send_TaskHandle",
    .stack_size = 128*4 ,
    .priority = (osPriority_t) osPriorityNormal,
    };
    CAN3_Send_TaskHandle = osThreadNew(CAN3_Send_Task, NULL, &CAN3_SendTaskHandle_attributes);

    const osThreadAttr_t DebugTaskHandle_attributes = {
    .name = "Debug_TaskHandle",
    .stack_size = 128*4 ,
    .priority = (osPriority_t) osPriorityNormal,
    };
    Debug_TaskHandle = osThreadNew(Debug_Task, NULL, &DebugTaskHandle_attributes);

    const osThreadAttr_t ControlTaskHandle_attributes = {
    .name = "Control_TaskHandle",
    .stack_size = 128*4 ,
    .priority = (osPriority_t) osPriorityNormal,
    };
    Control_TaskHandle = osThreadNew(Control_Task, NULL, &ControlTaskHandle_attributes);
}
