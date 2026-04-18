#include "robot.h"



/* 更多特色功能可以在这里通过宏开关来决定是否使用 */

void Robot_Init()
{
    /* 关闭中断，防止在初始化过程中发生中断 */
    __disable_irq();

    DWT_Init(SystemCoreClock / 1000000U);
    /* 实际应用的can总线初始化 */
    CAN_FILTER_Init(&hfdcan1, FDCAN_STANDARD_ID, 1,FDCAN_FILTER_TO_RXFIFO0,0,0);
    CAN_FILTER_Init(&hfdcan1, FDCAN_EXTENDED_ID, 2,FDCAN_FILTER_TO_RXFIFO1,0,0);
    BSP_FDCAN_Init(&hfdcan1,CAN1_Rx_Callback);
    CAN_FILTER_Init(&hfdcan2, FDCAN_STANDARD_ID, 3,FDCAN_FILTER_TO_RXFIFO0,0,0);
    CAN_FILTER_Init(&hfdcan2, FDCAN_EXTENDED_ID, 4,FDCAN_FILTER_TO_RXFIFO1,0,0);
    BSP_FDCAN_Init(&hfdcan2,CAN2_Rx_Callback);
    CAN_FILTER_Init(&hfdcan3, FDCAN_STANDARD_ID, 5,FDCAN_FILTER_TO_RXFIFO0,0,0);
    CAN_FILTER_Init(&hfdcan3, FDCAN_EXTENDED_ID, 6,FDCAN_FILTER_TO_RXFIFO1,0,0);
    BSP_FDCAN_Init(&hfdcan3,CAN3_Rx_Callback);  
    if (Common_Service_Init() != 0)
    {
        Error_Handler();
    }
    /* freertos任务调度初始化 */
    osTaskInit();

    /*初始化完成，开启中断 */
    __enable_irq();
}


