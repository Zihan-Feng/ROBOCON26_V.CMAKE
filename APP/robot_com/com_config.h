#pragma once

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

/*------------------------------------extern------------------------------------*/

extern QueueHandle_t CAN1_TxPort;
extern QueueHandle_t CAN2_TxPort;
extern QueueHandle_t CAN3_TxPort;

/*-----------------------------------macro------------------------------------*/
#define CAN1_TX_QUEUE_SIZE      8
#define CAN2_TX_QUEUE_SIZE      8
#define CAN3_TX_QUEUE_SIZE      8

/*----------------------------------function----------------------------------*/

/**
 * @brief 
 * 
 * @return uint8_t 
 */
uint8_t Common_Service_Init();


/**
 * @brief 
 * 
 * @param can_instance 
 */
void CAN1_Rx_Callback(FDCAN_RxFrame_TypeDef *can_instance);


/**
 * @brief 
 * 
 * @param can_instance 
 */
void CAN2_Rx_Callback(FDCAN_RxFrame_TypeDef *can_instance);

/**
 * @brief 
 * 
 * @param can_instance 
 */
void CAN3_Rx_Callback(FDCAN_RxFrame_TypeDef *can_instance);

/**
 * @brief 
 * 
 * @param argument 
 */
void CAN1_Send_Task(void *argument);
void CAN2_Send_Task(void *argument);
void CAN3_Send_Task(void *argument);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#endif