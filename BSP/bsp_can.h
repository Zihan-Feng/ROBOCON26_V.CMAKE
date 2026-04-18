#ifndef BSP_CAN_H
#define BSP_CAN_H

/* c语言库接口头文件 */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/* hal层接口头文件 */
#include "fdcan.h"

/*-----------------------------------macro------------------------------------*/
#define DEVICE_CAN_CNT  6 // 一条can总线上至多挂载6个设备


typedef struct {
	FDCAN_HandleTypeDef *hcan_handle;// hal库can句柄成员
    FDCAN_TxHeaderTypeDef Header;    
    uint32_t isExTid;                // 要发送的是否为扩展帧
    uint32_t tx_mailbox;             // 发送邮箱号
    uint32_t tx_id;                  // 发送id
    uint8_t tx_len;                  // 发送数据帧长度，长度范围为0~8
    uint8_t can_tx_buff[8];          // can发送buffer
}FDCAN_TxFrame_TypeDef;

typedef struct {
	FDCAN_HandleTypeDef *hcan_handle;// hal库can句柄成员
    FDCAN_RxHeaderTypeDef Header;
    uint8_t rx_len;                 // 接收数据帧长度，长度范围为0~8
    uint32_t rx_id;                 // 接收id   
    uint8_t can_rx_buff[8];         // can接收buffer
} FDCAN_RxFrame_TypeDef;



/*----------------------------------function----------------------------------*/
/**
 * @brief can过滤器初始化函数
 *  
 * @param hcan can句柄
 * @param idType 标准帧还是扩展帧
 * @param bank 过滤器组号
 * @param fifo 中断fifo号
 * @param id 过滤器id
 * @param maskId 过滤器掩码
 * @return void
 * */
void CAN_FILTER_Init(FDCAN_HandleTypeDef* hcan,uint32_t idType,uint32_t bank, uint32_t fifo, uint32_t id, uint32_t maskId);
/**
 * @brief can初始化函数
 * 
 * @param hcan 
 * @param pFunc 
 * @return void
 */
void BSP_FDCAN_Init(FDCAN_HandleTypeDef* hcan,void (*pFunc)(FDCAN_RxFrame_TypeDef *can_rx_instance));

/**
 * @brief 发送函数
 * 
 * @param can_tx_instance 
 */
void CAN_Transmit(FDCAN_TxFrame_TypeDef *can_tx_instance);

#endif /* BSP_CAN_H */
