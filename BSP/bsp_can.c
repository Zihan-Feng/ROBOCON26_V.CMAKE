#include "fdcan.h"
#include "bsp_can.h"

static void (*pCAN1_RxCpltCallback)(FDCAN_RxFrame_TypeDef *);
static void (*pCAN2_RxCpltCallback)(FDCAN_RxFrame_TypeDef *);
static void (*pCAN3_RxCpltCallback)(FDCAN_RxFrame_TypeDef *);

void All_Can_Rx_It_Process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo);

void CAN_FILTER_Init(
    FDCAN_HandleTypeDef* hcan,
    uint32_t idType,
		uint32_t bank, 
		uint32_t fifo, 
		uint32_t id, 
		uint32_t maskId)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.FilterConfig = fifo;// 消息过滤器配置为接收FIFO0或FIFO1
  sFilterConfig.FilterID1 = id;// 过滤器ID1
	sFilterConfig.FilterID2 = maskId;// 过滤器ID2
	sFilterConfig.FilterIndex = bank;// 过滤器编号
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;// 掩码模式
	sFilterConfig.IdType = idType;// 标准帧或拓展帧
	sFilterConfig.IsCalibrationMsg = 0;// 不是标定消息
	sFilterConfig.RxBufferIndex = 0;

  if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void BSP_FDCAN_Init(FDCAN_HandleTypeDef* hcan,void (*pFunc)(FDCAN_RxFrame_TypeDef*)){
  //使能can接收FIFO0新消息中断	
  if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  //使能can接收FIFO1新消息中断
  if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  //使能can外设
  if (HAL_FDCAN_Start(hcan) != HAL_OK)
  {
    Error_Handler();
  }
  if (hcan->Instance == FDCAN1)
  {
    pCAN1_RxCpltCallback = pFunc;
  }
  else if (hcan->Instance == FDCAN2)
  {
    pCAN2_RxCpltCallback = pFunc;
  }
  else if (hcan->Instance == FDCAN3)
  {
    pCAN3_RxCpltCallback = pFunc;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
	{
		All_Can_Rx_It_Process(hfdcan, FDCAN_RX_FIFO0);
	}
	 
}
	
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{ 
  
  if (RxFifo1ITs == FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
	{
		All_Can_Rx_It_Process(hfdcan, FDCAN_RX_FIFO1);
	}
	 
}

void All_Can_Rx_It_Process(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo)
{
    static FDCAN_RxFrame_TypeDef can_rx_hdr;
    if (HAL_FDCAN_GetRxMessage(hfdcan, RxFifo, &can_rx_hdr.Header,can_rx_hdr.can_rx_buff) != HAL_OK)
    {
        return;
    }
    if (hfdcan == &hfdcan1)
    {
      pCAN1_RxCpltCallback(&can_rx_hdr);
    }
    else if (hfdcan == &hfdcan2)
    {
      pCAN2_RxCpltCallback(&can_rx_hdr);
    }
    else if (hfdcan == &hfdcan3)
    {
      pCAN3_RxCpltCallback(&can_rx_hdr);
    }
}

void CAN_Transmit(FDCAN_TxFrame_TypeDef *can_tx_instance)
{
    FDCAN_TxHeaderTypeDef can_tx_hdr;// 帧头
    can_tx_hdr.IdType = can_tx_instance->isExTid;// 标准帧还是扩展帧
    can_tx_hdr.Identifier = can_tx_instance->tx_id;// 发送id
    can_tx_hdr.DataLength = can_tx_instance->tx_len;// 发送数据帧长度，长度范围为0~8
    can_tx_hdr.TxFrameType = FDCAN_DATA_FRAME;// 数据帧
    can_tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;// 不使用比特率切换
    can_tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;// 激活错误状态指示器
    can_tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 不使用事件FIFO
    can_tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;// 经典CAN帧
    can_tx_hdr.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(can_tx_instance->hcan_handle, &can_tx_hdr, can_tx_instance->can_tx_buff) != HAL_OK)
    {
        Error_Handler();
    }
}

