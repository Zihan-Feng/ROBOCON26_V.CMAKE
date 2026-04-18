#ifndef BSP_USART_H 
#define BSP_USART_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
/* c标准库接口 */
#include <stdint.h>

/* bsp层直接操作hal库底层 */
#include "usart.h"

/* freertos接口，用于提供堆管理 */
#include "FreeRTOS.h"
#include "task.h"
/*-----------------------------------macro------------------------------------*/
#define DEVICE_UART_CNT 8 // 学院设计的外设板至多分配8个串口
#define UART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里
/*----------------------------------typedef-----------------------------------*/
/**
 * @brief 串口回调函数定义
 * 
 */
typedef uint8_t (*uart_callback_t)(void* uart_device,uint16_t rx_buf_num);// 定义回调函数类型

/* 串口包数据结构体类型 */
typedef struct
{
    UART_HandleTypeDef *uart_handle;// hal库usart句柄成员
    uint8_t use_static_length_data;// 是否接收定长数据，1：是 0：否
    uint16_t rx_buffer_size;// 定义buffer的大小
    uint8_t *rx_buffer;// 定义一个buffer
    uart_callback_t uart_callback;// 定义回调函数指针
    uint8_t IT_CHOOSE ;//0：DWA空闲接收中断 1：关闭DWA，普通中断
}uart_package_t;

/* uart instance 串口设备实例 */
typedef struct 
{
    /* 串口接收包结构体 */
    uart_package_t uart_package;
    void *device;// 父指针，储存设备指针
    uint8_t (*Uart_Deinit)(void *);// 串口设备注销函数
}Uart_Instance_t;

typedef struct 
{
    UART_HandleTypeDef *uart_handle;// hal库usart句柄成员
    uint8_t *tx_buffer;// 发送缓存
    uint8_t tx_buffer_size;// 发送缓存大小
}Uart_Tx_Package_t;
/*----------------------------------function----------------------------------*/
/**
 * @brief 串口设备注册函数，用户通过创建一个实例指针和串口数据包，然后通过调用此函数以及将实例传入本函数来获取返回值的实例
 *        实现串口设备的动态注册，如果创建失败会自动free内存
 * 
 * @param uart_config     uart_package_t* 串口数据包 ,IDLE空闲中断是否开启，IT中断是否开启，DMA是否开启
 * @param queue_length    uint32_t 队列中所能存储的元素数
 * @param queue_data      size_t 队列元素的大小，使用 sizeof() 获取，注意数据类型是size_t
 * @return Uart_Instance_t* NULL 创建失败
 *                          实例  创建成功
 */
Uart_Instance_t* Uart_Register(uart_package_t *uart_config);


/**
 * @brief 串口中断管理函数
 *        这个插入在hal库的it.c代码中，可以强行把hal库对应的串口中断管理转移到这边进行处理
 * 
 * @param uart_instance 串口设备实例
 * @return uint8_t --- 1 :success
 *                 --- 0 :failed
 */
uint8_t Uart_Receive_Handler(Uart_Instance_t *uart_instance);

uint8_t XBOX_Receive_Handler(Uart_Instance_t *uart_instance);

/**
 * @brief 串口注销函数
 * 
 * @param uart_instance 
 * @return uint8_t 
 */
uint8_t Uart_UnRegister(void *uart_instance);


/**
 * @brief 阻塞式发送，不推荐使用，会阻塞线程
 * 
 * @param tx_package 
 * @return uint8_t 
 */
uint8_t Uart_Tx_By_Blocking(Uart_Tx_Package_t tx_package);


/**
 * @brief 不阻塞发送，利用中断发送，推荐打开发送中断，做一些操作来显示是否正常发送，当然也可以不开，可有可无的操作
 * 
 * @param tx_package 
 * @return uint8_t 
 */
uint8_t Uart_Tx_By_It(Uart_Tx_Package_t tx_package);


/**
 * @brief 利用DMA进行发送，需要先在cubemx打开配置，推荐使用！
 * 
 * @param tx_package 
 * @return uint8_t 
 */
uint8_t Uart_Tx_By_DMA(Uart_Tx_Package_t tx_package);


#ifdef __cplusplus
}
#endif

#endif	/* BSP_USART_H */
