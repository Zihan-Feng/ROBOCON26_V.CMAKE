/**
  ******************************************************************************
  * @file	 controller.h
  * @author  Wang Hongxi
  * @author  Zhang Hongyu (fuzzy pid)
  * @version V1.1.3
  * @date    2021/7/3
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */

#ifndef PID_CONTROLLER_H 
#define PID_CONTROLLER_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include <math.h>


#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

/******************************* PID CONTROL *********************************/
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000 不使用任何优化环节
    Integral_Limit = 0x01,              //0000 0001 使用积分限幅
    Derivative_On_Measurement = 0x02,   //0000 0010 微分先行
    Trapezoid_Intergral = 0x04,         //0000 0100 使用梯形积分
    Proportional_On_Measurement = 0x08, //0000 1000 比例先行
    OutputFilter = 0x10,                //0001 0000 输出滤波（LR）
    ChangingIntegrationRate = 0x20,     //0010 0000 变速积分
    DerivativeFilter = 0x40,            //0100 0000 微分滤波（LR）
    ErrorHandle = 0x80,                 //1000 0000 错误处理（电机堵转）
    IMCREATEMENT_OF_OUT = 0x100,        //0001 0000 0000 启用增量式输出
    Feedforward_CONTROLL = 0x200,       //0010 0000 0000 前馈控制
} PID_Improvement_e;

/**
 * @brief pid计算错误情况
 *  Motor_Blocked 电机堵转
 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;


/**
 * @brief 
 * 
 */
typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/**
 * @brief 
 * 
 */
typedef struct PID_T
{
    float Ref;
    float Kp;
    float Ki;
    float Kd;
    float FFJ;
    float FFB;

    float Measure;
    float Last_Measure;// 上次的测量值
    float Eriler_Measure;// 上上次的测量值
    float Err;
    float Last_Err;// 上一次的error
    float Eriler_Err;// 上上次的error
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float FFout;

    float Output;
    float Last_Output;
    float Last_Dout;// 上一次的输出

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float CoefA;         //For Changing Integral
    float CoefB;         //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;


    uint32_t DWT_CNT;
    float dt;


    uint16_t Improve;// 配置pid优化环节，使用 | 运算符连接，详细见枚举 PID_Improvement_e

    PID_ErrorHandler_t ERRORHandler;

    void (*User_Func1_f)(struct PID_T *pid);
    void (*User_Func2_f)(struct PID_T *pid);
} PID_t;

void PID_Init(PID_t *pid);


float PID_Calculate(PID_t *pid, float measure, float ref);

void PID_Reset(PID_t *pid);


#ifdef __cplusplus
}
#endif

#endif

