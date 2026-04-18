#include "pid_controller.h"

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Feedforward(PID_t *pid);                  // 前馈控制
static void f_Trapezoid_Intergral(PID_t *pid);          // 梯形积分
static void f_Integral_Limit(PID_t *pid);               // 积分限幅   
static void f_Derivative_On_Measurement(PID_t *pid);    // 基于测量值的微分
static void f_Changing_Integration_Rate(PID_t *pid);    // 变速积分
static void f_Output_Filter(PID_t *pid);                // 对输出进行滤波处理
static void f_Derivative_Filter(PID_t *pid);            // 不完全微分
static void f_Output_Limit(PID_t *pid);                 // 输出限幅
static void f_Proportion_Limit(PID_t *pid);             // 限制比例控制器输出
static void f_PID_ErrorHandle(PID_t *pid);   

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      PID结构体   PID structure
 * @param[in]      略
 * @retval         返回空      null
 */
void PID_Init(PID_t *pid)
{
    pid->Ref = 0;
    pid->ITerm = 0;
    pid->Err = 0;


    // DWT定时器计数变量清零
    // reset DWT Timer count counter
    pid->DWT_CNT = 0;
    pid->dt = 0;

    // 设置PID异常处理 目前仅包含电机堵转保护
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->FFout = 0;
    pid->Output = 0;
}// 误差计算

float PID_Calculate(PID_t *pid, float measure, float ref)
{
    uint8_t use_increment_output = 0;
    uint8_t use_feedforward = 0;

    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);
    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
    if (pid->dt <= 1e-6f)
        pid->dt = 1e-6f;

    use_increment_output = (pid->Improve & IMCREATEMENT_OF_OUT) ? 1U : 0U;
    use_feedforward = (pid->Improve & Feedforward_CONTROLL) ? 1U : 0U;

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

    if (((pid->Err > 0.0f) ? pid->Err : -pid->Err) > pid->DeadBand)
    {
        if (use_increment_output)
        {
            pid->Pout = pid->Kp * pid->Err - pid->Kp * pid->Last_Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            pid->Dout = pid->Kd * (pid->Err + pid->Eriler_Err - 2 * pid->Last_Err) / pid->dt;// 微分先行是计算测量值的变化率
        }
        else
        {
            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }
        
        if (pid->User_Func2_f != NULL)
            pid->User_Func2_f(pid);

        // 前馈
        if (pid->Improve & Feedforward_CONTROLL)
            f_Feedforward(pid);
        // 梯形积分
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器(不完全微分)
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;// 计算积分项

        if (use_increment_output && use_feedforward)
            pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Last_Output + pid->FFout;// 计算输出项
        else if (use_increment_output)
            pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Last_Output;// 计算输出项
        else if (use_feedforward)
            pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->FFout;// 计算输出项
        else
            pid->Output = pid->Pout + pid->Iout + pid->Dout;// 计算输出项

        // 输出滤波
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);

        // 无关紧要
        f_Proportion_Limit(pid);
    }
    else
    {
        pid->Output = 0;
        pid->Iout = 0;
    }
    pid->Eriler_Measure = pid->Last_Measure;
    pid->Last_Measure = pid->Measure;
    pid->Last_Dout = pid->Dout;
    pid->Eriler_Err = pid->Last_Err;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
    pid->Last_Output = pid->Output;

    return pid->Output;
}

void PID_Reset(PID_t *pid)
{
    pid->Ref = 0;
    pid->ITerm = 0;
    pid->Err = 0;
    pid->Pout = 0;
    pid->Iout = 0;
    pid->Dout = 0;
    pid->Output = 0;
    pid->Last_Output = 0;
    pid->Last_Measure = 0;
    pid->Eriler_Measure = 0;
    pid->Last_Err = 0;
    pid->Eriler_Err = 0;
    pid->Last_ITerm = 0;
    pid->Last_Dout = 0;
    pid->DWT_CNT = 0;
}

static void f_Feedforward(PID_t *pid)
{
    //电机速度环的加速度前馈
    pid->FFout = (pid->FFJ * (pid->Measure - pid->Last_Measure) / pid->dt) + (pid->FFB * pid->Measure);
}

// 梯形积分，积分项为两次采样值的平均
static void f_Trapezoid_Intergral(PID_t *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        // Integral still increasing
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral 全速积分
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;// 变速积分 CoefB < err <= CoefA + CoefB
        else
            // 误差较大，停止积分，避免积分饱和过冲
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;// 存储临时输出和积分项
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    // 抗积分饱和
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            pid->ITerm = 0;
        }
    }

    // 积分限幅
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    // 是否使用增量式pid
    if(pid->Improve & IMCREATEMENT_OF_OUT)
    {
        pid->Dout = pid->Kd * (pid->Measure + pid->Eriler_Measure - 2 * pid->Last_Measure) / pid->dt;// 微分先行是计算测量值的变化率
    }
    else
        pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;// 微分先行是计算测量值的变化率
}

// 对微分输出部分进行低通滤波
static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 对输出进行低通滤波
static void f_Output_Filter(PID_t *pid)
{
    // 指数平均滤波实现
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}


// 输出限幅
static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}


// 限制比例控制器输出
static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function 电机堵转检测
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}
