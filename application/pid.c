#include "pid.h"
#include "stm32f4xx.h"
#include "math.h"

#define ABS(x)		((x>0)? x: -x)
#define RAMP_YawUp_Acc 150
#define RAMP_YawDown_Acc 150


/**
  * @brief          PID参数初始化
  * @param[in]      none
  * @retval         none
  */
static void pid_param_init( PID_TypeDef *pid,
							PID_ID   id,
							float maxout,
							float intergral_limit,
							float deadband,
							float period,
							float  max_err,
							float  target,
							float 	kp,
							float 	ki,
							float 	kd,
							PID_Improvement_e Improve)
{
	pid->id = id;

	pid->ControlPeriod = period;             //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->output = 0;
	pid->Improve = Improve;

}

/**
  * @brief          中途更改参数设定
  */
static void pid_reset(PID_TypeDef *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/**
  * @brief          梯形积分
  */
static void f_trapezoid_intergral(PID_TypeDef *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->iout_temp = pid->ki * ((pid->err + pid->last_err) / 2) * pid->dtime;
	
//TODO:将dtime赋值
	
}

///**
//  * @brief       变速积分
//  */
//static void f_changing_integration_rate(PID_TypeDef *pid)
//{
//    if (pid->err * pid->iout > 0)
//    {
//        // 积分呈累积趋势
//        if (ABS(pid->err) <= pid->iterm_cofeB)
//            return; // Full integral
//        if (ABS(pid->err) <= (pid->iterm_cofeA + pid->iterm_cofeB))
//						{
//								pid->iout_temp *= (pid->iterm_cofeA - ABS(pid->err) + pid->iterm_cofeB) / pid->iterm_cofeA;			
//						}
//        else // 最大阈值,不使用积分
//            pid->iout_temp = 0;
//    }
//}

/**
  * @brief   积分限幅
  */
static void f_integral_limit(PID_TypeDef *pid)
{
    static float temp_output, temp_iout;
    temp_iout = pid->iout + pid->iout_temp;
    temp_output = pid->pout + pid->iout + pid->dout;
	
    if (ABS(temp_output) > pid->MaxOutput)
    {
        if (pid->err * pid->iout > 0) // 积分却还在累积
        {
            pid->iout_temp = 0; // 当前积分项置零
        }
    }

    if (temp_iout > pid->IntegralLimit)
    {
        pid->iout_temp = 0;
        pid->iout = pid->IntegralLimit;
    }
    if (temp_iout < -pid->IntegralLimit)
    {
        pid->iout_temp = 0;
        pid->iout = -pid->IntegralLimit;
    }
}
/**
  * @brief 微分先行(仅使用反馈值而不计参考输入的微分)
  * 
  */
static void f_derivative_on_measurement(PID_TypeDef *pid)
{
    pid->dout = pid->kd * (pid->last_measure - pid->measure) / pid->dtime;
	
//TODO:将dtime和last_measure赋值
	
}
/**
  * @brief 微分滤波(采集微分时,滤除高频噪声)
  * 
  */
static void f_derivative_filter(PID_TypeDef *pid)
{
    pid->dout = pid->dout * pid->dtime / (pid->derivative_lpf_rc + pid->dtime) +
                pid->last_dout * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dtime);
}
/**
  * @brief 输出滤波
  * 
  */
static void f_output_filter(PID_TypeDef *pid)
{
    pid->output = pid->output * pid->dtime / (pid->output_lpf_rc + pid->dtime) +
                  pid->last_output * pid->output_lpf_rc / (pid->output_lpf_rc + pid->dtime);
}

/**
  * @brief 输出限幅
  * 
  */
static void f_output_limit(PID_TypeDef *pid)
{
    if (pid->output > pid->MaxOutput)
    {
        pid->output = pid->MaxOutput;
    }
    if (pid->output < -(pid->MaxOutput))
    {
        pid->output = -(pid->MaxOutput);
    }
}

/**
  * @brief          pid计算
  */
static float pid_calculate(PID_TypeDef *pid, float measure)//, int16_t target)
{
	//	uint32_t time,lasttime;


	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime - pid->lasttime;

	pid->measure = measure;
	//	pid->target = target;

	pid->err = pid->target - pid->measure;

	//是否进入死区
	if((ABS(pid->err) > pid->DeadBand))
	{
			pid->pout       =  pid->kp * pid->err;
			pid->iout_temp  =  (pid->ki * pid->err) * pid->dtime;
			pid->dout       =  pid->kd * (pid->err - pid->last_err) / pid->dtime;

			pid->iout   +=  pid->iout_temp;
			pid->output  =  pid->pout + pid->iout + pid->dout;
		
//			if(pid->Improve & PID_ChangingIntegrationRate)
//			{
//				f_changing_integration_rate(pid);
//			}
			if(pid->Improve & PID_Trapezoid_Intergral)
			{
				f_trapezoid_intergral(pid);
			}
		  if (pid->Improve & PID_Integral_Limit)
			{
				f_integral_limit(pid);
			}
			if(pid->Improve & PID_Derivative_On_Measurement)
			{
				f_derivative_on_measurement(pid);
			}
//			if(pid->Improve & PID_Proportional_On_Measurement)
//			{
//				
//			}
			if(pid->Improve & PID_OutputFilter)
			{
				f_output_filter(pid);
			}
			if(pid->Improve & PID_DerivativeFilter)
			{
				f_derivative_filter(pid);
			}
			//输出限幅
			f_output_limit(pid);

	}else
	{
		pid->err  = 0;
		pid->pout = 0;
    pid->iout = 0;
		pid->dout = 0;
		pid->output = 0;
	}
	
	//缓存数据
	pid->last_err     = pid->err;
	pid->last_output  = pid->output;
	pid->last_dout    = pid->dout;
	pid->last_measure = pid->measure;
	pid->lasttime = pid->thistime;

	return pid->output;
}



///* ----------------------------下面是pid优化环节的实现---------------------------- */



//// 电机堵转检测
//static void f_PID_ErrorHandle(PIDInstance *pid)
//{
//    /*Motor Blocked Handle*/
//    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
//        return;

//    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
//    {
//        // Motor blocked counting
//        pid->ERRORHandler.ERRORCount++;
//    }
//    else
//    {
//        pid->ERRORHandler.ERRORCount = 0;
//    }

//    if (pid->ERRORHandler.ERRORCount > 500)
//    {
//        // Motor blocked over 1000times
//        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
//    }
//}

///* ---------------------------下面是PID的外部算法接口--------------------------- */

///**
// * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
// *
// * @param pid    PID实例
// * @param config PID初始化设置
// */
//void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
//{
//    // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
//    // @todo: 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
//    // 后续修改为逐个赋值
//    memset(pid, 0, sizeof(PIDInstance));
//    // utilize the quality of struct that its memeory is continuous
//    memcpy(pid, config, sizeof(PID_Init_Config_s));
//    // set rest of memory to 0
//    DWT_GetDeltaT(&pid->DWT_CNT);
//}

///**
// * @brief          PID计算
// * @param[in]      PID结构体
// * @param[in]      测量值
// * @param[in]      期望值
// * @retval         返回空
// */
//float PIDCalculate(PIDInstance *pid, float measure, float ref)
//{
//    // 堵转检测
//    if (pid->Improve & PID_ErrorHandle)
//        f_PID_ErrorHandle(pid);

//    pid->dt = DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

//    // 保存上次的测量值和误差,计算当前error
//    pid->Measure = measure;
//    pid->Ref = ref;
//    pid->Err = pid->Ref - pid->Measure;

//    // 如果在死区外,则计算PID
//    if (abs(pid->Err) > pid->DeadBand)
//    {
//        // 基本的pid计算,使用位置式
//        pid->Pout = pid->Kp * pid->Err;
//        pid->ITerm = pid->Ki * pid->Err * pid->dt;
//        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

//        // 梯形积分
//        if (pid->Improve & PID_Trapezoid_Intergral)
//            f_Trapezoid_Intergral(pid);
//        // 变速积分
//        if (pid->Improve & PID_ChangingIntegrationRate)
//            f_Changing_Integration_Rate(pid);
//        // 微分先行
//        if (pid->Improve & PID_Derivative_On_Measurement)
//            f_Derivative_On_Measurement(pid);
//        // 微分滤波器
//        if (pid->Improve & PID_DerivativeFilter)
//            f_Derivative_Filter(pid);
//        // 积分限幅
//        if (pid->Improve & PID_Integral_Limit)
//            f_Integral_Limit(pid);

//        pid->Iout += pid->ITerm;                         // 累加积分
//        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

//        // 输出滤波
//        if (pid->Improve & PID_OutputFilter)
//            f_Output_Filter(pid);

//        // 输出限幅
//        f_Output_Limit(pid);
//    }
//    else // 进入死区, 则清空积分和输出
//    {
//        pid->Output = 0;
//        pid->ITerm = 0;
//    }

//    // 保存当前数据,用于下次计算
//    pid->Last_Measure = pid->Measure;
//    pid->Last_Output = pid->Output;
//    pid->Last_Dout = pid->Dout;
//    pid->Last_Err = pid->Err;
//    pid->Last_ITerm = pid->ITerm;

//    return pid->Output;
//}

/**
  * @brief          pid结构体初始化，每一个pid参数需要调用一次

  */
void pid_init(PID_TypeDef *pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}
