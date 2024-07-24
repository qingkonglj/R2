#ifndef _PID_H
#define _PID_H
#include "stdint.h"




typedef enum
{
	PID_Position,
	PID_Speed
}PID_ID;
// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式(KEIL不支持2进制，改用16进制)
typedef enum
{
  PID_IMPROVE_NONE                 = 0x0000,              // 0000 0000
	PID_Integral_Limit               = 0x0001,              // 0000 0001
	PID_Derivative_On_Measurement    = 0x0002,              // 0000 0010
	PID_Trapezoid_Intergral          = 0x0004,              // 0000 0100
	PID_Proportional_On_Measurement  = 0x0008,              // 0000 1000
	PID_OutputFilter                 = 0x0010,              // 0001 0000
//	PID_ChangingIntegrationRate      = 0x0020,              // 0010 0000
	PID_DerivativeFilter             = 0x0040,              // 0100 0000
	PID_ErrorHandle                  = 0x0080,              // 1000 0000
} PID_Improvement_e;
typedef struct _PID_TypeDef
{
	PID_ID id;
	PID_Improvement_e Improve;
	float target;		            //目标值
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float measure;		          //测量值
	float last_measure;         //上次测量值
	float err;			            //误差
	float last_err;             //上次误差
	
//	float iterm_cofeA;          //积分项系数
//	float iterm_cofeB;          //积分项系数
	
	float pout;
	float iout;
	float dout;
	float iout_temp;
	float last_dout;
	
	float output;				         //本次输出
	float last_output;			     //上次输出
	
	float MaxOutput;			       //输出限幅
	float IntegralLimit;		     //积分限幅
	float DeadBand;			  	     //死区（绝对值）
	float ControlPeriod;		     //控制周期
	float  Max_Err;				       //最大误差
	
	float derivative_lpf_rc;     //微分一阶低通滤波器的滤波系数
	float output_lpf_rc;         //输出一阶低通滤波器的滤波系数
	
	
	uint32_t thistime;
	uint32_t lasttime;
	float dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
						 PID_ID id,
						 float maxOutput,
						 float integralLimit,
						 float deadband,
						 float controlPeriod,
						 float max_err,     
						 float  target,
						 float kp,
						 float ki,
						 float kd,
						 PID_Improvement_e Improve);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid计算

}PID_TypeDef;



void pid_init(PID_TypeDef* pid);

#endif

