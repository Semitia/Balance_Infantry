/*
 * @Author: DYH
 * @Date: 2021-06-28 10:07:51
 * @LastEditTime: 2021-07-01 19:50:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \624ctrl\PID.h
 */

/**
 * @description: initialize balence_feedback_PID structure
 * @param {*}angle_feedback_pid has been set externally
 * @return {*}null
 */

#ifndef WB_PID_H
#define WB_PID_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NOW     2
#define LAST    1
#define LLAST   0

typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    double set[3];				//目标值,包含NOW， LAST， LLAST上上次
    double get[3];				//测量值
    double err[3];				//误差
	
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
		float max_err;
		float deadband;						//err < deadband return
    double pid_mode;
    double MaxOutput;				//输出限幅
    double IntegralLimit;		//积分限幅
    
}pid_t;

//实现方式

//PID初始化
void PID_struct_init(pid_t *pid, float maxout, float max_iout, float kp, float ki, float kd);
//PID_struct_init(&M35081a, POSITION_PID, 1800, 150,M_Pa,M_Ia ,M_Da);

//PID计算
float pid_calc_arx(pid_t* pid, float get, float set);
//pid_calc(&M35081a,rmd_01.real_current/10,0);
//参数初始化
void PID_struct_init(pid_t *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;

    pid->get[0] = 0.0f;
    pid->get[1] = 0.0f;
    pid->get[2] = 0.0f;

    pid->IntegralLimit = max_iout;  //积分输出
    pid->MaxOutput = maxout;        //输出限幅
}

////////////ABS Limit
#define ABS(x)		((x>0)? x: -x) 
void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

/////////////PID函数

float pid_calc_arx(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    // if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
	// 		return 0;
	// if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
	// 		return 0;
    
    pid->pout = pid->p * pid->err[NOW];
    pid->iout += pid->i * pid->err[NOW];
    pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
    abs_limit(&(pid->iout), pid->IntegralLimit);
    pid->pos_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->pos_out), pid->MaxOutput);
    pid->last_pos_out = pid->pos_out;	//update last time 
   //printf("get=%.3f\tset=%.3f\terr=%.3f\n",pid->get[NOW], pid->set[NOW], pid->err[NOW]);
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pos_out;
//	
}

#ifdef __cplusplus
}
#endif

#endif /* WB_LED_H */