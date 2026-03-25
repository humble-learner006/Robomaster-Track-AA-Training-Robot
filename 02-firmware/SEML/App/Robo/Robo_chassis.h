#ifndef _ROBO_CHASSIS_H_
#define _ROBO_CHASSIS_H_
#endif
#include "main.h"
#include "SEML_common.h"
#include "Robo_Common.h"
#include "chassis.h"
#include "Dji_Motor.h"
#include "PID.h"
#include <stdlib.h>

typedef struct
{
	Chassis_Kinematic_t solve;						/**< 底盘运动学解算句柄 */
	Chassis_Velocity_t expect_speed;			/**< 期望速度 */
	Chassis_Velocity_t observation_speed; /**< 观测数据 */
	Chassis_Mode_t mode;

	float power_limit;
	float current_power;

	struct
	{
		Motor_t *motor;
		PIDConfig_t speed_PID;
		float send_data;
	} driver[4];
	struct
	{
		Motor_t *motor;
		PIDConfig_t speed_PID;
		PIDConfig_t angle_PID;
		float expect_speed;
		float send_data;
	} steer[4];
	float expect_driver_speed[4];
	float expect_steer_angle[4];

} Task_Chassis_t;

typedef enum
{
    POWER_PRIORITY = 1,
	HP_PRIORITY = 0,
};

/**
 * @brief 底盘任务初始化
 * 该函数执行底盘任务相关的初始化操作
 */
void Chassis_Init(void);
/**
 * @brief 底盘任务
 */
void Chassis_Task(void *conifg);