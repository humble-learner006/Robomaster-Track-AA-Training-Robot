#include "Robo_Chassis.h"
Task_Chassis_t chassis;

float Car_X;
float Car_Y;
float Car_W;
float FL;
float FR;
float BL;
float BR;
float init_yaw;
/**
 * @brief 将运动速度从云台坐标系转换为底盘坐标系
 * @param old_coordinate_velcoity 云台坐标系速度
 * @param expect_angle 期望角(云台角)
 * @param real_angle 实际角(底盘角)
 * @return 底盘坐标系结果
 */

Chassis_Velocity_t coordinate_conversion(Chassis_Velocity_t *old_coordinate_velcoity, float expect_angle, float real_angle);
/**
 * @brief 底盘任务初始化
 * 该函数执行底盘任务相关的初始化操作
 * @note 该函数定义为弱函数，可以依此为模板定义新的同名函数替换
 */
__weak void Chassis_Init(void)
{
	static Dji_Motor_t chassis_driver_motor[3];

	
	int16_t temp;
	
	// 绑定底盘电机
	//motor1 - id = 3
	chassis.driver[CHASSIS_FL].motor = DJI_Motor_Init(&chassis_driver_motor[0], DJI_Motor_C620, 3, &can1);
	//motor2 - id = 6
	chassis.driver[CHASSIS_FR].motor = DJI_Motor_Init(&chassis_driver_motor[1], DJI_Motor_C620, 6, &can1);
	//motor3 - id = 4
	chassis.driver[CHASSIS_BL].motor = DJI_Motor_Init(&chassis_driver_motor[2], DJI_Motor_C620, 4, &can1);
	
	// PID初始化
	temp = Get_DJI_Motor_Control_Max(chassis_driver_motor);
	PID_Init(&chassis.driver[CHASSIS_FL].speed_PID, 250, 1000, 0, temp, -temp, 0.001f);
	PID_Init(&chassis.driver[CHASSIS_FR].speed_PID, 250, 1000, 0, temp, -temp, 0.001f);
	PID_Init(&chassis.driver[CHASSIS_BL].speed_PID, 250, 1000, 0, temp, -temp, 0.001f);
	


}

/**
 * @brief 底盘任务
 * @note 该函数定义为弱函数，可以依此为模板定义新的同名函数替换
 */


static uint32_t motor_timer = 0;
static int motor_state = 0;  // 0=idle, 1=forward, 2=backward
static int constant = 1;

  
__weak void Chassis_Task(void *conifg)
{
	uint32_t elapsed_time = HAL_GetTick() - motor_timer;
    
  const uint32_t RUN_TIME_MS = 1500;
	
	Chassis_Velocity_t velcoity;
	float temp, temp2, temp_array[4];
	float spin_speed;
	chassis.expect_driver_speed[0] = -150 * constant;//正：离开电机 负：靠近电机
	chassis.expect_driver_speed[1] = 150 * constant;
	chassis.expect_driver_speed[2] = 300;//正：顺时针  负：逆时针

	

	// 功率限制

	for (int i = 0; i < 3; i++)
	{
		
		temp = Get_Motor_Speed_Data(chassis.driver[i].motor);
		chassis.driver[i].send_data = Basic_PID_Controller(&chassis.driver[i].speed_PID, chassis.expect_driver_speed[i], temp);
		
		Motor_Send_Data(chassis.driver[i].motor, chassis.driver[i].send_data);
	}
	
	//时间控制
	if(motor_state == 0 && elapsed_time < RUN_TIME_MS){//0-2的前进
			constant=1;
			motor_state = 0;
		}
		else if(motor_state == 0 && elapsed_time >= RUN_TIME_MS){//2s的前进转后退
			motor_state = 1;
			motor_timer = HAL_GetTick();
		}
		else if(motor_state == 1 && elapsed_time < RUN_TIME_MS){//0-2s的后退
			constant = -1;
			motor_state = 1;
		}
		else if(motor_state == 1 && elapsed_time >= RUN_TIME_MS){//2s的后退转前进
			motor_state = 0;
			motor_timer = HAL_GetTick();
		}
	
	
	
	
	
	
}