#include "Robo_Gimbal.h"
Task_Gimbal_t gimbal;
/**
 * @brief 云台任务初始化
 * 该函数执行云台任务相关的初始化操作
 * @note 该函数定义为弱函数，可以依此为模板定义新的同名函数替换
 */
__weak void Gimbal_Init(void)
{
	// static Dji_Motor_t pitch_motor,yaw_motor;
	// static SISO_Controller_t yaw_s_controller,pitch_s_controller;
	// float temp;
	// // 绑定电机
	// gimbal.pitch.motor = DJI_Motor_Init(&pitch_motor, DJI_Motor_GM6020, 6, &can2);
	// gimbal.yaw.motor = DJI_Motor_Init(&yaw_motor, DJI_Motor_GM6020, 1, &can1);
	// // 发送电机话题
	// Robo_Push_message_Cmd("Gimbal_Pitch_Motor",gimbal.pitch.motor);
	// Robo_Push_message_Cmd("Gimbal_Yaw_Motor",gimbal.yaw.motor);
	// // 初始化pid
	// temp = Get_DJI_Motor_Control_Max(&pitch_motor);
	// PID_Init(&gimbal.pitch.speed_PID, 800, 1000, 0.7f, temp, -temp, 0.001f);
	// PID_Init(&gimbal.pitch.position_PID, 120, 0, 4, 360, -360, 0.001f);

	// PID_Init(&gimbal.pitch.speed_PID_AA, 800, 1000, 0, temp, -temp, 0.001f);
	// PID_Init(&gimbal.pitch.position_PID_AA, 5, 0, 0.3f, 30, -30, 0.001f);

	// temp = Get_DJI_Motor_Control_Max(&yaw_motor);
	// PID_Init(&gimbal.yaw.speed_PID, 1000, 1000, 0, temp, -temp, 0.001f);
	// PID_Init(&gimbal.yaw.position_PID, 180, 0, 3, 1080, -1080, 0.001f);

	// //PID_Init(&gimbal.yaw.speed_PID_AA, 1000, 1000, 0, temp, -temp, 0.001f);
	// //PID_Init(&gimbal.yaw.position_PID_AA, 100, 0, 3, 1080, -1080, 0.001f);
	// PID_Init(&gimbal.yaw.speed_PID_AA, 430, 200, 1.5f, temp, -temp, 0.001f);
	// PID_Init(&gimbal.yaw.position_PID_AA, 1, 0, 0.15f, 30, -30, 0.001f);
	// // 初始化变量
	// gimbal.pitch.expect_angle = AHRS.euler_angle.pitch;
	// gimbal.yaw.expect_angle = AHRS.euler_angle.yaw;
}
/**
 * @brief 云台任务
 * @note 该函数定义为弱函数，可以依此为模板定义新的同名函数替换
 */
__weak void Gimbal_Task(void *conifg)
{
	// float temp,temp1,temp2;
	// static float power;
	// // 获取话题
	// Robo_Get_message_Cmd("Set_Gimbal_Yaw_Angle", gimbal.yaw.expect_angle);
	// Robo_Get_message_Cmd("Set_Gimbal_Pitch_Angle", gimbal.pitch.expect_angle);
	// Robo_Get_message_Data("Set_Gimbal_Imu_Data", (void*)&gimbal.imu);
	// if (robo_control_flag.remote_off)
	// {
	// 	Motor_Send_Data(gimbal.yaw.motor,0);
	// 	Motor_Send_Data(gimbal.pitch.motor,0);
	// 	return;
	// }

	// // yaw闭环
	// temp = Zero_Crossing_Process(2 * PI, gimbal.yaw.expect_angle + PI, gimbal.imu->yaw + PI);
	// Robo_Get_message_Cmd("Set_AA_on", temp1);
	// if(temp1)
	// {
	// 	gimbal.yaw.expect_speed = Basic_PID_Controller(&gimbal.yaw.position_PID_AA, temp, gimbal.imu->yaw + PI);
	// 	gimbal.yaw.send_data = Basic_PID_Controller(&gimbal.yaw.speed_PID_AA,gimbal.yaw.expect_speed,Get_Motor_Speed_Data(gimbal.yaw.motor));
	// }
	// else
	// {
	// 	gimbal.yaw.expect_speed = Basic_PID_Controller(&gimbal.yaw.position_PID, temp, gimbal.imu->yaw + PI);
	// 	gimbal.yaw.send_data = Basic_PID_Controller(&gimbal.yaw.speed_PID,gimbal.yaw.expect_speed,Get_Motor_Speed_Data(gimbal.yaw.motor));
	// }
	// Motor_Send_Data(gimbal.yaw.motor,gimbal.yaw.send_data);

	
	// // pitch闭环
	// temp = Zero_Crossing_Process(2 * PI, gimbal.pitch.expect_angle + PI, gimbal.imu->pitch + PI);
	// Robo_Get_message_Cmd("Set_AA_on", temp1);
	// if(temp1)
	// {
	// 	gimbal.pitch.expect_speed = Basic_PID_Controller(&gimbal.pitch.position_PID_AA, temp, gimbal.imu->pitch + PI);
	// 	temp2 = (-9e-08f * gimbal.pitch.expect_angle + 0.0002f) * gimbal.pitch.expect_angle + 0.6147f;
	// 	gimbal.pitch.send_data = Basic_PID_Controller(&gimbal.pitch.speed_PID_AA,gimbal.pitch.expect_speed,Get_Motor_Speed_Data(gimbal.pitch.motor)) +temp2;
	// }
	// else
	// {
	// 	gimbal.pitch.expect_speed = Basic_PID_Controller(&gimbal.pitch.position_PID, temp, gimbal.imu->pitch + PI);
	// 	temp2 = (-9e-08f * gimbal.pitch.expect_angle + 0.0002f) * gimbal.pitch.expect_angle + 0.6147f;
	// 	gimbal.pitch.send_data = Basic_PID_Controller(&gimbal.pitch.speed_PID,gimbal.pitch.expect_speed,Get_Motor_Speed_Data(gimbal.pitch.motor)) +temp2;
	// }
	// Motor_Send_Data(gimbal.pitch.motor,gimbal.pitch.send_data);
	// // Motor_Speed_Ctrl(gimbal.pitch.motor,gimbal.pitch.expect_speed);
}