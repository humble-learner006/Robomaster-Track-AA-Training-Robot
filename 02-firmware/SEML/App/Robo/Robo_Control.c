#include "Robo_Control.h"
Task_Control_t control;
int flag=0;
__weak void Remote_Control(void)
{
	float temp, temp1, temp2;
	Motor_t *motor;
#define RC_MOVE_MAX 0.3f
#define RC_MOVE_SENSITIVITY (RC_MOVE_MAX / 660.0f)
#define RC_ADD_SENSITIVITY (1.0f / 660.0f)
	// 移动
	temp = (RC_Ctrl.rc.ch0 - DR16_RC_OFFSET) * RC_MOVE_SENSITIVITY;
	temp1 = (RC_Ctrl.rc.ch1 - DR16_RC_OFFSET) * RC_MOVE_SENSITIVITY;
	// 移动速度归一化
	temp2 = math_sqrt(temp * temp + temp1 * temp1);
	if (temp2 > RC_MOVE_MAX)
	{
		temp2 = RC_MOVE_MAX / temp2;
		temp *= temp2;
		temp1 *= temp2;
	}
	Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", temp1);
	temp = ((RC_Ctrl.rc.ch2 - DR16_RC_OFFSET)) * RC_MOVE_SENSITIVITY;
	Robo_Push_message_Cmd("Set_Rotate_Speed", temp);
	
	Robo_Get_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Get_message_Cmd("Set_Move_Y_Speed", temp1);
	/*
	if(temp1 != 0 || temp != 0)
	{
		Robo_Get_message_Cmd("Rotate", temp);
		temp2 = Ramp_float(0.7f * PI,temp,0.02f);
		Robo_Push_message_Cmd("Rotate", temp2);
	}
	else
	{
		Robo_Get_message_Cmd("Rotate", temp);
		temp2 = Ramp_float(1.2f * PI,temp,0.02f);
		Robo_Push_message_Cmd("Rotate", temp2);
	}*/
}

__weak void Keyboard_Control(void)
{
	float temp, temp1, temp2;
	Motor_t *motor;
#define KB_MOVE_MAX 1.0f
#define KB_MOVE_SENSITIVITY (RC_MOVE_MAX / 660.0f)
#define KB_ADD_SENSITIVITY (1.0f / 660.0f)
	// 移动
	temp = 0.0f;
	temp1 = 0.0f;
	if(RC_Ctrl.keyboard.key_W)
		temp += 1.0f;
	if(RC_Ctrl.keyboard.key_S)
		temp -= 1.0f;
	if(RC_Ctrl.keyboard.key_A)
		temp1 += 1.0f;
	if(RC_Ctrl.keyboard.key_D)
		temp1 -= 1.0f;
	// 移动速度归一化
	temp2 = math_sqrt(temp * temp + temp1 * temp1);
	if (temp2 > RC_MOVE_MAX)
	{
		temp2 = RC_MOVE_MAX / temp2;
		temp *= temp2;
		temp1 *= temp2;
	}
	Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", temp1);
	// 云台yaw跟随
	Robo_Get_message_Cmd("Set_Gimbal_Yaw_Angle", temp);
	temp -= RC_Ctrl.mouse.x * 2 * PI / 1024.0f * 0.001f;
	if (temp >= PI)
		temp -= 2 * PI;
	else if (temp < -PI)
		temp += 2 * PI;
	Robo_Push_message_Cmd("Set_Gimbal_Yaw_Angle", temp);
	// 云台pitch跟随
	Robo_Get_message_Cmd("Set_Gimbal_Pitch_Angle", temp);
	temp += RC_Ctrl.mouse.y * 2 * PI / 1024.0f * 0.001f;
	if (temp >= 21 / 360.0f * 2 * PI)
		temp = 21 / 360.0f * 2 * PI;
	else if (temp < -5 / 360.0f * 2 * PI)
		temp = -5 / 360.0f * 2 * PI;
	Robo_Push_message_Cmd("Set_Gimbal_Pitch_Angle", temp);
	// 底盘yaw跟随
	temp = 4.328f;
	Robo_Push_message_Cmd("Set_Chassis_Yaw_Angle", temp);
	Robo_Get_message_Cmd("Gimbal_Yaw_Motor", motor);
	temp = Get_Motor_Position_Data(motor);
	
	Robo_Push_message_Cmd("Real_Chassis_Yaw_Angle", temp);
}

__weak void Auto_Control(void)
{
	float temp, temp1, temp2;
#define Auto_MOVE_MAX 1.0f
#define Auto_MOVE_SENSITIVITY (RC_MOVE_MAX / 10.0f)
#define Auto_ADD_SENSITIVITY (1.0f / 660.0f)
	// 移动
	Robo_Get_message_Cmd("Forward", temp);
	Robo_Get_message_Cmd("Left", temp1);
	temp *= Auto_MOVE_SENSITIVITY;
	temp1 *= Auto_MOVE_SENSITIVITY;
	Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", temp1);
	Robo_Get_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Get_message_Cmd("Set_Move_Y_Speed", temp1);
	if(temp1 != 0 || temp != 0)
	{
		Robo_Get_message_Cmd("Rotate", temp);
		temp2 = Ramp_float(0.2f * PI,temp,0.3f);
		Robo_Push_message_Cmd("Rotate", temp2);
	}
	else
	{
		Robo_Get_message_Cmd("Rotate", temp);
		temp2 = Ramp_float(0.7f * PI,temp,0.3f);
		Robo_Push_message_Cmd("Rotate", temp2);
	}
	
	/*
	Robo_Get_message_Cmd("Rotate", temp1);
	temp -= temp1;
	if (temp >= PI)
		temp -= 2 * PI;
	else if (temp < -PI)
		temp += 2 * PI;
	temp = 4.328f;
	Robo_Push_message_Cmd("Set_Chassis_Yaw_Angle", temp);*/
}

__weak void Robo_Control_Init(void *config)
{
	float temp = 0;
	Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", temp);
	Robo_Push_message_Cmd("Set_Gimbal_Yaw_Angle", temp);
	Robo_Push_message_Cmd("Set_Chassis_Yaw_Angle", temp);
	Robo_Push_message_Cmd("Set_Arm_X_Location",temp);
	temp = AHRS.euler_angle.roll;
	Robo_Get_message_Cmd("Set_Gimbal_Pitch_Angle", temp);
}
__weak void Robo_Control_Task(void *config)
{
	uint32_t temp;
	float temp_f;
	int flag;
	temp_f = AHRS.euler_angle.yaw;
	Robo_Push_message_Data("Set_Gimbal_Imu_Data", (void *)&AHRS.euler_angle, sizeof(AHRS.euler_angle));
	Robo_Push_message_Cmd("Real_Chassis_Yaw_Angle", temp_f);
	// S1控制源
	switch (RC_Ctrl.rc.s1)
	{
	case RC_S_DOWM:
		control.remote_source = Scource_None;
		temp = 0;
		robo_control_flag.remote_off = 1;
		Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
		Robo_Push_message_Cmd("Set_Move_Y_Speed", temp);
		temp = Shoot_Off;
		Robo_Push_message_Cmd("Shoot_Mode", temp);
		flag = 0;
		Robo_Push_message_Cmd("Set_AA_on", flag);
		break;
	case RC_S_MID:
		control.remote_source = Scource_Remote;
		robo_control_flag.remote_off = 0;
		// temp = Chassis_Normal_Mode;
		// Robo_Push_message_Cmd("Chassis_Rotate_Mode", temp);
		Remote_Control();
		// s2模式
		switch (RC_Ctrl.rc.s2)
		{
		case RC_S_DOWM:
			temp = 0;
			Robo_Push_message_Cmd("Yaw_Start", temp);
			temp = 0;
			Robo_Push_message_Cmd("Shoot_System_Start", temp);
			break;
		case RC_S_MID:
			temp = 1;
			Robo_Push_message_Cmd("Yaw_Start", temp);
			temp = 0;
			Robo_Push_message_Cmd("Shoot_System_Start", temp);
			break;
		case RC_S_UP:
			temp = 0;
			Robo_Push_message_Cmd("Yaw_Start", temp);
			temp = 1;
			Robo_Push_message_Cmd("Shoot_System_Start", temp);
			break;
		default:
			break;
		}
		break;
	case RC_S_UP:
		// control.remote_source = Scource_Key;
		// robo_control_flag.remote_off = 0;
		// temp = Rotate_Made;
		// Robo_Push_message_Cmd("Chassis_Rotate_Mode", temp);
		//Auto_Control();
		break;
	}
}
