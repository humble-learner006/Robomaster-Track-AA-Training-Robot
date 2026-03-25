#include "RC_Ctrl_Key.h"
Coordinates_t Absolute_Speed; //底盘绝对坐标系速度，一个速度暂存值中间变量
eChassisAction Chassis_Mode = CHASSIS_FOLLOW_GIMBAL;   //默认底盘跟随云台行走
Key_Mode chassic_mode;
extern Task_Gimbal_t gimbal;
extern Keyboard_t keyboard;
extern RC_Ctrl_t RC_Ctrl;
extern Task_Chassis_t chassis;
uint16_t Get_Chassis_Pow_Limit;          //获取功率上限
/**
 * @brief 键盘模式初始化
 * @param keyboard 键盘按键句柄
 */
void Key_Init(Keyboard_t*keyboard)
{
    assert_param(keyboard != NULL);
    Button_Init(&keyboard->key_W,1,Get_Key_Status,(void*)RC_Key_W);
    Button_Init(&keyboard->key_S,1,Get_Key_Status,(void*)RC_Key_S);
    Button_Init(&keyboard->key_A,1,Get_Key_Status,(void*)RC_Key_A);
    Button_Init(&keyboard->key_D,1,Get_Key_Status,(void*)RC_Key_D);
    Button_Init(&keyboard->key_Shift,1,Get_Key_Status,(void*)RC_Key_Shift);
    Button_Init(&keyboard->key_Ctrl,1,Get_Key_Status,(void*)RC_Key_Ctrl);
    Button_Init(&keyboard->key_Q,1,Get_Key_Status,(void*)RC_Key_Q);
    Button_Init(&keyboard->key_E,1,Get_Key_Status,(void*)RC_Key_E);
    Button_Init(&keyboard->key_R,1,Get_Key_Status,(void*)RC_Key_R);
    Button_Init(&keyboard->key_F,1,Get_Key_Status,(void*)RC_Key_F);
    Button_Init(&keyboard->key_G,1,Get_Key_Status,(void*)RC_Key_G);
    Button_Init(&keyboard->key_Z,1,Get_Key_Status,(void*)RC_Key_Z);
    Button_Init(&keyboard->key_X,1,Get_Key_Status,(void*)RC_Key_X);
    Button_Init(&keyboard->key_C,1,Get_Key_Status,(void*)RC_Key_C);
    Button_Init(&keyboard->key_V,1,Get_Key_Status,(void*)RC_Key_V);
    Button_Init(&keyboard->key_B,1,Get_Key_Status,(void*)RC_Key_B);  

    Button_Init(&keyboard->mouse_press_left,1,Get_Mouse_Left_Status,(void*)RC_Mouse_Left);
    Button_Init(&keyboard->mouse_press_right,1,Get_Mouse_Right_Status,(void*)RC_Mouse_Right);
}

/**
 * @brief 键盘事件回调注册
 * @param keyboard 键盘按键句柄
 */
void Key_Register(Keyboard_t*keyboard)
{
    //按键按下，速度正常累加
    Button_Register(&keyboard->key_W,Press_Hold,Front_Press_Hold);
    Button_Register(&keyboard->key_S,Press_Hold,Back_Press_Hold);
    Button_Register(&keyboard->key_A,Press_Hold,Left_Press_Hold);
    Button_Register(&keyboard->key_D,Press_Hold,Right_Press_Hold);
    //按键释放，减速
    Button_Register(&keyboard->key_W,Press_None,Front_Press_Release);
    Button_Register(&keyboard->key_S,Press_None,Back_Press_Release);
    Button_Register(&keyboard->key_A,Press_None,Left_Press_Release);
    Button_Register(&keyboard->key_D,Press_None,Right_Press_Release);
    //鼠标左键打弹
    //Button_Register(&keyboard->mouse_press_left,Press_Hold,Gimbal_Shoot_Fire);
    //鼠标右键自瞄

    //shift按下切换为小陀螺或底盘跟随
    Button_Register(&keyboard->key_Shift,Single_Clink,Mode_Switch);
}

/**
 * @brief 按键按下，速度正常累加
 */
void Front_Press_Hold(Button_t *button)
{
    Absolute_Speed.vx = Ramp_float(Key_Max, Absolute_Speed.vx,FBA);	
}
void Back_Press_Hold(Button_t *button)
{
    Absolute_Speed.vx = Ramp_float(-Key_Max, Absolute_Speed.vx,FBA);	
}
void Left_Press_Hold(Button_t *button)
{
    Absolute_Speed.vy = Ramp_float(Key_Max, Absolute_Speed.vy,LRA);	
}
void Right_Press_Hold(Button_t *button)
{
    Absolute_Speed.vy = Ramp_float(-Key_Max, Absolute_Speed.vy,LRA);	
}

/**
 * @brief 按键释放，按斜坡函数减速
 */
void Front_Press_Release(Button_t *button)
{
    if (Absolute_Speed.vx > 0)    		
     Absolute_Speed.vx = Ramp_float(0, Absolute_Speed.vx,0.05*Key_Max);
}
void Back_Press_Release(Button_t *button)
{
    if (Absolute_Speed.vx < 0)    		
     Absolute_Speed.vx = Ramp_float(0, Absolute_Speed.vx,0.05*Key_Max);
}
void Left_Press_Release(Button_t *button)
{
    if(Absolute_Speed.vy > 0)
     Absolute_Speed.vy = Ramp_float(0, Absolute_Speed.vy,0.05*Key_Max);
}
void Right_Press_Release(Button_t *button)
{
    if(Absolute_Speed.vy < 0)
     Absolute_Speed.vy = Ramp_float(0, Absolute_Speed.vy,0.05*Key_Max);
}

/**
 * @brief 获取裁判系统功率上限
 * @return temp_power 功率上限
 */
uint16_t JUDGE_usGetChassis(void)
{
	uint16_t static temp_power;
	temp_power= chassis.omni_wheel_chassis_power_limit.reference->game_robot_status.chassis_power_limit;
	return temp_power;
}

/**
 * @brief 超电模式选择
 */
void Chassis_Mode_Choose(void)
{
	if (chassis.omni_wheel_chassis_power_limit.status.enable_supercap == DISABLE)
	{
		chassis.omni_wheel_chassis_power_limit.status.enable_supercap = ENABLE;
	}
    else if(chassis.omni_wheel_chassis_power_limit.status.enable_supercap == ENABLE)
	{		
        chassis.omni_wheel_chassis_power_limit.status.enable_supercap = DISABLE;
	}
    //判断超级电容状态，调整加速度以及最大速度
	switch (chassis.omni_wheel_chassis_power_limit.status.enable_supercap)
    {
    	case DISABLE:
			FBA = Add_DISABLE_SuperCup_Speed; //加速度
			LRA = Add_DISABLE_SuperCup_Speed; //加速度
    		break;
		
    	case ENABLE:
			FBA = Add_ENABLE_SuperCup_Speed; //加速度
			LRA = Add_ENABLE_SuperCup_Speed; //加速度
    		break;
    	default:
    		break;
    }
    //依据功率上限选择最大速度
    //Get_Chassis_Pow_Limit = JUDGE_usGetChassis();
    Get_Chassis_Pow_Limit = 60;
	switch(Get_Chassis_Pow_Limit)
	{
		case 60:
			Key_Max = KEY_CHASSIS_MOVE_MAX_60W;
			break;
		case 80:
			Key_Max = KEY_CHASSIS_MOVE_MAX_80W;
			break;
		case 100:
			Key_Max = KEY_CHASSIS_MOVE_MAX_100W;
			break;
		default:
			Key_Max = KEY_CHASSIS_MOVE_MAX_60W;
	}
}
/**
 * @brief 小陀螺与底盘跟随模式切换
 */
void Mode_Switch(Button_t *button)
{
    if(chassis.mode.rotate == Yaw_Follow_Mode)
        chassis.mode.rotate = Rotate_Made;
    else if(chassis.mode.rotate == Rotate_Made)
        chassis.mode.rotate = Yaw_Follow_Mode;
}

/**
 * @brief 底盘键盘模式
 */
void Chassis_KEY_Ctrl(void)
{
    float temp;
    temp = 6.15f;
    Motor_t *motor;
	Robo_Push_message_Cmd("Set_Chassis_Yaw_Angle", temp);
    Robo_Get_message_Cmd("Gimbal_Yaw_Motor", motor);
    temp = Get_Motor_Position_Data(motor);
	Robo_Push_message_Cmd("Real_Chassis_Yaw_Angle", temp);

    switch(Chassis_Mode) 
    {
		case CHASSIS_FOLLOW_GIMBAL://跟随云台
            // chassic_mode = Yaw_Follow;
            chassis.mode.rotate = Yaw_Follow_Mode;
			break;
		case CHASSIS_SPIN:		//小陀螺模式
            // chassic_mode = Rotate;
            chassis.mode.rotate = Rotate_Made;
			switch ( Get_Chassis_Pow_Limit )
            {
            	case 60:
					chassis.expect_speed.vw =- Round_SPEED_1 * Round_S;					
            		break;
            	case 80:
					chassis.expect_speed.vw =- Round_SPEED_2 * Round_S;
            		break;
				case 100: 
					chassis.expect_speed.vw =- Round_SPEED_3 * Round_S;
            		break;
            	default:
					chassis.expect_speed.vw =- Round_SPEED * Round_S;		
            		break;
            }
			break;
		default:
			break;
    }
    Robo_Push_message_Cmd("Set_Move_X_Speed", Absolute_Speed.vx);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", Absolute_Speed.vy);
}

/**
 * @brief 云台键盘模式
 */
void GIMBAL_KEY_Ctrl(void)
{   
    float temp;
    Robo_Get_message_Cmd("Set_Gimbal_Pitch_Angle", gimbal.pitch.expect_angle);
    temp = gimbal.pitch.expect_angle;
/*     //使用电机刻度限位
	if( (gimbal.pitch.motor->position < Pitch_Limit_High_Angle) && (RC_Ctrl.mouse.y<0) )	
        RC_Ctrl.mouse.y=0;
	else if( (gimbal.pitch.motor->position > Pitch_Limit_Low_Angle) && (RC_Ctrl.mouse.y>0))	
        RC_Ctrl.mouse.y=0;
	else  */
	    temp += (Mouse_Sensitivit * RC_Ctrl.mouse.y);
    temp = Lowpass(temp, gimbal.pitch.expect_angle, 0.2f);
    //Ramp_float(, Absolute_Speed.vy,0.02f);
    Robo_Push_message_Cmd("Set_Gimbal_Pitch_Angle", temp);
    //
    Robo_Get_message_Cmd("Set_Gimbal_Yaw_Angle", gimbal.yaw.expect_angle);
    temp = gimbal.yaw.expect_angle;
	temp -= (Mouse_Sensitivit * RC_Ctrl.mouse.x);
    temp = Lowpass(temp, gimbal.yaw.expect_angle, 0.1f);
    Robo_Push_message_Cmd("Set_Gimbal_Yaw_Angle", temp);			
}

/**
 * @brief 拨弹盘开启供弹，开始打弹
 */
void Gimbal_Shoot_Fire(Button_t *button)
{
    Shoot_Open_Fire(0.8);
}
