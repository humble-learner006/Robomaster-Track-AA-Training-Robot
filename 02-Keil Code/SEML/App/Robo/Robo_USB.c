/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *	 V2.0.0			Dec-27-2023			HansonZhang			2. added realtime PID tuning capability
  *  V3.0.0			Jan-4-2024			HansonZhang			3. PC communication and  auto aiming capability
  *	 V3.0.1			Jan-5-2024			HansonZhang			4. bug fixed: PC communication failed when auto aiming enabled
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "Robo_USB.h"

//Prototypes of functions
static void usb_printf(const char *fmt,...);
static void USB_send(Send_Referee_t *packet);
static void USB_receive(void);
static void demo_show_receive(void);
static void auto_control(void);
static void remote_control(void);
static void pause_control(void);
static void send_decision(void);
static void update_referee(void);
void Remote_Control(void);
//End of prototypes

uint8_t usb_buf[256];
uint32_t usb_buf_pos=0;
uint32_t count=0;

static const char status[2][7] = {"OK", "ERROR!"};
static uint8_t usb_send[256];
int8_t usbOK = 1;

Reference_t referee;
Send_Referee_t send_referee;
Received_Navigation_t received_navigation;
Received_RC_t received_rc;

float temp;

void USB_Init()
{
    MX_USB_DEVICE_Init();
	Reference_Init(&referee,&huart6);
	send_referee.header = DECISION_HEADER;
}

void USB_Task(void *config)
{
	update_referee();
	USB_receive();
	send_decision();
	Remote_Control();
}

static void USB_send(Send_Referee_t *packet)
{
	memcpy(usb_send, packet, sizeof(Send_Referee_t)-2);
  	packet->checksum = get_CRC16_check_sum(usb_send, sizeof(Send_Referee_t)-2, 0xffff);
	memcpy(usb_send, packet, sizeof(Send_Referee_t));
	CDC_Transmit_FS(usb_send, sizeof(Send_Referee_t));
}

static void USB_receive(void)
{
	uint32_t len = 384;
	usbOK = CDC_Receive_FS(usb_buf, &len); // Read data into the buffer
	switch (usb_buf[0])
	{
		case NAVIGATION_HEADER:
			if (received_rc.mode == 2)
			{
				memcpy(&received_navigation, usb_buf, sizeof(Received_Navigation_t));
				auto_control();
			}
			break;
		case RC_HEADER:
			memcpy(&received_rc, usb_buf, sizeof(Received_RC_t));
			remote_control();
			break;
		default:
			pause_control();
			break;
	}
}

static void auto_control(void)
{
	Robo_Push_message_Cmd("Set_Move_X_Speed", received_navigation.linear_vx);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", received_navigation.linear_vy);
	Robo_Push_message_Cmd("Set_Rotate_Speed", received_navigation.angular_x);	
}

static void remote_control(void)
{
	switch (received_rc.mode)
		{	case 0:
				pause_control();
				break;
		case 1:
				//Robo_Push_message_Cmd("Set_Move_X_Speed", received_rc.linear_vx);
				//Robo_Push_message_Cmd("Set_Move_Y_Speed", received_rc.linear_vy);
				//Robo_Push_message_Cmd("Set_Rotate_Speed", received_rc.linear_vw);
				break;
		//case 2:
				//auto_control();
				//float rotate_speed = 0.5f;
				//Robo_Push_message_Cmd("Set_Rotate_Speed", rotate_speed);
		//default:
				//pause_control();
				//break;
		break;
		
	}
}

static void pause_control(void)
{
	float temp = 0;
	Robo_Push_message_Cmd("Set_Move_X_Speed", temp);
	Robo_Push_message_Cmd("Set_Move_Y_Speed", temp);
	Robo_Push_message_Cmd("Set_Rotate_Speed", temp);
}

static void send_decision(void)
{	
  	send_referee.blue_1_robot_hp = referee.game_robot_HP.blue_1_robot_HP;
  	send_referee.blue_2_robot_hp = referee.game_robot_HP.blue_2_robot_HP;
	send_referee.blue_3_robot_hp = referee.game_robot_HP.blue_3_robot_HP;
	send_referee.blue_4_robot_hp = referee.game_robot_HP.blue_4_robot_HP;
	send_referee.blue_5_robot_hp = referee.game_robot_HP.blue_5_robot_HP;
	send_referee.blue_7_robot_hp = referee.game_robot_HP.blue_7_robot_HP;
	send_referee.blue_outpost_hp = referee.game_robot_HP.blue_outpost_HP;
	send_referee.blue_base_hp = referee.game_robot_HP.blue_base_HP;
	send_referee.red_1_robot_hp = referee.game_robot_HP.red_1_robot_HP;
	send_referee.red_2_robot_hp = referee.game_robot_HP.red_2_robot_HP;
	send_referee.red_3_robot_hp = referee.game_robot_HP.red_3_robot_HP;
	send_referee.red_4_robot_hp = referee.game_robot_HP.red_4_robot_HP;
	send_referee.red_5_robot_hp = referee.game_robot_HP.red_5_robot_HP;
	send_referee.red_7_robot_hp = referee.game_robot_HP.red_7_robot_HP;
	send_referee.red_outpost_hp = referee.game_robot_HP.red_outpost_HP;
	send_referee.red_base_hp = referee.game_robot_HP.red_base_HP;

	send_referee.is_attacked = referee.game_robot_status.current_HP < send_referee.current_hp;	// Is being attacked if this HP is less than last iteration
	send_referee.game_progress = referee.game_status.game_progress;
	send_referee.stage_remain_time = referee.game_status.stage_remain_time;
	send_referee.robot_id = referee.game_robot_status.robot_id;
	send_referee.current_hp = referee.game_robot_status.current_HP;	// Update HP
	send_referee.shooter_heat = referee.game_robot_status.shooter_barrel_cooling_value;
	send_referee.team_color = referee.game_robot_status.robot_id > 100;

	send_referee.chassis_power_limit = referee.game_robot_status.chassis_power_limit;
	send_referee.shooter_barrel_heat_limit = referee.game_robot_status.shooter_barrel_heat_limit;
	send_referee.shooter_17mm_1_barrel_heat = referee.power_heat_data.shooter_id1_17mm_cooling_heat;
	send_referee.chassis_power = referee.power_heat_data.chassis_power;
	send_referee.chassis_power_buffer = referee.power_heat_data.chassis_power_buffer;

	send_referee.x = referee.game_robot_pos.x;
	send_referee.y = referee.game_robot_pos.y;
	send_referee.angle = referee.game_robot_pos.angle;
	
	send_referee.event_data = referee.event_data.event_data;

	USB_send(&send_referee);
}

static void update_referee(void)
{
	Robo_Push_message_Cmd("Power_Limit", referee.game_robot_status.chassis_power_limit);
	Robo_Push_message_Cmd("Current_Power", referee.power_heat_data.chassis_power);
}

__weak void Remote_Control(void)
{
	float temp, temp1, temp2;
#define RC_MOVE_MAX 0.1f
#define RC_MOVE_SENSITIVITY (RC_MOVE_MAX / 660.0f)
#define RC_ADD_SENSITIVITY (1.0f / 660.0f)
	// 移动
	temp = (RC_Ctrl.rc.ch1 - DR16_RC_OFFSET) * RC_MOVE_SENSITIVITY;
	temp1 = -(RC_Ctrl.rc.ch0 - DR16_RC_OFFSET) * RC_MOVE_SENSITIVITY;
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

	// 旋转
	if(RC_Ctrl.rc.s1 == 0)
	{
		temp = (RC_Ctrl.rc.ch2 - DR16_RC_OFFSET) * RC_MOVE_SENSITIVITY;
		Robo_Push_message_Cmd("Set_Rotate_Speed", temp);
	}
	if(RC_Ctrl.rc.s1 == 1)
	{
		temp = 0.5f * PI;
		Robo_Push_message_Cmd("Set_Rotate_Speed", temp);
	}
}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}