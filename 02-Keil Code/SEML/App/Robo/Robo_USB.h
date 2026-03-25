/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      no action.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
	*  V2.0.0			Jan-4-2024			HansonZhang			2. PC communication and  auto aiming capability
	*	 V2.0.1			Jan-5-2024			HansonZhang			3. bug fixed: PC communication failed when auto aiming enabled
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H

#include "stdint.h"
#include "reference.h"
#include "DR16_Remote.h"
#include "gpio.h"
#include "SEML_common.h"
#include "Robo_Common.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "message.h"
#include "usart.h"

// Receive
#define NAVIGATION_HEADER 0x11
#define RC_HEADER 0x12

// Send
#define DECISION_HEADER 0x22

typedef struct __attribute__((packed))
{
  uint8_t header;

  float linear_vx;
  float linear_vy;
  float linear_vz;
  float angular_x;
  float angular_y;
  float angular_z;

  uint16_t checksum;
} Received_Navigation_t;

typedef struct __attribute__((packed))
{
  uint8_t header;

  float linear_vx;
  float linear_vy;
  float linear_vw;

  uint8_t mode;
  uint16_t checksum;
} Received_RC_t;

typedef struct __attribute__((packed))
{
  uint8_t header;

  uint16_t red_1_robot_hp;
  uint16_t red_2_robot_hp;
  uint16_t red_3_robot_hp;
  uint16_t red_4_robot_hp;
  uint16_t red_5_robot_hp;
  uint16_t red_7_robot_hp;
  uint16_t red_outpost_hp;
  uint16_t red_base_hp;
  uint16_t blue_1_robot_hp;
  uint16_t blue_2_robot_hp;
  uint16_t blue_3_robot_hp;
  uint16_t blue_4_robot_hp;
  uint16_t blue_5_robot_hp;
  uint16_t blue_7_robot_hp;
  uint16_t blue_outpost_hp;
  uint16_t blue_base_hp;

  uint8_t game_progress;
  uint16_t stage_remain_time;
  uint8_t robot_id;
  uint16_t current_hp;
  uint16_t shooter_heat;

  _Bool team_color;
  _Bool is_attacked;

  uint16_t chassis_power_limit;
  uint16_t shooter_barrel_heat_limit;     // 枪口热量限制
  uint16_t shooter_17mm_1_barrel_heat;    // 17mm枪口热量
  float chassis_power;					          // 底盘输出功率
  uint16_t chassis_power_buffer;		    	// 剩余缓冲能量

  // Robo Position
  float x;
  float y;
  float angle; //测速模块的朝向

  uint32_t event_data;
  // bit 2：己方补给区的占领状态，1为已占领
  // bit30-31: ：中心增益点的占领情况 0 为未被占领，1 为被已方占领，2 为被对方占领，3 为被双方占领。

  uint16_t checksum;
} Send_Referee_t;

extern void USB_Init();
extern void USB_Task(void *config);

#endif
