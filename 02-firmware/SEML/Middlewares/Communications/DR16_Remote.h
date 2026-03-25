#ifndef _REMOTE_H_
#define _REMOTE_H_
#include "SEML_common.h"
#include "timer.h"
#include "uart_if.h"

#define DR16_GetTick() SEML_GetTick()
#define DR16_TIMEOUT 20
#define DR16_rx_buffer RC_Ctrl.buffer

#define DR16_RC_OFFSET 1024     
#define DR16_RC_OFFSET_MAX 660  // DR16单通道最大偏置值
#define DR16_DEAD_ZONE 5
/**
 * @brief 遥控器拨杆位置
 */
typedef enum
{
	RC_S_UP = 1,
	RC_S_MID = 3,
	RC_S_DOWM = 2
} RC_S_t;

/**
 * @brief DR16遥控器句柄
 */
typedef struct
{
	struct
	{
		int16_t ch0; /**< 右摇杆左右方向 */
		int16_t ch1; /**< 右摇杆上下方向 */
		int16_t ch2; /**< 左摇杆左右方向 */
		int16_t ch3; /**< 左摇杆上下方向 */
		RC_S_t s1;	 /**< 左拨杆 */
		RC_S_t s2;	 /**< 右拨杆 */
		int16_t sw;	 /**< 侧滚轮 */
	} rc;					 /**< 遥控器 */
	struct
	{
		int16_t x;			 /**< 鼠标移动x轴 */
		int16_t y;			 /**< 鼠标移动y轴 */
		int16_t z;			 /**< 鼠标中键滚动 */
		uint8_t press_l; /**< 鼠标左击 */
		uint8_t press_r; /**< 鼠标右击 */
	} mouse;					 /**< 鼠标 */
	struct key_value_t
	{
		uint16_t key_W : 1;
		uint16_t key_S : 1;
		uint16_t key_A : 1;
		uint16_t key_D : 1;
		uint16_t key_Shift : 1;
		uint16_t key_Ctrl : 1;
		uint16_t key_Q : 1;
		uint16_t key_E : 1;
		uint16_t key_R : 1;
		uint16_t key_F : 1;
		uint16_t key_G : 1;
		uint16_t key_Z : 1;
		uint16_t key_X : 1;
		uint16_t key_C : 1;
		uint16_t key_V : 1;
		uint16_t key_B : 1;
	} keyboard;					/**< 键盘 */
	uint8_t buffer[18]; /**< 接收缓冲句柄 */
	uint32_t rx_tick;		/**< 接收时间戳 */
	UART_Handle_t uart;
	SEML_StatusTypeDef status;
#ifdef USE_SOFT_WATCHDOG
	Soft_WatchDog_t watchdog; /**< 软件看门狗*/
#endif
} RC_Ctrl_t;

extern RC_Ctrl_t RC_Ctrl;

/**
 * @brief DR16遥控器初始化
 * @param huart 串口句柄
*/
void DR16_Init(RC_Ctrl_t *handle, UART_IF_t *huart);

/**
 * @brief DR16接收回调函数
 * @param config 接收句柄
 */
void DR16_Rx_Callback(UART_Handle_t *config);

/**
 * @brief DR16失联复位回调函数
 * @param config 接收句柄
 */
void DR16_Rx_Timeout_Reset_Callack(void *config);

/**
 * @brief 获取DR16接收机是否失联
 * @return 1:失联 0:未失联
 */
#define DR16_Get_Rx_Timeout() (DR16_GetTick() - RC_Ctrl.rx_tick >= DR16_TIMEOUT)

/**
 * @brief DR16接收数据复位
 */
void DR16_Reset(RC_Ctrl_t *handle);

#endif
