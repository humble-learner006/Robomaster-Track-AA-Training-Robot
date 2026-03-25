#include "chassis.h"
#include "math_common.h"

void Euclidean_To_Polar(float x, float y, float *r, float *theta)
{
	*r = sqrt(x * x + y * y);
	*theta = atan2(y, x);
}

/**
 * @brief 底盘逆向运动学解算
 * @note 根据运动状态求解出每个轮子的子状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] car_velocity 车辆运动状态
 * @param[out] wheel_rpm 轮子转速(deg/s)
 */
inline void Chassis_Inverse_Kinematics(Chassis_Kinematic_t *chassis, Chassis_Velocity_t *car_velocity, float wheel_rpm[4])
{
	chassis->inverse_kinematics_fun(chassis, car_velocity, wheel_rpm);
}

/**
 * @brief 底盘正向运动学解算
 * @note 根据每个轮子的子状态求解出运动状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] wheel_rpm 轮子转速(deg/s)
 * @param[out] car_velocity 车辆运动状态
 */
inline void Chassis_Forward_Kinematics(Chassis_Kinematic_t *chassis, float wheel_rpm[4], Chassis_Velocity_t *car_velocity)
{
	chassis->forward_kinematics_fun(chassis, wheel_rpm, car_velocity);
}

/**
 * @brief 麦克纳姆轮运动学模型初始化
 * @param chassis 底盘运动学句柄
 * @param mechanical_param 底盘机械参数
 *   需初始化轮子半径(wheel_radius),轴距(wheel_base),轮距(wheel_track),旋转方向(rotation_direction),电机减速比(wheel_motor_reduction_ratio)
 */
void Mecanum_Wheel_Kinematics_Init(Chassis_Kinematic_t *chassis, Chassis_Mechanical_Param_t mechanical_param)
{
	assert_param(chassis != NULL);
	assert_param(mechanical_param.rotation_direction == 1 || mechanical_param.rotation_direction == -1);
	assert_param(mechanical_param.wheel_radius > 0);
	assert_param(mechanical_param.wheel_base > 0);
	assert_param(mechanical_param.wheel_base > 0);
	assert_param(mechanical_param.wheel_motor_reduction_ratio > 0);
	// 底盘运动学参数计算
	// 计算轮子周长
	float wheel_perimeter = 2 * PI * mechanical_param.wheel_radius / 1000.0f;
	// 计算转动系数
	chassis->export_ratio = (mechanical_param.wheel_base + mechanical_param.wheel_track) / 2000.0f * mechanical_param.rotation_direction;
	// 计算输出系数
	chassis->wheel_rpm_ratio = 2 * PI * mechanical_param.wheel_motor_reduction_ratio / wheel_perimeter;
	// 回调函数注册
	chassis->inverse_kinematics_fun = (Inverse_Kinematics_Fun_t)Mecanum_Wheel_Inverse_Kinematics;
	chassis->forward_kinematics_fun = (Forward_Kinematics_Fun_t)Mecanum_Wheel_Forward_Kinematics;
}

/**
 * @brief 麦克纳姆轮逆向运动学解算
 * @note 根据运动状态求解出每个轮子的子状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] car_velocity 车辆运动状态
 * @param[out] wheel_rpm 轮子转速(deg/s)
 */
void Mecanum_Wheel_Inverse_Kinematics(Chassis_Kinematic_t *chassis, Chassis_Velocity_t *car_velocity, float wheel_rpm[4])
{
	float vx, vy, vw;
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_rpm != NULL);
	vx = car_velocity->vx;
	vy = car_velocity->vy;
	vw = car_velocity->vw;
	// 底盘逆向运动学模型解算
	wheel_rpm[CHASSIS_FL] = (+vx - vy + vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_FR] = (-vx - vy + vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_BL] = (+vx + vy + vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_BR] = (-vx + vy + vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
}

/**
 * @brief 麦克纳姆轮正向运动学解算
 * @note 根据每个轮子的子状态求解出运动状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] wheel_rpm 轮子转速(deg/s)
 * @param[out] car_velocity 车辆运动状态
 */
void Mecanum_Wheel_Forward_Kinematics(Chassis_Kinematic_t *chassis, float wheel_rpm[4], Chassis_Velocity_t *car_velocity)
{
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_rpm != NULL);
	car_velocity->vx = (wheel_rpm[CHASSIS_BR] + wheel_rpm[CHASSIS_BL]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vy = (wheel_rpm[CHASSIS_BL] - wheel_rpm[CHASSIS_FL]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vw = (wheel_rpm[CHASSIS_FR] - wheel_rpm[CHASSIS_BL]) / (4 * chassis->export_ratio * chassis->wheel_rpm_ratio);
}

/**
 * @brief 全向轮运动学模型初始化
 * @param chassis 底盘运动学句柄
 * @param mechanical_param 底盘机械参数
 *   需初始化轮子半径(wheel_radius),轴距(wheel_base),旋转方向(rotation_direction),电机减速比(wheel_motor_reduction_ratio)
 */
void Four_Omni_Wheel_Kinematics_Init(Chassis_Kinematic_t *chassis, Chassis_Mechanical_Param_t mechanical_param)
{
	assert_param(chassis != NULL);
	assert_param(mechanical_param.rotation_direction == 1 || mechanical_param.rotation_direction == -1);
	assert_param(mechanical_param.wheel_radius > 0);
	assert_param(mechanical_param.wheel_base > 0);
	assert_param(mechanical_param.wheel_motor_reduction_ratio > 0);
	// 底盘运动学参数计算
	// 计算轮子周长
	float wheel_perimeter = 2 * PI * mechanical_param.wheel_radius / 1000.0f;
	// 计算转动系数
	chassis->export_ratio = mechanical_param.wheel_base / 2000.0f * mechanical_param.rotation_direction;
	// 计算输出系数
	chassis->wheel_rpm_ratio = 2 * PI * mechanical_param.wheel_motor_reduction_ratio / wheel_perimeter;
	// 回调函数注册
	chassis->inverse_kinematics_fun = (Inverse_Kinematics_Fun_t)Four_Omni_Wheel_Inverse_Kinematics;
	chassis->forward_kinematics_fun = (Forward_Kinematics_Fun_t)Four_Omni_Wheel_Forward_Kinematics;
}

/**
 * @brief 全向轮逆向运动学解算
 * @note 根据运动状态求解出每个轮子的子状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] car_velocity 车辆运动状态
 * @param[out] wheel_rpm 轮子转速(rad/s)
 */
void Four_Omni_Wheel_Inverse_Kinematics(Chassis_Kinematic_t *chassis, Chassis_Velocity_t *car_velocity, float wheel_rpm[4])
{
	float vx, vy, vw;
	const float INV_SQRT_2 = 0.414213562f;
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_rpm != NULL);
	vx = car_velocity->vx;
	vy = car_velocity->vy;
	vw = car_velocity->vw;
	// 底盘逆向运动学模型解算
	wheel_rpm[CHASSIS_FL] = ((+vx - vy) * INV_SQRT_2 - vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_FR] = ((-vx - vy) * INV_SQRT_2 - vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_BL] = ((+vx + vy) * INV_SQRT_2 - vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
	wheel_rpm[CHASSIS_BR] = ((-vx + vy) * INV_SQRT_2 - vw * chassis->export_ratio) * chassis->wheel_rpm_ratio;
}

/**
 * @brief 全向轮正向运动学解算
 * @note 根据每个轮子的子状态求解出运动状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] wheel_rpm 轮子转速(deg/s)
 * @param[out] car_velocity 车辆运动状态
 */
void Four_Omni_Wheel_Forward_Kinematics(Chassis_Kinematic_t *chassis, float wheel_rpm[4], Chassis_Velocity_t *car_velocity)
{
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_rpm != NULL);
	car_velocity->vx = (wheel_rpm[CHASSIS_FL] + wheel_rpm[CHASSIS_BR]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vy = (wheel_rpm[CHASSIS_BL] + wheel_rpm[CHASSIS_FR]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vw = (wheel_rpm[CHASSIS_FL] + wheel_rpm[CHASSIS_BR] + wheel_rpm[CHASSIS_BL] + wheel_rpm[CHASSIS_FR]) / (4 * chassis->export_ratio * chassis->wheel_rpm_ratio);
}


/**
 * @brief 舵轮运动学模型初始化
 * @param chassis 底盘运动学句柄
 * @param mechanical_param 底盘机械参数
 *   需初始化轮子半径(wheel_radius),轴距(wheel_base),旋转方向(rotation_direction),电机减速比(wheel_motor_reduction_ratio)
 */
void Steer_Wheel_Kinematics_Init(Chassis_Kinematic_t *chassis, Chassis_Mechanical_Param_t mechanical_param)
{
	assert_param(chassis != NULL);
	assert_param(mechanical_param.rotation_direction == 1 || mechanical_param.rotation_direction == -1);
	assert_param(mechanical_param.wheel_radius > 0);
	assert_param(mechanical_param.wheel_base > 0);
	assert_param(mechanical_param.wheel_motor_reduction_ratio > 0);
	// 底盘运动学参数计算
	// 计算轮子周长
	float wheel_perimeter = 2 * PI * mechanical_param.wheel_radius / 1000.0f;
	// 计算转动系数
	chassis->export_ratio = mechanical_param.wheel_base / 2000.0f * mechanical_param.rotation_direction;
	// 计算输出系数
	chassis->wheel_rpm_ratio = 2 * PI * mechanical_param.wheel_motor_reduction_ratio / wheel_perimeter;
	// 回调函数注册
	chassis->inverse_kinematics_fun = (Inverse_Kinematics_Fun_t)Steer_Wheel_Inverse_Kinematics;
	chassis->forward_kinematics_fun = (Forward_Kinematics_Fun_t)Steer_Wheel_Forward_Kinematics;
}

/**
 * @brief 舵轮逆向运动学解算
 * @note 根据运动状态求解出每个轮子的子状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] car_velocity 车辆运动状态
 * @param[out] wheel_rpm 轮子转速(rad/s)
 */
void Steer_Wheel_Inverse_Kinematics(Chassis_Kinematic_t *chassis, Chassis_Velocity_t *car_velocity, float wheel_drive_rpm[4], float wheel_steer_rad[4])
{
	float vx, vy, vw;
	float partial_vx[4];
	float partial_vy[4];
	//const float INV_SQRT_2 = 0.707106781186548f;//0.414213562f;//
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_drive_rpm != NULL);
	assert_param(wheel_steer_rad != NULL);
	vx = car_velocity->vx;
	vy = car_velocity->vy;
#define R 0.24f
	vw = car_velocity->vw * R;
	// 底盘逆向运动学模型解算
#define INV_SQRT_2 0.707106781186548f
#define RM6020_FL_ANGLE_OFFSET 0.889f
#define RM6020_FR_ANGLE_OFFSET -0.628f - PI
#define RM6020_BL_ANGLE_OFFSET 0.4092926f
#define RM6020_BR_ANGLE_OFFSET 0.9076f
#define ALL_OFFSET 0.332f
	partial_vx[CHASSIS_FL] = vx + vw * INV_SQRT_2;
	partial_vx[CHASSIS_FR] = vx + vw * INV_SQRT_2;
	partial_vx[CHASSIS_BL] = vx - vw * INV_SQRT_2;
	partial_vx[CHASSIS_BR] = vx - vw * INV_SQRT_2;
	
	partial_vy[CHASSIS_FL] = vy + vw * INV_SQRT_2;
	partial_vy[CHASSIS_FR] = vy - vw * INV_SQRT_2;
	partial_vy[CHASSIS_BL] = vy + vw * INV_SQRT_2;
	partial_vy[CHASSIS_BR] = vy - vw * INV_SQRT_2;

	for(int i = 0; i < 4; i++)
	{
		Euclidean_To_Polar(partial_vx[i], partial_vy[i], &wheel_drive_rpm[i], &wheel_steer_rad[i]);
		wheel_drive_rpm[i] *= 500;
	}
	wheel_steer_rad[CHASSIS_FL] += RM6020_FL_ANGLE_OFFSET + ALL_OFFSET;
	wheel_steer_rad[CHASSIS_FR] += RM6020_FR_ANGLE_OFFSET + ALL_OFFSET;
	wheel_steer_rad[CHASSIS_BL] += RM6020_BL_ANGLE_OFFSET + ALL_OFFSET;
	wheel_steer_rad[CHASSIS_BR] += RM6020_BR_ANGLE_OFFSET + ALL_OFFSET;
}

/**
 * @brief 舵轮正向运动学解算
 * @note 根据每个轮子的子状态求解出运动状态
 * @param[in] chassis 底盘运动学句柄
 * @param[in] wheel_rpm 轮子转速(deg/s)
 * @param[out] car_velocity 车辆运动状态
 */
void Steer_Wheel_Forward_Kinematics(Chassis_Kinematic_t *chassis, float wheel_rpm[4], Chassis_Velocity_t *car_velocity)
{
	assert_param(chassis != NULL);
	assert_param(car_velocity != NULL);
	assert_param(wheel_rpm != NULL);
	car_velocity->vx = (wheel_rpm[CHASSIS_FL] + wheel_rpm[CHASSIS_BR]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vy = (wheel_rpm[CHASSIS_BL] + wheel_rpm[CHASSIS_FR]) / (2 * chassis->wheel_rpm_ratio);
	car_velocity->vw = (wheel_rpm[CHASSIS_FL] + wheel_rpm[CHASSIS_BR] + wheel_rpm[CHASSIS_BL] + wheel_rpm[CHASSIS_FR]) / (4 * chassis->export_ratio * chassis->wheel_rpm_ratio);
}
