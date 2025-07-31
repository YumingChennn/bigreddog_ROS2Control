// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef Tangair_usb2can_motor_imu_H
#define Tangair_usb2can_motor_imu_H

#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <cmath>
#include <iostream>

#include <sys/mman.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <sched.h>

#include <assert.h>
#include <chrono>
#include <stdio.h>
#include <signal.h>
#include "usb_can.h"
#include "config_loader.h"

#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>


#include <Eigen/Dense>

using Matrix3x4d = Eigen::Matrix<double, 3, 4>;

// 辅助函数
Matrix3x4d mujoco_ang2real_ang(const std::vector<double>& dof_pos);
std::vector<double> real_ang2mujoco_ang(const std::vector<double>& dof_pos);

void PrintMatrix(const std::string& name, const Eigen::Matrix<double, 3, 4>& matrix);

float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);


//达妙其他电机参数    {P, V, T}
				// {12.5, 30, 10 }, // DM4310
				// {12.5, 50, 10 }, // DM4310_48V
				// {12.5, 8, 28 },  // DM4340
				// {12.5, 10, 28 }, // DM4340_48V
				// {12.5, 45, 20 }, // DM6006
				// {12.5, 45, 40 }, // DM8006
				// {12.5, 45, 54 }, // DM8009
				// {12.5,25,  200}, // DM10010L
				// {12.5,20, 200},  // DM10010
				// {12.5,280,1},    // DMH3510
				// {12.5,45,10},    // DMH6215
				// {12.5,45,10}     // DMG6220

// 达妙电机,此处为DM8006参数
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN_8006 -40.0f
#define T_MAX_8006 40.0f
#define T_MIN_8009 -54.0f
#define T_MAX_8009 54.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

#define PI (3.1415926f)

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

typedef struct
{
	uint16_t id;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	//位置限制
	float Tau_Min;
	float Tau_Max;

} Motor_CAN_Send_Struct;

typedef struct
{
	uint16_t ERR;

	uint16_t current_position; //
	uint16_t current_speed;	   //
	uint16_t current_torque;   //
	uint16_t current_temp_MOS;	   //
	uint16_t current_temp_Rotor;	   //

	float current_position_f;//
	float current_speed_f;//
	float current_torque_f;//

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send, ID_5_motor_send, ID_6_motor_send, ID_7_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve, ID_5_motor_recieve, ID_6_motor_recieve, ID_7_motor_recieve;

} USB2CAN_CAN_Bus_Struct;

class Tangair_usb2can
{
public:  
	// CAN0
	int USB2CAN0_;

	Tangair_usb2can();
	~Tangair_usb2can();

	std::atomic<bool> running_{false};
	std::atomic<bool> imu_running_{false};

	// IMU 
	int IMU_Init();
    void StartIMUThread();

	std::thread sensorThread;
	void startThreadedMeasurement();
    void IMU_Shutdown();

	// DDS 
	void DDS_Init();
	void LowCmdMessageHandler(const void *messages);
	void PublishLowState();

	// Main control
	void StartReadLoop();
	void StartPositionLoop();
	void StopAllThreads();
	
    bool LoadConfigFromYAML(const std::string& filepath);
    bool CheckPositionAndGainValidity(const Matrix3x4d& positions,
                                      const Matrix3x4d& kp_array,
                                      const Matrix3x4d& kd_array);

	void SetMotorTarget(Motor_CAN_Send_Struct &motor, double pos, double kp = 3.0, double kd = 0.1);
	void SetTargetPosition(const Matrix3x4d& positions, 
							const Matrix3x4d& kp_array, 
							const Matrix3x4d& kd_array);

	void ResetPositionToZero();

	std::vector<double> GetMotorPositions();
    std::vector<double> GetMotorVelocity();
    std::vector<double> GetMotorTorque();

	void UpdateMotorState();
    std::vector<double> GetMotorFloatVector(const std::string& field);

	std::thread _CAN_TX_position_thread;
	void CAN_TX_position_thread();
	

	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

	/*********************************       ***Motor related***      ***********************************************/
	// Motor basic param
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	// Receive buffer
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;

	// Data form motor
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2;  // 模块0，can2
	
	// Init
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct* Leg_Data);

	void USB2CAN_CAN_Bus_Init();

	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data); // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);


private:
	int num_motor_ = 12; // ex: 12

	struct MotorState {
		std::vector<double> position;
		std::vector<double> velocity;  // 20個 0
		std::vector<double> torque;
	};

    MotorState motor_state_;         
    std::mutex motor_state_mutex;    

	std::vector<double> dof_pos;
	Matrix3x4d kp_array_;
	Matrix3x4d kd_array_;
	Matrix3x4d real_angles_;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> lowstate_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowStatePuberThreadPtr;
    ControlLimits control_limits_;
    LegJointLimitsMap joint_limits_per_leg_;  // ✅ 這一行最重要
};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
