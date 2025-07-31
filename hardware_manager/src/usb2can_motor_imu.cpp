// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can_motor_imu.h"
#include <chrono>
#include <cmath>
#include <array> 
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <mutex>
#include <thread> 

#include "callback_handler.h"
#include <iomanip>
#include <string>
#include <cassert>

#include <yaml-cpp/yaml.h>

using namespace std::chrono;
using namespace std;

using std::chrono::milliseconds;

XsControl* control = nullptr;
XsDevice* device = nullptr;
XsPortInfo mtPort;
CallbackHandler callback;
SensorData sensorData;

/// @brief 构造函数，初始化
/// @return
Tangair_usb2can::Tangair_usb2can()
{

    USB2CAN0_ = openUSBCAN("/dev/ttyRedDog");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                  << "ttyRedDog open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "ttyRedDog opened ,num=" << USB2CAN0_ << std::endl;

    // 电机ID配置
    USB2CAN_CAN_Bus_Init();

    // 启动成功
    std::cout << std::endl
              << "ttyRedDog   NODE INIT__OK   by TANGAIR" << std::endl
              << std::endl
              << std::endl;
}

/// @brief 析构函数
Tangair_usb2can::~Tangair_usb2can()
{
    std::cout << "End";
    StopAllThreads();

    // 关闭设备
    closeUSBCAN(USB2CAN0_);
}

// /*********************************       *** IMU related***      ***********************************************/

int Tangair_usb2can::IMU_Init()
{
    cout << "Creating XsControl object..." << endl;
    control = XsControl::construct();
    assert(control != 0);

    cout << "Scanning for devices..." << endl;
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();

    for (auto const &portInfo : portInfoArray) {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig()) {
            mtPort = portInfo;
            break;
        }
    }

    if (mtPort.empty()) {
        cerr << "No MTi device found. Aborting." << endl;
        return -1;
    }
    
    cout << "Found device @ port: " << mtPort.portName().toStdString() << endl;

    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate())) {
        cerr << "Could not open port. Aborting." << endl;
        return -1;
    }

    device = control->device(mtPort.deviceId());
    assert(device != nullptr);
    device->addCallbackHandler(&callback);

    if (!device->gotoConfig()) {
        cerr << "Failed to enter config mode." << endl;
        return -1;
    }

    device->readEmtsAndDeviceConfiguration();

    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

    if (device->deviceId().isImu()) {
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 100));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
    } else if (device->deviceId().isVru() || device->deviceId().isAhrs()) {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 100));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 1000));
        configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
    } else if (device->deviceId().isGnss()) {
        configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_LatLon, 100));
        configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 100));
        configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 100));
    }

    if (!device->setOutputConfiguration(configArray)) {
        cerr << "Failed to configure device." << endl;
        return -1;
    }

    if (device->createLogFile("logfile.mtb") != XRV_OK) {
        cerr << "Failed to create log file." << endl;
        return -1;
    }

    return 0;
}

void Tangair_usb2can::StartIMUThread()
{   
    imu_running_ = true;
    sensorThread = std::thread(&Tangair_usb2can::startThreadedMeasurement, this);
}

void Tangair_usb2can::startThreadedMeasurement()
{   
    if (!device->gotoMeasurement()) {
        cerr << "Failed to enter measurement mode." << endl;
        return;
    }

    if (!device->startRecording()) {
        cerr << "Failed to start recording." << endl;
        return;
    }

    // === 加入計時與計數器 ===
    int packetCount = 0;
    int64_t lastPrintTime = XsTime::timeStampNow();

    while (imu_running_) // 不再限制時間，只依照 imu_running_ 控制
    {   
        if (callback.packetAvailable()) 
        {
            XsDataPacket packet = callback.getNextPacket();
            cout << setw(5) << fixed << setprecision(2);
            if (packet.containsCalibratedData()) {
                sensorData.acc = packet.calibratedAcceleration();
                sensorData.gyr = packet.calibratedGyroscopeData();
                sensorData.mag = packet.calibratedMagneticField();
            }

            if (packet.containsRateOfTurnHR()) {
                sensorData.gyr = packet.rateOfTurnHR();

                // cout << " |Gyr X:" << sensorData.gyr[0]
				// 	<< ", Gyr Y:" << sensorData.gyr[1]
				// 	<< ", Gyr Z:" << sensorData.gyr[2] << endl;
            }

            if (packet.containsOrientation())
            {
                sensorData.quat = packet.orientationQuaternion();
                sensorData.euler = packet.orientationEuler();

				// cout << "q0:" << sensorData.quat.w()
				// 	<< ", q1:" << sensorData.quat.x()
				// 	<< ", q2:" << sensorData.quat.y()
				// 	<< ", q3:" << sensorData.quat.z();

				// cout << " |Roll:" << sensorData.euler.roll()
				// 	<< ", Pitch:" << sensorData.euler.pitch()
				// 	<< ", Yaw:" << sensorData.euler.yaw();
            }

            if (packet.containsLatitudeLongitude())
            {
                sensorData.latlon = packet.latitudeLongitude();
                // cout << " |Lat:" << sensorData.latlon[0]
                //      << ", Lon:" << sensorData.latlon[1];
            }

            if (packet.containsAltitude())
                sensorData.altitude = packet.altitude();
                // cout << " |Alt:" << sensorData.altitude;

            if (packet.containsVelocity())
            {
                sensorData.velocity = packet.velocity(XDI_CoordSysEnu);
                // cout << " |E:" << sensorData.velocity[0]
                //      << ", N:" << sensorData.velocity[1]
                //      << ", U:" << sensorData.velocity[2];
            }
            
            packetCount++;
            int64_t now = XsTime::timeStampNow();
            if (now - lastPrintTime >= 1000) {  // 每秒顯示一次
                // cout << "\r[INFO] IMU packet rate: " << packetCount << " Hz" << flush;
                packetCount = 0;
                lastPrintTime = now;
            }
            cout << flush;
        }

        XsTime::msleep(1);  // 避免 CPU 過度負載
    }

    cout << "\n[INFO] Measurement thread finished." << endl;
}

void Tangair_usb2can::IMU_Shutdown()
{
    imu_running_ = false;

    if (sensorThread.joinable())
    sensorThread.join();

    device->stopRecording();
    device->closeLogFile();
    control->closePort(mtPort.portName().toStdString());
    control->destruct();
}

/*********************************       *** DDS related***      ***********************************************/

void Tangair_usb2can::DDS_Init()
{   
    // /*create publisher*/
    lowstate_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_publisher->InitChannel();

    if (!lowstate_publisher) {
        std::cerr << "[ERROR] lowstate_publisher is null." << std::endl;
        return;
    } else {
        std::cout << "[INFO] lowstate_publisher 建立成功，準備開始傳送資料。" << std::endl;
    }

    /*create subscriber*/
    lowcmd_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_subscriber->InitChannel(std::bind(&Tangair_usb2can::LowCmdMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &Tangair_usb2can::PublishLowState, this);
    // std::cout << "[DEBUG] "<< std::endl;
}

void Tangair_usb2can::LowCmdMessageHandler(const void *msg)
{   
    const unitree_go::msg::dds_::LowCmd_ *cmd = static_cast<const unitree_go::msg::dds_::LowCmd_ *>(msg);
    if (!cmd) {
        std::cerr << "[ERROR] Received null pointer\n";
        return;
    }

    const auto& motor_cmds = cmd->motor_cmd();
    if (motor_cmds.size() < 12) {
        std::cerr << "[ERROR] motor_cmd size too small: " << motor_cmds.size() << std::endl;
        return;
    }

    dof_pos.clear();
    Matrix3x4d kp_temp = Matrix3x4d::Zero();
    Matrix3x4d kd_temp = Matrix3x4d::Zero();

    for (int i = 0; i < 12; ++i) {
        int leg = i / 4;
        int joint = i % 4;

        const auto& m = motor_cmds[i];
        double q  = m.q();
        double kp = m.kp();
        double kd = m.kd();

        // std::cout << "[DEBUG] motor[" << i << "] q=" << q << " kp=" << kp << " kd=" << kd << std::endl;

        dof_pos.push_back(q);
        kp_temp(leg, joint) = kp;
        kd_temp(leg, joint) = kd;
    }
    Matrix3x4d temp = mujoco_ang2real_ang(dof_pos);
    
    real_angles_ = temp;
    kp_array_ = kp_temp;
    kd_array_ = kd_temp;
}

void Tangair_usb2can::PublishLowState()
{   
    // std::cout << "[DEBUG] PublishLowState() called!" << std::endl;
    std::vector<double> pos = GetMotorPositions();
    std::vector<double> vel = GetMotorVelocity();

    if ((int)pos.size() < num_motor_ || (int)vel.size() < num_motor_) {
        std::cerr << "Please make sure match the start pos.\n";
        motor_state_.position = {0.0, 1.6, -2.8, -0.0, 1.6, -2.8, -0.0, -1.6, 2.8, 0.0, -1.6, 2.8};
        motor_state_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return;
    }

    unitree_go::msg::dds_::LowState_ low_state_go_{};

    for (int i = 0; i < num_motor_; ++i) {
        low_state_go_.motor_state()[i].q() = pos[i];
        low_state_go_.motor_state()[i].dq() = vel[i];
        low_state_go_.motor_state()[i].tau_est() = 0;
        
        // std::cout << "[Motor " << i << "] Position (q): " << pos[i] << std::endl;
    }

    // std::cout << "[CHECK] motor_state size = " << low_state_go_.motor_state().size() << std::endl;

    low_state_go_.imu_state().quaternion()[0] = sensorData.quat.w();
    low_state_go_.imu_state().quaternion()[1] = sensorData.quat.x();
    low_state_go_.imu_state().quaternion()[2] = sensorData.quat.y();
    low_state_go_.imu_state().quaternion()[3] = sensorData.quat.z();

    low_state_go_.imu_state().gyroscope()[0] = sensorData.gyr[0];
    low_state_go_.imu_state().gyroscope()[1] = sensorData.gyr[1];
    low_state_go_.imu_state().gyroscope()[2] = sensorData.gyr[2];

    lowstate_publisher->Write(low_state_go_);
}

/*********************************       *** Main control related ***      ***********************************************/

void Tangair_usb2can::StartReadLoop() {
    if (running_) return;
    running_ = true;

    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);
}

void Tangair_usb2can::StartPositionLoop() {
    if (running_) return;
    running_ = true;

    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);
    _CAN_TX_position_thread = std::thread(&Tangair_usb2can::CAN_TX_position_thread, this);
}

void Tangair_usb2can::StopAllThreads() {
    if (!running_) return;
    running_ = false;
    
    IMU_Shutdown();

    if (_CAN_RX_device_0_thread.joinable()) _CAN_RX_device_0_thread.join();
    if (_CAN_TX_position_thread.joinable()) _CAN_TX_position_thread.join();

    DISABLE_ALL_MOTOR(237);
    std::cout << "[Tangair] 所有執行緒已安全停止。\n";
}

void Tangair_usb2can::SetMotorTarget(Motor_CAN_Send_Struct &motor, double pos, double kp, double kd) {
    motor.position = pos;
    motor.speed = 0;
    motor.torque = 0;
    motor.kp = kp;
    motor.kd = kd;
}

bool Tangair_usb2can::LoadConfigFromYAML(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);

        auto ctrl = config["controller_limits"];
        control_limits_.kp_min = ctrl["kp_min"].as<double>();
        control_limits_.kp_max = ctrl["kp_max"].as<double>();
        control_limits_.kd_min = ctrl["kd_min"].as<double>();
        control_limits_.kd_max = ctrl["kd_max"].as<double>();

        auto joints = config["joint_limits"];
        std::vector<std::string> legs = { "FR", "FL", "RR", "RL" };

        for (const auto& leg : legs) {
            JointLimits limits;
            limits.hip.min   = joints[leg]["hip"]["min"].as<double>();
            limits.hip.max   = joints[leg]["hip"]["max"].as<double>();
            limits.thigh.min = joints[leg]["thigh"]["min"].as<double>();
            limits.thigh.max = joints[leg]["thigh"]["max"].as<double>();
            limits.calf.min  = joints[leg]["calf"]["min"].as<double>();
            limits.calf.max  = joints[leg]["calf"]["max"].as<double>();

            joint_limits_per_leg_[leg] = limits;
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to load YAML config: " << e.what() << std::endl;
        return false;
    }
}

bool Tangair_usb2can::CheckPositionAndGainValidity(const Matrix3x4d& positions, 
                                                   const Matrix3x4d& kp_array, 
                                                   const Matrix3x4d& kd_array) {
    const std::array<std::string, 4> leg_names = { "FR", "FL", "RR", "RL" };

    for (int row = 0; row < 3; ++row) {  // 0: calf, 1: thigh, 2: hip
        for (int col = 0; col < 4; ++col) {
            double pos = positions(row, col);
            double kp  = kp_array(row, col);
            double kd  = kd_array(row, col);

            const auto& leg_name = leg_names[col];
            const auto& joint_limit = joint_limits_per_leg_.at(leg_name);

            double pos_min, pos_max;

            if (row == 2) { // hip
                pos_min = joint_limit.hip.min;
                pos_max = joint_limit.hip.max;
            } else if (row == 1) { // thigh
                pos_min = joint_limit.thigh.min;
                pos_max = joint_limit.thigh.max;
            } else { // calf
                pos_min = joint_limit.calf.min;
                pos_max = joint_limit.calf.max;
            }

            if (!std::isfinite(pos) || pos < pos_min || pos > pos_max) {
                std::cerr << "[ERROR] Invalid position (" << pos << ") at [" << row << ", " << col
                          << "] (" << leg_name << "), limit: [" << pos_min << ", " << pos_max << "]\n";
                return false;
            }

            if (!std::isfinite(kp) || kp < control_limits_.kp_min || kp > control_limits_.kp_max) {
                std::cerr << "[ERROR] Invalid kp (" << kp << ") at [" << row << ", " << col
                          << "] (" << leg_name << "), limit: [" << control_limits_.kp_min << ", " << control_limits_.kp_max << "]\n";
                return false;
            }

            if (!std::isfinite(kd) || kd < control_limits_.kd_min || kd > control_limits_.kd_max) {
                std::cerr << "[ERROR] Invalid kd (" << kd << ") at [" << row << ", " << col
                          << "] (" << leg_name << "), limit: [" << control_limits_.kd_min << ", " << control_limits_.kd_max << "]\n";
                return false;
            }
        }
    }

    return true;
}

void Tangair_usb2can::SetTargetPosition(const Matrix3x4d &positions, 
                                        const Matrix3x4d &kp_array, 
                                        const Matrix3x4d &kd_array) {
    if (!CheckPositionAndGainValidity(positions, kp_array, kd_array)) {
        std::cerr << "[SetTargetPosition] 檢查未通過，取消指令發送。\n";
        return;
    }    

    // FR
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_1_motor_send, positions(2, 0), kp_array(2, 0), kd_array(2, 0));
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_2_motor_send, positions(1, 0), kp_array(1, 0), kd_array(1, 0));
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_3_motor_send, positions(0, 0), kp_array(0, 0), kd_array(0, 0));

    // FL
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_5_motor_send, positions(2, 1), kp_array(2, 1), kd_array(2, 1));
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_6_motor_send, positions(1, 1), kp_array(1, 1), kd_array(1, 1));
    SetMotorTarget(USB2CAN0_CAN_Bus_2.ID_7_motor_send, positions(0, 1), kp_array(0, 1), kd_array(0, 1));

    // RR
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_1_motor_send, positions(2, 2), kp_array(2, 2), kd_array(2, 2));
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_2_motor_send, positions(1, 2), kp_array(1, 2), kd_array(1, 2));
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_3_motor_send, positions(0, 2), kp_array(0, 2), kd_array(0, 2));

    // RL
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_5_motor_send, positions(2, 3), kp_array(2, 3), kd_array(2, 3));
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_6_motor_send, positions(1, 3), kp_array(1, 3), kd_array(1, 3));
    SetMotorTarget(USB2CAN0_CAN_Bus_1.ID_7_motor_send, positions(0, 3), kp_array(0, 3), kd_array(0, 3));
}

void Tangair_usb2can::ResetPositionToZero()
{   
    Matrix3x4d pos;
    pos <<  2.8, -2.8, -2.8,  2.8,
            -1.6,  1.6,  1.6, -1.6,
            0.0,  0.0,  0.0,  0.0;

    // std::cout << "Target Position (pos):\n" << pos(2, 0) << std::endl;

    Matrix3x4d kp = Matrix3x4d::Constant(10.0);
    Matrix3x4d kd = Matrix3x4d::Constant(0.2);

    SetTargetPosition(pos, kp, kd);
    CAN_TX_ALL_MOTOR(120);
}

std::vector<double> Tangair_usb2can::GetMotorPositions() {
    std::lock_guard<std::mutex> lock(motor_state_mutex);
    return motor_state_.position;
}

std::vector<double> Tangair_usb2can::GetMotorVelocity() {
    std::lock_guard<std::mutex> lock(motor_state_mutex);
    return motor_state_.velocity;
}

std::vector<double> Tangair_usb2can::GetMotorTorque() {
    std::lock_guard<std::mutex> lock(motor_state_mutex);
    return motor_state_.torque;
}

void Tangair_usb2can::UpdateMotorState() {
    motor_state_.position = GetMotorFloatVector("position");
    motor_state_.velocity = GetMotorFloatVector("velocity");
    motor_state_.torque   = GetMotorFloatVector("torque");
}

std::vector<double> Tangair_usb2can::GetMotorFloatVector(const std::string& field) {
    std::vector<double> result;
    const std::vector<std::pair<int, int>> motorMap = {
        {2, 1}, {2, 2}, {2, 3}, {2, 5}, {2, 6}, {2, 7},
        {1, 1}, {1, 2}, {1, 3}, {1, 5}, {1, 6}, {1, 7}
    };

    for (const auto& [bus_id, motor_id] : motorMap) {
        auto& motor = (bus_id == 1 ? USB2CAN0_CAN_Bus_1 : USB2CAN0_CAN_Bus_2);
        auto& recv = [&]() -> Motor_CAN_Recieve_Struct& {
            switch (motor_id) {
                case 1: return motor.ID_1_motor_recieve;
                case 2: return motor.ID_2_motor_recieve;
                case 3: return motor.ID_3_motor_recieve;
                case 5: return motor.ID_5_motor_recieve;
                case 6: return motor.ID_6_motor_recieve;
                case 7: return motor.ID_7_motor_recieve;
                default: throw std::runtime_error("Invalid motor ID");
            }
        }();

        float val = 0;
        if (field == "position") val = recv.current_position_f;
        else if (field == "velocity") val = recv.current_speed_f;
        else if (field == "torque") val = recv.current_torque_f;
        else throw std::invalid_argument("Invalid field: " + field);

        // 特定 motor 做反向處理
        if ((bus_id == 2 && (motor_id == 2 || motor_id == 3)) ||
            (bus_id == 1 && (motor_id == 1 || motor_id == 2 || motor_id == 3 || motor_id == 5))) {
            val *= -1;
        }

        result.push_back(val);
    }

    return result;
}


void Tangair_usb2can::CAN_TX_position_thread()
{
    std::cout << "[THREAD] CAN_TX_position_thread start\n";

    auto last_time_tx = high_resolution_clock::now();
    int count_tx = 0;

    ENABLE_ALL_MOTOR(120);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    while (running_) {
        count_tx++;

        // PrintMatrix("real_angles_", real_angles_);
        // PrintMatrix("kp_array_ (as kp)", kp_array_);
        // PrintMatrix("kd_array_ (as kd)", kd_array_);

        SetTargetPosition(real_angles_, kp_array_, kd_array_);

        CAN_TX_ALL_MOTOR(120);

        /********************************* ***TX Finish*** ***********************************************/

        std::lock_guard<std::mutex> lock(motor_state_mutex);
        UpdateMotorState();
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        auto now_tx = high_resolution_clock::now();
        auto duration_tx = duration_cast<seconds>(now_tx - last_time_tx).count();
        if (duration_tx >= 1) {
            std::cout << "[Frequency] CAN TX = " << count_tx << " Hz" << std::endl;
            count_tx = 0;
            last_time_tx = now_tx;
        }
    }

    std::cout << "CAN_TX_position_thread Exit~~" << std::endl;
}

/// @brief can设备0，接收线程函数
void Tangair_usb2can::CAN_RX_device_0_thread()
{
    auto last_time_rx = high_resolution_clock::now();
    int count_rx = 0;

    while (running_)
    {   
        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};

        // 阻塞1s接收
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);
        // 接收到数据
        if (recieve_re != -1)
        {   
            count_rx++;
            // 解码
            CAN_DEV0_RX.ERR = data_rx[0]>>4&0X0F;
            
            CAN_DEV0_RX.current_position = (data_rx[1]<<8)|data_rx[2]; //电机位置数据
			CAN_DEV0_RX.current_speed  = (data_rx[3]<<4)|(data_rx[4]>>4); //电机速度数据
			CAN_DEV0_RX.current_torque = ((data_rx[4]&0xF)<<8)|data_rx[5]; //电机扭矩数据
			CAN_DEV0_RX.current_temp_MOS  = data_rx[6];
            CAN_DEV0_RX.current_temp_Rotor  = data_rx[7];
  
            if (channel == 1) // 模块0，can1
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_1_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_1_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_1_motor_recieve = CAN_DEV0_RX;
                   
                    break;
                }
                case 0X12:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_2_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_2_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_2_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X13:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_3_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_3_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_3_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X15:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_5_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_5_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_5_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X16:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_6_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_6_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_6_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X17:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_1.ID_7_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_1.ID_7_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_1.ID_7_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                default:
                    break;
                }
            }
            else if (channel == 2) // 模块0，can2
            {
                switch (info_rx.canID)
                {
                case 0X11:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_1_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_1_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_1_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X12:
                {
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_2_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_2_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_2_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X13:
                {
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_3_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_3_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_3_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X15:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_5_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_5_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_5_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X16:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_6_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_6_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_6_motor_recieve = CAN_DEV0_RX;
                    break;
                }
                case 0X17:
                {   
                    CAN_DEV0_RX.current_position_f = uint_to_float(CAN_DEV0_RX.current_position, (P_MIN), (P_MAX), 16);
                    CAN_DEV0_RX.current_speed_f = uint_to_float(CAN_DEV0_RX.current_speed, (V_MIN), (V_MAX), 12);    
                    CAN_DEV0_RX.current_torque_f = uint_to_float(CAN_DEV0_RX.current_torque, (USB2CAN0_CAN_Bus_2.ID_7_motor_send.Tau_Min), (USB2CAN0_CAN_Bus_2.ID_7_motor_send.Tau_Max), 12);
                    USB2CAN0_CAN_Bus_2.ID_7_motor_recieve = CAN_DEV0_RX;
                    break;
                }
               
                default:
                    break;
                }
            }
            auto now_rx = high_resolution_clock::now();
            auto duration_rx = duration_cast<seconds>(now_rx - last_time_rx).count();
            if (duration_rx >= 1) {
            std::cout << "[Frequency] CAN RX = " << count_rx / 12 << " Hz" << std::endl;
            count_rx = 0;
            last_time_rx = now_rx;
            }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***电机相关***      ***********************************************/
/*****************************************************************************************************/

void Tangair_usb2can::USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct *CAN_Bus)
{
    CAN_Bus->ID_1_motor_send.id = 0X01;
    CAN_Bus->ID_1_motor_send.Tau_Min = T_MIN_8006;
    CAN_Bus->ID_1_motor_send.Tau_Max = T_MAX_8006;

    CAN_Bus->ID_2_motor_send.id = 0X02;
    CAN_Bus->ID_2_motor_send.Tau_Min = T_MIN_8006;
    CAN_Bus->ID_2_motor_send.Tau_Max = T_MAX_8006;

    CAN_Bus->ID_3_motor_send.id = 0X03;
    CAN_Bus->ID_3_motor_send.Tau_Min = T_MIN_8009;
    CAN_Bus->ID_3_motor_send.Tau_Max = T_MAX_8009;

    CAN_Bus->ID_5_motor_send.id = 0X05;
    CAN_Bus->ID_5_motor_send.Tau_Min = T_MIN_8006;
    CAN_Bus->ID_5_motor_send.Tau_Max = T_MAX_8006;

    CAN_Bus->ID_6_motor_send.id = 0X06;
    CAN_Bus->ID_6_motor_send.Tau_Min = T_MIN_8006;
    CAN_Bus->ID_6_motor_send.Tau_Max = T_MAX_8006;

    CAN_Bus->ID_7_motor_send.id = 0X07;
    CAN_Bus->ID_7_motor_send.Tau_Min = T_MIN_8009;
    CAN_Bus->ID_7_motor_send.Tau_Max = T_MAX_8009;
}

void Tangair_usb2can::USB2CAN_CAN_Bus_Init()
{
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_1);
    USB2CAN_CAN_Bus_inti_set(&USB2CAN0_CAN_Bus_2);
}

/// @brief 使能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;
    // printf("0x%02X", txMsg_CAN.canID);

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFC;

    int ret = sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFD;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 设置零点
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    txMsg_CAN.canID = Motor_Data->id;

    Data_CAN[0] = 0xFF;
    Data_CAN[1] = 0xFF;
    Data_CAN[2] = 0xFF;
    Data_CAN[3] = 0xFF;
    Data_CAN[4] = 0xFF;
    Data_CAN[5] = 0xFF;
    Data_CAN[6] = 0xFF;
    Data_CAN[7] = 0xFE;

    sendUSBCAN(dev, channel, &txMsg_CAN, Data_CAN);
}

/// @brief 电机控制
/// @param dev 模块设备号
/// @param channel can1或者can2
/// @param Motor_Data 电机数据
void Tangair_usb2can::CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data) 
{
    // 运控模式专用的局部变
    FrameInfo txMsg_Control = {
        .canID = Motor_Data->id,
        .frameType = STANDARD,
        .dataLength = 8,
    };
    uint8_t Data_CAN_Control[8];

    //限制范围
    if(Motor_Data->kp>KP_MAX) Motor_Data->kp=KP_MAX;
        else if(Motor_Data->kp<KP_MIN) Motor_Data->kp=KP_MIN;
    if(Motor_Data->kd>KD_MAX ) Motor_Data->kd=KD_MAX;
        else if(Motor_Data->kd<KD_MIN) Motor_Data->kd=KD_MIN;
    if(Motor_Data->position>P_MAX)	Motor_Data->position=P_MAX;
        else if(Motor_Data->position<P_MIN) Motor_Data->position=P_MIN;
    if(Motor_Data->speed>V_MAX)	Motor_Data->speed=V_MAX;
        else if(Motor_Data->speed<V_MIN) Motor_Data->speed=V_MIN;
    if(Motor_Data->torque>Motor_Data->Tau_Max)	Motor_Data->torque=Motor_Data->Tau_Max;
        else if(Motor_Data->torque<Motor_Data->Tau_Min) Motor_Data->torque=Motor_Data->Tau_Min;

    Data_CAN_Control[0] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16)>>8; //位置�?? 8
    Data_CAN_Control[1] = float_to_uint(Motor_Data->position, P_MIN, P_MAX, 16)&0xFF; //位置�?? 8
    Data_CAN_Control[2] = float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12)>>4; //速度�?? 8 �??
    Data_CAN_Control[3] = ((float_to_uint(Motor_Data->speed, V_MIN, V_MAX, 12)&0xF)<<4)|(float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12)>>8); //速度�?? 4 �?? KP �?? 4 �??
    Data_CAN_Control[4] = float_to_uint(Motor_Data->kp, KP_MIN, KP_MAX, 12)&0xFF; //KP �?? 8 �??
    Data_CAN_Control[5] = float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12)>>4; //Kd �?? 8 �??
    Data_CAN_Control[6] = ((float_to_uint(Motor_Data->kd, KD_MIN, KD_MAX, 12)&0xF)<<4)|(float_to_uint(Motor_Data->torque, Motor_Data->Tau_Min, Motor_Data->Tau_Max, 12)>>8); //KP �?? 4 位扭矩高 4 �??
    Data_CAN_Control[7] = float_to_uint(Motor_Data->torque, Motor_Data->Tau_Min, Motor_Data->Tau_Max, 12)&0xFF; //扭矩�?? 8

    int ret = sendUSBCAN(dev, channel, &txMsg_Control, Data_CAN_Control);
}

/// @brief 电机阻尼模式
/// @param dev
/// @param channel
/// @param Motor_Data
void Tangair_usb2can::Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data)
{
    Motor_Data->speed = 0;
    Motor_Data->kp = 0;
    Motor_Data->kd = 2.0;
    Motor_Data->torque = 0;

    CAN_Send_Control(dev, channel, Motor_Data);
}

void Tangair_usb2can::ENABLE_ALL_MOTOR(int delay_us)
{   
    // FRH
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRH
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLH
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLH
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRT
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRC
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Enable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Enable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::DISABLE_ALL_MOTOR(int delay_us)
{
    // FRH
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRH
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLH
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLH
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRT
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRC
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Disable(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Disable(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::ZERO_ALL_MOTOR(int delay_us)
{
    // FRH
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RRH
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // FLH
    // Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RLH
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRT
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RRT
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // FLT
    // Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RLT
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRC
    Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RRC
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // FLC
    // Motor_Zore(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // // RLC
    // Motor_Zore(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    // std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::PASSIVE_ALL_MOTOR(int delay_us)
{   
    // FRH
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRH
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLH
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLH
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRT
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRT
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLT
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLT
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    // FRC
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RRC
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // FLC
    Motor_Passive_SET(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
    // RLC
    Motor_Passive_SET(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

/// @brief can控制发送，12个电机的数据
// 目前能达到1000hz的控制频率--------3000hz的总线发送频率---------同一路can的发送间隔在300us
void Tangair_usb2can::CAN_TX_ALL_MOTOR(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    //FRH
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_1_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RRH
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_1_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FLH
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_5_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RLH
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_5_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //FRT
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_2_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RRT
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_2_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FLT
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_6_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RLT
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_6_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    //FRC
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_3_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RRC
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_3_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //FLC
    CAN_Send_Control(USB2CAN0_, 2, &USB2CAN0_CAN_Bus_2.ID_7_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    //RLC
    CAN_Send_Control(USB2CAN0_, 1, &USB2CAN0_CAN_Bus_1.ID_7_motor_send);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

/// @brief 辅助函数
Matrix3x4d mujoco_ang2real_ang(const std::vector<double>& dof_pos) {
    std::vector<std::string> motor_order = {
        "frd", "fld", "rrd", "rld",  // Lower legs
        "fru", "flu", "rru", "rlu",  // Upper legs
        "frh", "flh", "rrh", "rlh"   // Hips
    };

    std::vector<std::string> mujoco_order = {
        "frh", "fru", "frd",
        "flh", "flu", "fld",
        "rrh", "rru", "rrd",
        "rlh", "rlu", "rld"
    };

    std::vector<int> index_map;
    for (const auto& name : motor_order) {
        auto it = std::find(mujoco_order.begin(), mujoco_order.end(), name);
        if (it == mujoco_order.end()) {
            throw std::runtime_error("Motor name not found in mujoco_order: " + name);
        }
        index_map.push_back(std::distance(mujoco_order.begin(), it));
    }

    std::vector<double> reordered_dof_pos;
    for (int i : index_map) {
        reordered_dof_pos.push_back(dof_pos[i]);
    }

    Matrix3x4d result;
    result << 
        -reordered_dof_pos[0],  reordered_dof_pos[1],  -reordered_dof_pos[2],  reordered_dof_pos[3],
        -reordered_dof_pos[4],  reordered_dof_pos[5],  -reordered_dof_pos[6],  reordered_dof_pos[7],
        reordered_dof_pos[8],  reordered_dof_pos[9],  -reordered_dof_pos[10], -reordered_dof_pos[11];
    return result;
}

std::vector<double> real_ang2mujoco_ang(const std::vector<double>& dof_pos) {
    std::vector<std::string> motor_order = {
        "frd", "fld", "rrd", "rld",  // Lower legs
        "fru", "flu", "rru", "rlu",  // Upper legs
        "frh", "flh", "rrh", "rlh"   // Hips
    };

    std::vector<std::string> mujoco_order = {
        "frh", "fru", "frd",
        "flh", "flu", "fld",
        "rrh", "rru", "rrd",
        "rlh", "rlu", "rld"
    };

    // 建立 motor_order 名稱到 index 的映射
    std::unordered_map<std::string, int> name_to_index;
    for (int i = 0; i < motor_order.size(); ++i) {
        name_to_index[motor_order[i]] = i;
    }

    // 依照 mujoco_order 重排
    std::vector<double> reordered_dof_pos;
    for (const auto& name : mujoco_order) {
        reordered_dof_pos.push_back(dof_pos[name_to_index[name]]);
    }

    // 依照特定規則調整正負號
    reordered_dof_pos[1] = -reordered_dof_pos[1];
    reordered_dof_pos[2] = -reordered_dof_pos[2];
    reordered_dof_pos[6] = -reordered_dof_pos[6];
    reordered_dof_pos[7] = -reordered_dof_pos[7];
    reordered_dof_pos[8] = -reordered_dof_pos[8];
    reordered_dof_pos[9] = -reordered_dof_pos[9];

    return reordered_dof_pos;
}

void PrintMatrix(const std::string& name, const Eigen::Matrix<double, 3, 4>& matrix) {
    std::cout << "\n" << matrix << "\n";
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

bool isSensorDataValid(const SensorData& data)
{
    if (!data.acc.empty() && (data.acc[0] != 0 || data.acc[1] != 0 || data.acc[2] != 0))
        return true;
    if (!data.gyr.empty() && (data.gyr[0] != 0 || data.gyr[1] != 0 || data.gyr[2] != 0))
        return true;
    if (!data.mag.empty() && (data.mag[0] != 0 || data.mag[1] != 0 || data.mag[2] != 0))
        return true;
    if (data.quat.w() != 0 || data.quat.x() != 0 || data.quat.y() != 0 || data.quat.z() != 0)
        return true;
    if (data.euler.roll() != 0 || data.euler.pitch() != 0 || data.euler.yaw() != 0)
        return true;
    if (!data.latlon.empty() && (data.latlon[0] != 0 || data.latlon[1] != 0))
        return true;
    if (data.altitude != 0)
        return true;
    if (!data.velocity.empty() && (data.velocity[0] != 0 || data.velocity[1] != 0 || data.velocity[2] != 0))
        return true;

    return false;
}

