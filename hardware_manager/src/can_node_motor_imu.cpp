// Copyright (c) 2023–2025 TANGAIR
// SPDX-License-Identifier: Apache-2.0

#include "Tangair_usb2can_motor_imu.h"
#include <memory>
#include <csignal>
#include <unistd.h>
#include <sched.h>
#include <atomic>
#include <functional>
#include <unordered_map>

using std::chrono::milliseconds;

std::shared_ptr<Tangair_usb2can> CAN_ptr;

volatile sig_atomic_t shutdown_requested = 0;

void signal_callback_handler(int signum) {
    shutdown_requested = 1;
}

int main(int argc, const char **argv) {
    if (argc < 2)
        ChannelFactory::Instance()->Init(1, "lo");
    else
        ChannelFactory::Instance()->Init(0, argv[1]);

    std::cout << "Press enter to start";

    // 設定 real-time 行程排程
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] 設定即時排程失敗。\n";
    }
    

    // ========== 開始主程式 ==========
    CAN_ptr = std::make_shared<Tangair_usb2can>();
    signal(SIGINT, signal_callback_handler);

    constexpr int kDefaultDelayUs = 120;

    CAN_ptr->DDS_Init();
    CAN_ptr->IMU_Init();
    CAN_ptr->StartIMUThread();
    
    CAN_ptr->LoadConfigFromYAML("/home/crazydogv2/test/bigreddog_ROS2Control/hardware_manager/config/config.yaml");


    std::unordered_map<std::string, std::function<void()>> command_map = {
        {"enable", []() {
            CAN_ptr->ENABLE_ALL_MOTOR(kDefaultDelayUs); 
        }},
        {"disable", []() { 
            CAN_ptr->DISABLE_ALL_MOTOR(kDefaultDelayUs); 
        }},
        {"passive", []() { 
            CAN_ptr->PASSIVE_ALL_MOTOR(kDefaultDelayUs); 
        }},
        {"set", []() { 
            CAN_ptr->ZERO_ALL_MOTOR(kDefaultDelayUs); 
        }},
        {"reset", []() {
            CAN_ptr->StopAllThreads();
            CAN_ptr->ResetPositionToZero();
        }},
        {"position", []() {
            CAN_ptr->StopAllThreads();
            CAN_ptr->StartPositionLoop();
        }},
        {"stop", []() {
            CAN_ptr->StopAllThreads();
        }},
        {"exit", []() {
            CAN_ptr->StopAllThreads();
            std::exit(0); // 安全退出
        }},
    };

    std::cout << "\n請輸入指令啟動馬達操作：\n(enable / disable / passive / set / reset / position / stop / exit)\n";

    while (true) {
        if (shutdown_requested) {
            std::cout << "\n[INFO] 收到中斷訊號，正在安全結束...\n";
            CAN_ptr->StopAllThreads();
            break;
        }

        std::string input;
        std::cout << ">> ";
        std::cin >> input;

        auto cmd = command_map.find(input);
        if (cmd != command_map.end()) {
            cmd->second(); // 執行對應 lambda
        } else {
            std::cout << "[提示] 不支援的指令，請輸入：enable / disable / passive / set / reset / position / stop / exit\n";
        }
    }

    std::cout << "[INFO] 程式結束。\n";
    return 0;
}
