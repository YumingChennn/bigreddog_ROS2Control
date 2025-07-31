#pragma once
#include <string>
#include <unordered_map>

struct ControlLimits {
    double kp_min, kp_max;
    double kd_min, kd_max;
};

struct JointLimit {
    double min, max;
};

struct JointLimits {
    JointLimit hip;
    JointLimit thigh;
    JointLimit calf;
};

using LegJointLimitsMap = std::unordered_map<std::string, JointLimits>;
