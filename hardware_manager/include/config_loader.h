#pragma once

struct ControlLimits {
    double kp_min, kp_max;
    double kd_min, kd_max;
};

struct JointLimit {
    double min, max;
};

struct JointLimits {
    JointLimit hip, thigh, calf;
};