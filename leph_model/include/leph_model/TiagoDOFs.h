#ifndef LEPH_MODEL_TIAGODOFS_H
#define LEPH_MODEL_TIAGODOFS_H

#include <string>
#include <vector>
#include <map>

namespace leph {

static const std::map<std::string, double> PostureDefault = {
    {"head_1_joint", 0.0},
    {"head_2_joint", 0.0},
    {"torso_lift_joint", 0.1},
    {"arm_1_joint", 0.05},
    {"arm_2_joint", -1.0},
    {"arm_3_joint", -0.15},
    {"arm_4_joint", 2.0},
    {"arm_5_joint", 0.0},
    {"arm_6_joint", 0.0},
    {"arm_7_joint", -M_PI_2},
    {"wheel_left_joint", 0.0},
    {"wheel_right_joint", 0.0},
    {"gripper_left_finger_joint", 0.045},
    {"gripper_right_finger_joint", 0.045},
    {"hand_thumb_joint", 0.0},
    {"hand_mrl_joint", 0.0},
    {"hand_index_joint", 0.0},
};

}

#endif

