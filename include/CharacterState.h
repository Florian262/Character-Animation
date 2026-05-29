#ifndef CHARACTER_STATE_H
#define CHARACTER_STATE_H

#include "MathUtils.h"
#include <vector>

/**
 * @brief Represents the physical state of a single joint.
 * Used for PD control calculations.
 */
struct JointState {
    // Current state (computed by physics/integration)
    Quaternion q_cur;
    Vector3D   w_cur; // Angular velocity (rad/s)

    // Reference state (sampled from animation clip)
    Quaternion q_ref;
    Vector3D   w_ref; // Target angular velocity

    // Control output
    Vector3D   torque;

    JointState() : q_cur(), w_cur(), q_ref(), w_ref(), torque() {}
};

/**
 * @brief Represents the full state of the character, including root and major joints.
 */
struct CharacterState {
    // Root Transform (World Space)
    Vector3D   root_position;
    Quaternion root_rotation;

    // Root Velocities
    Vector3D   root_linear_velocity;
    Vector3D   root_angular_velocity;

    // Major Joints (10 joints as per spec)
    // Order: UpperArm L/R, LowerArm L/R, Thigh L/R, Calf L/R, Foot L/R
    JointState joints[10];

    // Balance State (Critical for running simulation)
    Vector3D center_of_mass;
    Vector3D com_velocity;

    // Status
    bool is_grounded;
    double accumulated_time;

    CharacterState() 
        : root_position(), root_rotation(), root_linear_velocity(), 
          root_angular_velocity(), center_of_mass(), com_velocity(),
          is_grounded(false), accumulated_time(0.0) {}
};

#endif // CHARACTER_STATE_H
