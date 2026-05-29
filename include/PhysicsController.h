#ifndef PHYSICS_CONTROLLER_H
#define PHYSICS_CONTROLLER_H

#include "CharacterState.h"

/**
 * @brief Handles PD control and physics-based animation logic.
 */
class PhysicsController {
public:
    /**
     * @brief Computes torques for all 10 major joints based on the PD control law.
     * 
     * @param state The character state containing current and reference joint data.
     * @param kp Proportional gain (stiffness).
     * @param kd Derivative gain (damping).
     */
    static void computeJointTorques(CharacterState& state, double kp, double kd);

    /**
     * @brief Specialized version with per-joint gains (useful for tuning).
     */
    static void computeJointTorques(CharacterState& state, const double kp[10], const double kd[10]);
};

#endif // PHYSICS_CONTROLLER_H
