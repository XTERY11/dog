/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22
*/

#ifndef ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H

#include <map>
#include "robots/motor.h"
#include "openloop_gait_generator.h"
#include "state_estimator/robot_estimator.h"
#include "planner/local_planner.h"

namespace Quadruped {
class RaibertSwingLegController {

public:
    RaibertSwingLegController(Robot *robot,
                              OpenloopGaitGenerator *gaitGenerator,
                              RobotEstimator *stateEstimator,
                              LocalPlanner *localPlanner,
                              Eigen::Matrix<float, 3, 1> desiredSpeed,
                              float desiredTwistingSpeed,
                              float desiredHeight,
                              float footClearance = 0.01);

    ~RaibertSwingLegController() = default;

    /**
     * @brief Quadratic interpolation function, used to generate polygon curve.
     * @param phase
     * @param start
     * @param mid
     * @param end
     * @return a float value with phase
     */
    float GenParabola(float phase, float start, float mid, float end);

    /**
     * @brief Generating the trajectory of the swing leg
     * @param inputPhase
     * @param startPos
     * @param endPos
     * @return foot position like (x,y,z)
     */
    Eigen::Matrix<float, 3, 1> GenSwingFootTrajectory(float inputPhase, Eigen::Matrix<float, 3, 1> startPos, Eigen::Matrix<float, 3, 1> endPos);

    void Reset(float currentTime);

    void Update(float currentTime);

    /** @brief google's function for velocity mode control */
    std::map<int, Eigen::Matrix<float, 5, 1>> GetAction();

    std::tuple<std::map<int, Eigen::Matrix<float, 5, 1>>, Eigen::Matrix<float, 3, 1>, Eigen::Matrix<float, 3, 1>>
    TestGetAction(Eigen::Matrix<float, 3, 1> testComVelocity, float testYawDot,
                  Eigen::Matrix<float, 3, 4> testHipPositions,
                  Eigen::Matrix<int, 3, 1> testJointIdx,
                  Eigen::Matrix<float, 3, 1> testJointAngles,
                  Eigen::Matrix<int, 4, 1> testLegState,
                  Eigen::Matrix<float, 4, 1> testStanceDuration,
                  Eigen::Matrix<float, 4, 1> testPhase,
                  Eigen::Matrix<float, 3, 4> testPhaseSwitchFootLocalPos,
                  Eigen::Matrix<float, 12, 1> testKps,
                  Eigen::Matrix<float, 12, 1> testKds);
    std::tuple<std::map<int, Eigen::Matrix<float, 5, 1>>, Eigen::Matrix<float, 3, 1>, Eigen::Matrix<float, 3, 1>>
    GetActionTest(Eigen::Matrix<float, 3, 1> testComVelocity, float testYawDot,
                  Eigen::Matrix<float, 3, 4> testHipPositions,
                  Eigen::Matrix<int, 3, 1> testJointIdx,
                  Eigen::Matrix<float, 3, 1> testJointAngles,
                  Eigen::Matrix<int, 4, 1> testLegState,
                  Eigen::Matrix<float, 4, 1> testStanceDuration,
                  Eigen::Matrix<float, 4, 1> testPhase,
                  Eigen::Matrix<float, 3, 4> testPhaseSwitchFootLocalPos,
                  Eigen::Matrix<float, 12, 1> testKps,
                  Eigen::Matrix<float, 12, 1> testKds);


    Eigen::Matrix<float, 3, 1> desiredSpeed; // appear in velocity mode usually.
    float desiredTwistingSpeed; // appear in velocity mode usually.

    Robot *robot;
    OpenloopGaitGenerator *gaitGenerator;
    RobotEstimator *stateEstimator;
    LocalPlanner *localPlanner;
    Eigen::Matrix<int, 4, 1> lastLegState;
    Eigen::Matrix<float, 3, 1> desiredHeight;
    std::map<int, float> swigJointAngles;
    std::map<int, std::tuple<float, float, int>> swingJointAnglesVelocities;
    Eigen::Matrix<float, 3, 4> phaseSwitchFootLocalPos; // foot positions in base frame when switch leg state
    Eigen::Matrix<float, 3, 4> phaseSwitchFootGlobalPos; //foot positions in world frame when switch leg state
    Eigen::Matrix<float, 3, 4> footHoldInWorldFrame; //footholds in world frame
    const int numMotorOfOneLeg = 3;
};
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_RAIBERT_SWING_LEG_CONTROLLER_H
