/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#ifndef ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
#define ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H

#include "utils/se3.h"
#include "robots/motor.h"
#include "robots/robot.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "state_estimator/robot_estimator.h"
#include "planner/com_adjuster.h"
#include "planner/local_planner.h"


namespace Quadruped {
class TorqueStanceLegController {
public:
    TorqueStanceLegController(Robot *robot,
                              OpenloopGaitGenerator *gaitGenerator,
                              RobotEstimator *robotVelocityEstimator,
                              ComAdjuster *comAdjuster,
                              LocalPlanner *localPlanner,
                              Eigen::Matrix<float, 3, 1> desired_speed,
                              float desiredTwistingSpeed,
                              float desiredBodyHeight,
                              int numLegs,
                              std::string configFilepath,
                              std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45});

    ~TorqueStanceLegController() = default;

    void Reset(float currentTime);

    void Update(float currentTime);

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetAction();

    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> GetActionTest(
            Eigen::Matrix<float, 4, 1> desiredLegStates,
            Eigen::Matrix<float, 3, 1> robotComPosition,
            Eigen::Matrix<float, 3, 1> robotComVelocity,
            Eigen::Matrix<float, 3, 1> robotComRpy,
            Eigen::Matrix<float, 3, 1> robotComRpyRate,
            Eigen::Matrix<float, 3, 1> desiredComPosition,
            Eigen::Matrix<float, 3, 1> desiredComVelocity,
            Eigen::Matrix<float, 3, 1> desiredComAngularVelocity,
            Eigen::Matrix<float, 3, 4> contactForces,
            std::vector<std::map<int, float>> motorTorques4);

//private:
    Robot *robot;
    OpenloopGaitGenerator *gaitGenerator;
    RobotEstimator *robotEstimator;
    ComAdjuster* comAdjuster;
    LocalPlanner *localPlanner;
    Eigen::Matrix<float, 3, 1> desiredSpeed = {0., 0., 0.};
    float desiredTwistingSpeed = 0.;
    float desiredBodyHeight = 0.45; //overwrite in the class constructor by robot->bodyHeight
    int numLegs = 4;
    std::vector<float> frictionCoeffs = {0.45, 0.45, 0.45, 0.45};
    std::string configFilepath;

    int force_dim;
    Eigen::Matrix<float, 6, 1> KP;
    Eigen::Matrix<float, 6, 1> KD;
    Eigen::Matrix<float, 6, 1> maxDdq;
    Eigen::Matrix<float, 6, 1> minDdq;
    Eigen::Matrix<float, 6, 1> accWeight;
};
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_TORQUE_STANCE_LEG_CONTROLLER_H
