/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: a interface of robot locomotion controller.
* Author: Zhu Yijie
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "mpc_controller/locomotion_controller.h"
namespace Quadruped {
LocomotionController::LocomotionController(Robot *robotIn,
                                           OpenloopGaitGenerator *gaitGeneratorIn,
                                           RobotEstimator *stateEstimatorIn,
                                           ComAdjuster *comAdjusterIn,
                                           RaibertSwingLegController *swingLegControllerIn,
                                           TorqueStanceLegController *stanceLegControllerIn) :
    robot(robotIn), gaitGenerator(gaitGeneratorIn), stateEstimator(stateEstimatorIn), comAdjuster(comAdjusterIn),
    swingLegController(swingLegControllerIn), stanceLegController(stanceLegControllerIn)
{
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.;
}

void LocomotionController::Reset()
{
    resetTime = robot->GetTimeSinceReset();
    timeSinceReset = 0.;
    gaitGenerator->Reset(timeSinceReset);
    stateEstimator->Reset(timeSinceReset);
    comAdjuster->Reset(timeSinceReset);
    swingLegController->Reset(timeSinceReset);
    stanceLegController->Reset(timeSinceReset);
}

void LocomotionController::Update()
{
    timeSinceReset = robot->GetTimeSinceReset() - resetTime;
    gaitGenerator->Update(timeSinceReset);
    stateEstimator->Update(timeSinceReset);
    comAdjuster->Update(timeSinceReset);
    swingLegController->Update(timeSinceReset);
    stanceLegController->Update(timeSinceReset);

}

std::tuple<std::vector<MotorCommand>, Eigen::Matrix<float, 3, 4>> LocomotionController::GetAction()
{
    action.clear();
    // Returns the control ouputs (e.g. positions/torques) for all motors. type: map
    auto swingAction = swingLegController->GetAction();
    auto [stanceAction, qpSol] = stanceLegController->GetAction(); // map<int, MotorCommand>
    std::vector<MotorCommand> action;
    for (int joint_id = 0; joint_id < robot->numMotors; ++joint_id) {
        auto it = swingAction.find(joint_id);
        if (it != swingAction.end()) {
            action.push_back(it->second);
        } else {
            action.push_back(stanceAction[joint_id]);
        }
    }
    return {action, qpSol};
}
} // namespace Quadruped

                                        