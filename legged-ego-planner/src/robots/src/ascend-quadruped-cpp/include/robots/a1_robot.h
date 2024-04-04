/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: inherited from quadruped robot, name as A1.
* Author: Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zhao Yao
*/

#ifndef ASCEND_QUADRUPED_CPP_A1_ROBOT_H
#define ASCEND_QUADRUPED_CPP_A1_ROBOT_H

#include "robot.h"

namespace Quadruped {
class A1Robot : public Robot {

public:

    A1Robot(std::string configFilePath);

    ~A1Robot() = default;

    void ReceiveObservation();

    void ReceiveObservationTest(LowState state);

    /**
     * @brief
     * @param motorCommands
     * @param motorControlMode
     */
    void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode);

    void ApplyAction(const std::vector<MotorCommand> &motorCommands, MotorMode motorControlMode);

    std::array<float, 60> ApplyActionTest(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode);

    void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode);

//    RobotInterface robotInterface;
//    LowState lowState;
//    HighState highState;

private:
    float yawOffset = 0.;

};
} // namespace Quadruped
#endif //ASCEND_QUADRUPED_CPP_A1_ROBOT_H
