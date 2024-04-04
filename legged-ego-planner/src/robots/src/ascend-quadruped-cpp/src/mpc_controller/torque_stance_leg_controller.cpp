/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Stance controller for stance foot.
* Author: Zang Yaohua & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: init the file. @ Zang Yaohua
*/

#include "mpc_controller/torque_stance_leg_controller.h"
#include "mpc_controller/qp_torque_optimizer.h"
using namespace std;
namespace Quadruped {

TorqueStanceLegController::TorqueStanceLegController(Robot *robot,
                                                     OpenloopGaitGenerator *gaitGenerator,
                                                     RobotEstimator *robotEstimator,
                                                     ComAdjuster *comAdjuster,
                                                     LocalPlanner *localPlanner,
                                                     Eigen::Matrix<float, 3, 1> desiredSpeed,
                                                     float desiredTwistingSpeed,
                                                     float desiredBodyHeight,
                                                     int numLegs,
                                                     std::string configFilepath,
                                                     std::vector<float> frictionCoeffs)
{
    this->robot = robot;
    this->gaitGenerator = gaitGenerator;
    this->robotEstimator = robotEstimator;
    this->comAdjuster = comAdjuster;
    this->localPlanner = localPlanner;
    this->configFilepath = configFilepath;
    this->desiredSpeed = desiredSpeed;
    this->desiredTwistingSpeed = desiredTwistingSpeed;
    this->desiredBodyHeight = desiredBodyHeight;
    this->numLegs = numLegs;
    this->frictionCoeffs = frictionCoeffs;

    //configFilepath is "config/stance_leg_controller.yaml".
    YAML::Node param = YAML::LoadFile(configFilepath);
    this->force_dim = param["stance_leg_params"]["force_dim"].as<int>();
    vector<float> v = param["stance_leg_params"]["KD"].as<vector<float>>();
    this->KD = Eigen::MatrixXf::Map(&v[0], 6, 1);
    v = param["stance_leg_params"]["KP"].as<vector<float>>();
    this->KP = Eigen::MatrixXf::Map(&v[0], 6, 1);
    v = param["stance_leg_params"]["max_ddq"].as<vector<float>>();
    this->maxDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
    v = param["stance_leg_params"]["min_ddq"].as<vector<float>>();
    this->minDdq = Eigen::MatrixXf::Map(&v[0], 6, 1);
    v = param["stance_leg_params"]["acc_weight"].as<vector<float>>();
    this->accWeight = Eigen::MatrixXf::Map(&v[0], 6, 1);
}

void TorqueStanceLegController::Reset(float currentTime)
{}

void TorqueStanceLegController::Update(float currentTime)
{}

std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegController::GetAction()
{
    Eigen::Matrix<int, 4, 1> contacts;
    Eigen::Matrix<float, 3, 1> robotComPosition = Eigen::Matrix<float, 3, 1>::Zero();
    Eigen::Matrix<float, 3, 1> robotComVelocity;
    Eigen::Matrix<float, 3, 1> robotComRpy;
    Eigen::Matrix<float, 3, 1> robotComRpyRate;
    Eigen::Matrix<float, 6, 1> robotQ;
    Eigen::Matrix<float, 6, 1> robotDq;

    Eigen::Matrix<float, 3, 1> desiredComPosition(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComVelocity(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComRpy(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 3, 1> desiredComAngularVelocity(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 6, 1> desiredQ;
    Eigen::Matrix<float, 6, 1> desiredDq;
    Eigen::Matrix<float, 3, 4> contactForces;

    /// leg contact status  ///
    for (int legId = 0; legId < 4; legId++) {
        int legState = gaitGenerator->desiredLegState[legId];
        if (legState == LegState::STANCE || legState == LegState::EARLY_CONTACT) {
            contacts[legId] = true;
        } else {
            contacts[legId] = false;
        }
    }

    /// robot status  ///
//    robotComPosition =
//        {robotVelocityEstimator->GetEstimatedPosition()[0], robotVelocityEstimator->GetEstimatedPosition()[1],
//         robot->basePosition[2]}; // 定x,y偏移 // base frame
//    robotComPosition =
//        {0.f, robotVelocityEstimator->GetEstimatedPosition()[1],
//         robot->basePosition[2]}; //定y偏移 // base frame

    robotComPosition = {0., 0., robot->basePosition[2]};     // base frame
    robotComVelocity = robotEstimator->GetEstimatedVelocity();      //base frame
    robotComRpy = robot->GetBaseRollPitchYaw();     //world frame
    robotComRpyRate = robot->GetBaseRollPitchYawRate();     //base frame

    robotQ << robotComPosition, robotComRpy;
    robotDq << robotComVelocity, robotComRpyRate;

    /// desired robot status  ///
    if (robot->controlParams["mode"] == LocomotionMode::VELOCITY_LOCOMOTION) {
        desiredComPosition = {0.f, 0.f, desiredBodyHeight};
        desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};
        desiredComRpy = {0.f, 0.f, 0.f};
        desiredComAngularVelocity = {0.f, 0.f, this->desiredTwistingSpeed};
    } else if (robot->controlParams["mode"] == LocomotionMode::POSITION_LOCOMOTION) {
        auto &comAdjPosInBaseFrame = comAdjuster->GetComPosInBaseFrame();
        desiredComPosition = {comAdjPosInBaseFrame[0], comAdjPosInBaseFrame[1],
                              desiredBodyHeight}; // get goal com position from comAdjuster, base frame
        desiredComVelocity = {desiredSpeed[0], desiredSpeed[1], 0.f};
        desiredComRpy =
            localPlanner->GetDesiredComPose().block(3, 0, 3, 1); // get goal rpy from localPlanner, world frame
        desiredComAngularVelocity = {0.f, 0.f, this->desiredTwistingSpeed};
    }

    desiredQ << desiredComPosition, desiredComRpy;
    desiredDq << desiredComVelocity, desiredComAngularVelocity;

    Eigen::Matrix<float, 6, 1> desiredDdq;
    desiredDdq = KP.cwiseProduct(desiredQ - robotQ) + KD.cwiseProduct(desiredDq - robotDq);
//    std::cout << "KD.cwiseProduct(desiredDq - robotDq):\n " << KD.cwiseProduct(desiredDq - robotDq) << endl;

    /// Clip ///
    for (int i = 0; i < 6; i++) {
        if (desiredDdq(i, 0) > this->maxDdq(i, 0)) {
            desiredDdq(i, 0) = this->maxDdq(i, 0);
        }
        if (desiredDdq(i, 0) < this->minDdq(i, 0)) {
            desiredDdq(i, 0) = this->minDdq(i, 0);
        }
    }
    // desiredDdq.min(maxDdq).max(minDdq)

    /// Compute Contact Force  ///
    contactForces << ComputeContactForce(this->robot, desiredDdq, contacts, this->accWeight);
//    cout << "contactForces: \n " << contactForces << endl;

    map<int, MotorCommand> action;
    map<int, float> motorTorques;
    for (int legId = 0; legId < 4; ++legId) {
        motorTorques = this->robot->MapContactForceToJointTorques(legId, contactForces.col(legId));
        for (map<int, float>::iterator it = motorTorques.begin(); it != motorTorques.end(); ++it) {
            MotorCommand temp{0., 0., 0., 0., it->second};
            action[it->first] = temp;
        }
    }
    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);

    return actionContactForce;
}


std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> TorqueStanceLegController::GetActionTest(
    Eigen::Matrix<float, 4, 1> desiredLegStates,
    Eigen::Matrix<float, 3, 1> robotComPosition,
    Eigen::Matrix<float, 3, 1> robotComVelocity,
    Eigen::Matrix<float, 3, 1> robotComRpy,
    Eigen::Matrix<float, 3, 1> robotComRpyRate,
    Eigen::Matrix<float, 3, 1> desiredComPosition,
    Eigen::Matrix<float, 3, 1> desiredComVelocity,
    Eigen::Matrix<float, 3, 1> desiredComAngularVelocity,
    Eigen::Matrix<float, 3, 4> contactForces,
    std::vector<map<int, float>> motorTorques4)
{
    Eigen::Matrix<float, 4, 1> contacts;
    Eigen::Matrix<float, 6, 1> robotQ;
    Eigen::Matrix<float, 6, 1> robotDq;
    Eigen::Matrix<float, 3, 1> desiredComRpy(0.0, 0.0, 0.0);
    Eigen::Matrix<float, 6, 1> desiredQ;
    Eigen::Matrix<float, 6, 1> desiredDq;
    // Actual q and dq
    for (int legId = 0; legId < 4; legId++) {
        int legState = gaitGenerator->desiredLegState[legId];
        if (legState == LegState::STANCE || legState == LegState::EARLY_CONTACT) {
            contacts[legId] = true;
        } else {
            contacts[legId] = false;
        }
    }
    robotQ << robotComPosition,
        robotComRpy;
    robotDq << robotComVelocity,
        robotComRpyRate;

    desiredQ << desiredComPosition,
        desiredComRpy;
    desiredDq << desiredComVelocity,
        desiredComAngularVelocity;

    Eigen::Matrix<float, 6, 1> desiredDdq;
    desiredDdq = KP.cwiseProduct(desiredQ - robotQ) + KD.cwiseProduct(desiredDq - robotDq);
    for (int i = 0; i < 6; ++i) {
        if (desiredDdq(i, 0) > this->maxDdq(i, 0)) {
            desiredDdq(i, 0) = this->maxDdq(i, 0);
        }
        if (desiredDdq(i, 0) < this->minDdq(i, 0)) {
            desiredDdq(i, 0) = this->minDdq(i, 0);
        }
    }
    std::map<int, MotorCommand> action;
    for (int legId = 0; legId < 4; legId++) {
        for (auto it = motorTorques4[legId].begin(); it != motorTorques4[legId].end(); ++it) {
            MotorCommand temp{0., 0., 0., 0., it->second};
            auto actionIt = action.find(it->first);
            if (actionIt != action.end()) {
                actionIt->second = temp;
            }
        }
    }
    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce(action, contactForces);
    return actionContactForce;
}

// float TorqueStanceLegController::EstimateRobotHeightTest(Eigen::Matrix<int, 4, 1> contacts,
//                                                          Eigen::Matrix<float, 4, 1> baseOrientation,
//                                                          Eigen::Matrix<float, 3, 3> rotMat,
//                                                          Eigen::Matrix<float, 4, 3> footPositions)
// {
//     Eigen::Matrix<float, 4, 3> footPositionsWorldFrame;
//     Eigen::Matrix<float, 4, 1> usefulHeights;
//     if (contacts.sum() == 0) {
//         return this->desiredBodyHeight;
//     } else {
//         footPositionsWorldFrame = (rotMat * footPositions.transpose()).transpose();
//         usefulHeights = -footPositionsWorldFrame.block<4, 1>(0, 2).cwiseProduct(contacts.cast<float>());
//         return usefulHeights.sum() / contacts.sum();
//     }
// }

} // namespace Quadruped