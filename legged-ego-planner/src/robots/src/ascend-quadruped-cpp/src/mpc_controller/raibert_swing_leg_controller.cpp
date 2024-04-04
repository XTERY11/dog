/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Swing Leg Controller
* Author: Xie Ming Cheng & Zhao Yao
* Create: 2021-10-25
* Notes: xx
* Modify: add head comment and add some function comments and delete some test functions. @ xie_mingcheng 2021.11.22;
*       add position mode control API for foothold. @ Zhu Yijie 2021.11.24;
*/

#include "mpc_controller/raibert_swing_leg_controller.h"

using namespace Eigen;
using namespace std;
namespace Quadruped {
// The position correction coefficients in Raibert's formula.
const Matrix<float, 3, 1> swingKp(0.03, 0.03, 0.03);

//At the end of swing, we leave a small clearance to prevent unexpected foot collision.
#define FOOT_CLEARANCE_M = 0.01

float RaibertSwingLegController::GenParabola(float phase, float start, float mid, float end)
{
    float mid_phase = 0.5;
    float deltaOne, deltaTwo, deltaThree, coefa, coefb, coefc;
    deltaOne = mid - start;
    deltaTwo = end - start;
    deltaThree = pow(mid_phase, 2) - mid_phase;
    coefa = (deltaOne - deltaTwo * mid_phase) / deltaThree;
    coefb = (deltaTwo * pow(mid_phase, 2) - deltaOne) / deltaThree;
    coefc = start;
    return coefa * pow(phase, 2) + coefb * phase + coefc;
}

Matrix<float, 3, 1> RaibertSwingLegController::GenSwingFootTrajectory(float inputPhase,
                                                                      Matrix<float, 3, 1> startPos,
                                                                      Matrix<float, 3, 1> endPos)
{
    float phase = inputPhase;
    if (inputPhase <= 0.5) {
        phase = 0.8 * sin(inputPhase * M_PI);
    } else {
        phase = 0.8 + (inputPhase - 0.5) * 0.4;
    }
    float x, y, maxClearance, mid, z;
    x = (1 - phase) * startPos(0, 0) + phase * endPos(0, 0);
    y = (1 - phase) * startPos(1, 0) + phase * endPos(1, 0);
    maxClearance = 0.1;
    mid = max(endPos(2, 0), startPos(2, 0)) + maxClearance;
    z = GenParabola(phase, startPos(2, 0), mid, endPos(2, 0));
    return Matrix<float, 3, 1>(x, y, z);
}

RaibertSwingLegController::RaibertSwingLegController(Robot *robot,
                                                     OpenloopGaitGenerator *gaitGenerator,
                                                     RobotEstimator *stateEstimator,
                                                     LocalPlanner *localPlanner,
                                                     Matrix<float, 3, 1> desiredSpeed,
                                                     float desiredTwistingSpeed,
                                                     float desiredHeight,
                                                     float footClearance)
{
    this->robot = robot;
    this->gaitGenerator = gaitGenerator;
    this->stateEstimator = stateEstimator;
    this->localPlanner = localPlanner;
    this->lastLegState = gaitGenerator->desiredLegState;
    this->desiredSpeed = desiredSpeed;
    this->desiredTwistingSpeed = desiredTwistingSpeed;
    this->desiredHeight = Matrix<float, 3, 1>(0, 0, desiredHeight - footClearance);
//    this->Reset(0);
}

void RaibertSwingLegController::Reset(float currentTime)
{
    lastLegState = gaitGenerator->desiredLegState;
    phaseSwitchFootLocalPos = robot->GetFootPositionsInBaseFrame();

    if (robot->controlParams["mode"] == LocomotionMode::POSITION_LOCOMOTION) {
        phaseSwitchFootGlobalPos = robot->GetFootPositionsInWorldFrame();
//        footHoldInWorldFrame = phaseSwitchFootGlobalPos; //todo reset by default foot pose setting
        footHoldInWorldFrame.row(0) << 0.185f, 0.185f, -0.175f, -0.175f;
        footHoldInWorldFrame.row(1) << -0.145f, 0.145f, -0.145f, 0.145f;
        footHoldInWorldFrame.row(2) << 0.f, 0.f, 0.f, 0.f;
        footHoldInWorldFrame(0,0) -= 0.05;
        footHoldInWorldFrame(0,3) -= 0.05;
        std::cout << "[SwingLegController Reset] phaseSwitchFootLocalPos: \n" << phaseSwitchFootLocalPos << std::endl;
        std::cout << "[SwingLegController Reset] phaseSwitchFootGlobalPos: \n" << phaseSwitchFootGlobalPos << std::endl;
        std::cout << "[SwingLegController Reset] footHoldInWorldFrame: \n" << footHoldInWorldFrame << std::endl;
    } // LocomotionMode::POSITION_LOCOMOTION

    swingJointAnglesVelocities.clear();
}

void RaibertSwingLegController::Update(float currentTime)
{
    Matrix<int, 4, 1> newLegState = gaitGenerator->desiredLegState;

    // Detects phase switch for each leg so we can remember the feet position at
    // the beginning of the swing phase.
    for (int legId = 0; legId < newLegState.size(); legId++) {
        if (newLegState(legId) == LegState::SWING && newLegState(legId) != lastLegState(legId)) {
            phaseSwitchFootLocalPos.col(legId) = robot->GetFootPositionsInBaseFrame().col(legId);
            // cout << "-------------------------------------------------------------------------" << endl;
            // std::cout << "[SwingLegController::Update] LegState Change \n" << std::endl;
            // std::cout << "[SwingLegController::Update] legId: " << legId << std::endl;
            // std::cout << "[SwingLegController::Update] phaseSwitchFootLocalPos.col(legId): \n"
            //           << phaseSwitchFootLocalPos.col(legId) << std::endl;
            if (robot->controlParams["mode"] == LocomotionMode::POSITION_LOCOMOTION) {
                phaseSwitchFootGlobalPos.col(legId) = robot->GetFootPositionsInWorldFrame().col(legId);
                std::cout << "[SwingLegController::Update] phaseSwitchFootGlobalPos.col(legId): \n"
                          << phaseSwitchFootGlobalPos.col(legId) << std::endl;
                std::cout << "[SwingLegController::Update] footHoldInWorldFrame.col(legId): \n"
                          << footHoldInWorldFrame.col(legId) << std::endl;
                if (legId == 0) { //update four leg footholds
                    static int stepCount = 0;
                    stepCount++;
                    cout << "step_count: " << stepCount << endl;
                    localPlanner->UpdateOnce(footHoldInWorldFrame); // based on the last foothold position
                }
                Eigen::Matrix<float, 3, 1> constOffset = {0.10f, 0.f, 0.f};
//                footHoldInWorldFrame.col(legId) += constOffset;
                footHoldInWorldFrame.col(legId) +=
                    localPlanner->GetFootholdsOffset().col(legId); // get the foothold offset of this leg by id
                cout << "[SwingLegController::Update] update footHoldInWorldFrame: \n"
                     << footHoldInWorldFrame.col(legId) << endl;
            } // LocomotionMode::POSITION_LOCOMOTION

        }
    }
    lastLegState = newLegState;
}

map<int, Matrix<float, 5, 1>> RaibertSwingLegController::GetAction()
{
    Matrix<float, 3, 1> comVelocity;
    Matrix<float, 3, 1> hipOffset;
    Matrix<float, 3, 1> twistingVector;
    Matrix<float, 3, 1> hipHorizontalVelocity;
    Matrix<float, 3, 1> targetHipHorizontalVelocity;
    Matrix<float, 3, 1> footTargetPosition;
    Matrix<float, 3, 1> footPositionInBaseFrame;
    Matrix<float, 3, 1> footPositionInWorldFrame;
    Matrix<int, 3, 1> jointIdx;
    Matrix<float, 3, 1> jointAngles;
    Matrix<float, 3, 4> hipPositions;
    float yawDot;
    comVelocity = stateEstimator->GetEstimatedVelocity();
    yawDot = robot->GetBaseRollPitchYawRate()(2, 0);
    hipPositions = robot->GetHipPositionsInBaseFrame();
    for (int legId = 0; legId < gaitGenerator->legState.size(); legId++) {
        int tempState = gaitGenerator->legState[legId];
        if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
            continue;
        }

        switch (robot->controlParams["mode"]) {
            case LocomotionMode::VELOCITY_LOCOMOTION:

                hipOffset = hipPositions.col(legId);
                twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
                hipHorizontalVelocity = comVelocity + yawDot * twistingVector;
                targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector;

                footTargetPosition = hipHorizontalVelocity * gaitGenerator->stanceDuration[legId] / 2 -
                    swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity) - desiredHeight +
                    Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0);

                footPositionInBaseFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                 phaseSwitchFootLocalPos.col(legId),
                                                                 footTargetPosition);
                break;
            case LocomotionMode::POSITION_LOCOMOTION:
                footPositionInWorldFrame = GenSwingFootTrajectory(gaitGenerator->normalizedPhase[legId],
                                                                  phaseSwitchFootGlobalPos.col(legId),
                                                                  footHoldInWorldFrame.col(legId)); // interpolation in world frame
                footPositionInBaseFrame = robotics::math::RigidTransform(robot->basePosition,
                                                                         robot->baseOrientation,
                                                                         footPositionInWorldFrame); // transfer to base frame
                break;
            default:throw std::domain_error("controlParams[mode] is not correct!\n");
        }

        robot->ComputeMotorAnglesFromFootLocalPosition(legId, footPositionInBaseFrame, jointIdx, jointAngles);

        for (int i = 0; i < numMotorOfOneLeg; ++i) {
            swingJointAnglesVelocities[jointIdx[i]] = {jointAngles[i], 0.f, legId};  //std::tuple<float, float, int>()
        }
    }
    map<int, Matrix<float, 5, 1>> actions;
    Matrix<float, 12, 1> kps, kds;
    kps = robot->GetMotorPositionGains();
    kds = robot->GetMotorVelocityGains();
    for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int &singleLegId = std::get<2>(posVelId);
        if (gaitGenerator->desiredLegState(singleLegId, 0) == LegState::SWING) {
            actions[it->first] << std::get<0>(posVelId), kps[it->first], std::get<1>(posVelId), kds[it->first], 0.f;
        }
    }
    return actions;
}

tuple<map<int, Eigen::Matrix<float, 5, 1>>, Eigen::Matrix<float, 3, 1>, Eigen::Matrix<float, 3, 1>>
RaibertSwingLegController::
GetActionTest(Eigen::Matrix<float, 3, 1> testComVelocity, float testYawDot,
              Eigen::Matrix<float, 3, 4> testHipPositions,
              Eigen::Matrix<int, 3, 1> testJointIdx,
              Eigen::Matrix<float, 3, 1> testJointAngles,
              Eigen::Matrix<int, 4, 1> testLegState,
              Eigen::Matrix<float, 4, 1> testStanceDuration,
              Eigen::Matrix<float, 4, 1> testPhase,
              Eigen::Matrix<float, 3, 4> testPhaseSwitchFootLocalPos,
              Eigen::Matrix<float, 12, 1> testKps,
              Eigen::Matrix<float, 12, 1> testKds)
{
    Matrix<float, 3, 1> hipOffset;
    Matrix<float, 3, 1> twistingVector;
    Matrix<float, 3, 1> hipHorizontalVelocity;
    Matrix<float, 3, 1> targetHipHorizontalVelocity;
    Matrix<float, 3, 1> footTargetPosition;
    Matrix<float, 3, 1> footPosition;
    map<int, Matrix<float, 5, 1>> actions;

    for (int legId = 0; legId < testLegState.size(); legId++) {
        int tempState = testLegState[legId];
        // do test
        if (tempState > 0) {
            break;
        }
        if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
            continue;
        }
        hipOffset = testHipPositions.col(legId);
        twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
        hipHorizontalVelocity = testComVelocity + testYawDot * twistingVector;
        targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector;
        footTargetPosition = hipHorizontalVelocity * testStanceDuration[legId] / 2 -
            swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity) - desiredHeight +
            Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0);
        footPosition = GenSwingFootTrajectory(testPhase[legId], testPhaseSwitchFootLocalPos.col(legId),
                                              footTargetPosition);
        for (int i = 0; i < numMotorOfOneLeg; ++i) {
            swingJointAnglesVelocities[testJointIdx[i]] =
                {testJointAngles[i], 0.f, legId};  //std::tuple<float, float, int>()
        }
    }
    for (auto it = swingJointAnglesVelocities.begin(); it != swingJointAnglesVelocities.end(); ++it) {
        const std::tuple<float, float, int> &posVelId = it->second;
        const int &singleLegId = std::get<2>(posVelId);
        if (testLegState(singleLegId, 0) == LegState::SWING) {
            actions[it->first]
                << std::get<0>(posVelId), testKps[it->first], std::get<1>(posVelId), testKds[it->first], 0.f;
        }
    }
    return {actions, footPosition, footTargetPosition};
}

tuple<map<int, Eigen::Matrix<float, 5, 1>>, Eigen::Matrix<float, 3, 1>, Eigen::Matrix<float, 3, 1>>
RaibertSwingLegController::TestGetAction(Matrix<float, 3, 1> testComVelocity, float testYawDot,
                                         Matrix<float, 3, 4> testHipPositions, Matrix<int, 3, 1> testJointIdx,
                                         Matrix<float, 3, 1> testJointAngles, Eigen::Matrix<int, 4, 1> testLegState,
                                         Eigen::Matrix<float, 4, 1> testStanceDuration,
                                         Eigen::Matrix<float, 4, 1> testPhase,
                                         Matrix<float, 3, 4> testPhaseSwitchFootLocalPos, Matrix<float, 12, 1> testKps,
                                         Matrix<float, 12, 1> testKds)
{

    Matrix<float, 3, 1> hipOffset;
    Matrix<float, 3, 1> twistingVector;
    Matrix<float, 3, 1> hipHorizontalVelocity;
    Matrix<float, 3, 1> targetHipHorizontalVelocity;
    Matrix<float, 3, 1> footTargetPosition;
    Matrix<float, 3, 1> footPosition;
    map<int, Matrix<float, 5, 1>> action;

    for (int legId = 0; legId < testLegState.size(); legId++) {
        int tempState = testLegState[legId];
        // do test
        if (tempState > 0) {
            break;
        }
        if (tempState == LegState::STANCE || tempState == LegState::EARLY_CONTACT) {
            continue;
        }
        hipOffset = testHipPositions.col(legId);
        twistingVector = Matrix<float, 3, 1>(-hipOffset[1], hipOffset[0], 0);
        hipHorizontalVelocity = testComVelocity + testYawDot * twistingVector;
        targetHipHorizontalVelocity = desiredSpeed + desiredTwistingSpeed * twistingVector;
        footTargetPosition = hipHorizontalVelocity * testStanceDuration[legId] / 2 -
            swingKp.cwiseProduct(targetHipHorizontalVelocity - hipHorizontalVelocity) - desiredHeight +
            Matrix<float, 3, 1>(hipOffset[0], hipOffset[1], 0);
        footPosition = GenSwingFootTrajectory(testPhase[legId], testPhaseSwitchFootLocalPos.col(legId),
                                              footTargetPosition);
        for (int i = 0; i < testJointIdx.size(); i++) {
            swigJointAngles[testJointIdx[i]] = testJointAngles[i];
            if (tempState == LegState::SWING) {
                action[testJointIdx[i]][0] = testJointAngles[i];
                action[testJointIdx[i]][1] = testKps[testJointIdx[i]];
                action[testJointIdx[i]][2] = 0;
                action[testJointIdx[i]][3] = testKds[testJointIdx[i]];
                action[testJointIdx[i]][4] = 0;
            }
        }
    }
    return {action, footPosition, footTargetPosition};
}
} // namespace Quadruped