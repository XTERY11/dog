/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Actions
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19;
*        add KeepStand @ Zhu Yijie 2021.11.23
*/

#include "action/action.h"

namespace Quadruped {
namespace Action {

    void StandUp(Robot *robot, float standUpTime, float totalTime, float timeStep)
    {
        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + standUpTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeStandUP = robot->GetMotorAngles();
        std::cout << "motorAnglesBeforeStandUP: \n" << motorAnglesBeforeStandUP.transpose() << std::endl;
        std::cout << "---------------------Standing Up---------------------" << std::endl;
        std::cout << "robot->standMotorAngles: \n" << robot->standUpMotorAngles.transpose() << std::endl;
        for (float t = startTime; t < totalTime; t += timeStep) {
            float blendRatio = (t - startTime) / standUpTime;
            Eigen::Matrix<float, 12, 1> action;
            if (blendRatio < 1.0f) {
                action = blendRatio * robot->standUpMotorAngles + (1 - blendRatio) * motorAnglesBeforeStandUP;
                robot->Step(action, MotorMode::POSITION_MODE);
                while (robot->GetTimeSinceReset() < t + timeStep) {}
            } else {
                robot->Step(action, MotorMode::POSITION_MODE);
                while (robot->GetTimeSinceReset() < t + timeStep) {}
            }
        }
        std::cout << "robot->GetMotorAngles: \n" << robot->GetMotorAngles().transpose() << std::endl;
        std::cout << "---------------------Stand Up Finished---------------------" << std::endl;
    }

    void SitDown(Robot *robot, float sitDownTime, float timeStep)
    {

        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + sitDownTime;
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeSitDown = robot->GetMotorAngles();
        std::cout << "motorAnglesBeforeSitDown: \n" << motorAnglesBeforeSitDown.transpose() << std::endl;
        std::cout << "robot->sitDownMotorAngles: \n" << robot->sitDownMotorAngles.transpose() << std::endl;

        for (float t = startTime; t < endTime; t += timeStep) {
            float blendRatio = (t - startTime) / sitDownTime;
            Eigen::Matrix<float, 12, 1> action;
            action = blendRatio * robot->sitDownMotorAngles + (1 - blendRatio) * motorAnglesBeforeSitDown;
            robot->Step(action, MotorMode::POSITION_MODE);
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        }
    }

        void KeepStand(Robot *robot, float KeepStandTime, float timeStep)
    {
        float startTime = robot->GetTimeSinceReset();
        float endTime = startTime + KeepStandTime;
        // record current motor angles.
        Eigen::Matrix<float, 12, 1> motorAnglesBeforeKeepStand = robot->standUpMotorAngles; // robot->GetMotorAngles();
        
        for (float t = startTime; t < endTime; t += timeStep) {
            robot->Step(motorAnglesBeforeKeepStand, MotorMode::POSITION_MODE);
            while (robot->GetTimeSinceReset() < t + timeStep) {}
        }
    }
    } // Action
} // Quadruped