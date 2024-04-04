/* 
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.        
* Description: inherited from quadruped robot, name as GO1.
* Author: Zhu Yijie
* Create: 2021-12-09
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*/

#include "robots/robot_go1.h"

namespace Quadruped {
    RobotGO1::RobotGO1(std::string configFilePathIn) {
        configFilePath = configFilePathIn;
        std::cout << configFilePath << std::endl; 
        robotConfig = YAML::LoadFile(configFilePath);
        std::cout << "success load go1 robot config file !"<<std::endl;
        robotName = robotConfig["name"].as<std::string>();

        bodyMass = robotConfig["robot_params"]["body_mass"].as<float>();
        std::vector<float> bodyInertiaList = robotConfig["robot_params"]["body_inertia"].as<std::vector<float>>();
        bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaList[0], 3, 3);
        bodyHeight = robotConfig["robot_params"]["body_height"].as<float>();

        hipLength = robotConfig["robot_params"]["hip_l"].as<float>();
        upperLegLength = robotConfig["robot_params"]["upper_l"].as<float>();
        lowerLegLength = robotConfig["robot_params"]["lower_l"].as<float>();


        std::vector<float> comOffsetList = robotConfig["robot_params"]["com_offset"].as<std::vector<float>>();
        comOffset = -Eigen::MatrixXf::Map(&comOffsetList[0], 3, 1);

        std::vector<std::vector<float>> hipOffsetList = robotConfig["robot_params"]["hip_offset"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix<float, 3, 1> hipOffsetFR = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1) + comOffset;
        Eigen::Matrix<float, 3, 1> hipOffsetFL = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1) + comOffset;
        Eigen::Matrix<float, 3, 1> hipOffsetRL = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1) + comOffset;
        Eigen::Matrix<float, 3, 1> hipOffsetRR = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1) + comOffset;
        hipOffset << hipOffsetFR, hipOffsetFL, hipOffsetRL, hipOffsetRR;

        std::vector<std::vector<float>> defaultHipPositionList = robotConfig["robot_params"]["default_hip_positions"].as<std::vector<std::vector<float>>>();
        Eigen::Matrix<float, 3, 1> defaultHipPositionFR =  Eigen::MatrixXf::Map(&defaultHipPositionList[0][0], 3, 1);
        Eigen::Matrix<float, 3, 1> defaultHipPositionFL =  Eigen::MatrixXf::Map(&defaultHipPositionList[1][0], 3, 1);
        Eigen::Matrix<float, 3, 1> defaultHipPositionRL =  Eigen::MatrixXf::Map(&defaultHipPositionList[2][0], 3, 1);
        Eigen::Matrix<float, 3, 1> defaultHipPositionRR =  Eigen::MatrixXf::Map(&defaultHipPositionList[3][0], 3, 1);
        defaultHipPosition << defaultHipPositionFR, defaultHipPositionFL, defaultHipPositionRL, defaultHipPositionRR;


        float abadKp, abadKd, hipKp, hipKd, kneeKp, kneeKd;
        abadKp = robotConfig["motor_params"]["abad_p"].as<float>();
        abadKd = robotConfig["motor_params"]["abad_d"].as<float>();
        hipKp = robotConfig["motor_params"]["hip_p"].as<float>();
        hipKd = robotConfig["motor_params"]["hip_d"].as<float>();
        kneeKp = robotConfig["motor_params"]["knee_p"].as<float>();
        kneeKd = robotConfig["motor_params"]["knee_d"].as<float>();
        Eigen::Matrix<float, 3, 1> kps(abadKp, hipKp, kneeKp);
        Eigen::Matrix<float, 3, 1> kds(abadKd, hipKd, kneeKd);
        motorKps << kps, kps, kps, kps;
        motorKds << kds, kds, kds, kds;

        std::vector<float> jointDirectionList = robotConfig["motor_params"]["joint_directions"].as<std::vector<float>>();
        std::vector<float> jointOffsetList = robotConfig["motor_params"]["joint_offsets"].as<std::vector<float>>();
        jointDirection = Eigen::MatrixXf::Map(&jointDirectionList[0], 12, 1);
        jointOffset = Eigen::MatrixXf::Map(&jointOffsetList[0], 12, 1);

        float standUpAbAngle, standUpHipAngle, standUpKneeAngle;
        standUpAbAngle = robotConfig["robot_params"]["default_standup_angle"]["ab"].as<float>();
        standUpHipAngle = robotConfig["robot_params"]["default_standup_angle"]["hip"].as<float>();
        standUpKneeAngle = robotConfig["robot_params"]["default_standup_angle"]["knee"].as<float>();
        Eigen::Matrix<float, 3, 1> defaultStandUpAngle(standUpAbAngle, standUpHipAngle, standUpKneeAngle);
        standUpMotorAngles << defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle, defaultStandUpAngle;

        float sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle;
        sitDownAbAngle = robotConfig["robot_params"]["default_sitdown_angle"]["ab"].as<float>();
        sitDownHipAngle = robotConfig["robot_params"]["default_sitdown_angle"]["hip"].as<float>();
        sitDownKneeAngle = robotConfig["robot_params"]["default_sitdown_angle"]["knee"].as<float>();
        Eigen::Matrix<float, 3, 1> defaultSitDownAngle(sitDownAbAngle, sitDownHipAngle, sitDownKneeAngle);
        sitDownMotorAngles << defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle, defaultSitDownAngle;

        controlParams["mode"] = robotConfig["contorller_params"]["mode"].as<int>(); //types.h: enum 

        timeStep = 0.001;
        this->ResetTimer();
        lastResetTime = GetTimeSinceReset();
        initComplete = true;

        std::cout << "RobotGO1 init Complete" << std::endl;
    
    }

    void RobotGO1::ReceiveObservation()
    {
        LowState state = robotInterface.ReceiveObservation();
        lowState = state;
        
        std::array<float, 4> quaternion;
        std::copy(std::begin(state.imu.quaternion), std::end(state.imu.quaternion), std::begin(quaternion));
        std::array<float, 3> rpy;
        std::copy(std::begin(state.imu.rpy), std::end(state.imu.rpy), std::begin(rpy));
        std::array<float, 3> gyro;
        std::copy(std::begin(state.imu.gyroscope), std::end(state.imu.gyroscope), std::begin(gyro));
        auto & acc = state.imu.accelerometer;
        baseLinearAcceleration << acc[0], acc[1], acc[2];
        // std::array<float, 4> quaternion = state.imu.quaternion;
        // std::array<float, 3> rpy = state.imu.rpy;
        // std::array<float, 3> gyro = state.imu.gyroscope;
        baseOrientation << quaternion[0], quaternion[1], quaternion[2], quaternion[3]; // wxyz
        baseRollPitchYaw << rpy[0], rpy[1], rpy[2];
        baseRollPitchYawRate << gyro[0], gyro[1], gyro[2];
        for (int motorId = 0; motorId < numMotors; motorId++) {
            motorAngles[motorId] = state.motorState[motorId].q;
            motorVelocities[motorId] = state.motorState[motorId].dq;
        }
        std::array<int16_t, 4> force;
        std::copy(std::begin(state.footForce), std::end(state.footForce), std::begin(force));    
        // std::array<int16_t, 4> force = state.footForce;
        footForce << force[0], force[1], force[2], force[3];
        for (int footId=0; footId < numLegs; ++footId) {
            if (footForce[footId] > 170) { // 100, 180,   180, 180
                footContact[footId] = true;
            } else {
                footContact[footId] = false;
            }
        }

    }

    void RobotGO1::ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode)
    {
        std::array<float, 60> motorCommandsArray = {0};
        if (motorControlMode == POSITION_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = motorCommandsShaped[motorId];
                motorCommandsArray[motorId * 5 + 1] = motorKps[motorId];
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = motorKds[motorId];
                motorCommandsArray[motorId * 5 + 4] = 0;
            }
        } else if (motorControlMode == TORQUE_MODE) {
            Eigen::Matrix<float, 1, 12> motorCommandsShaped = motorCommands.transpose();
            for (int motorId = 0; motorId < numMotors; motorId++) {
                motorCommandsArray[motorId * 5] = 0;
                motorCommandsArray[motorId * 5 + 1] = 0;
                motorCommandsArray[motorId * 5 + 2] = 0;
                motorCommandsArray[motorId * 5 + 3] = 0;
                motorCommandsArray[motorId * 5 + 4] = motorCommandsShaped[motorId];
            }
        } else if (motorControlMode == HYBRID_MODE) {
            Eigen::Matrix<float, 5, 12> motorCommandsShaped = motorCommands;
            for (int motorId = 0; motorId < numMotors; motorId++) {     
                motorCommandsArray[motorId * 5] = motorCommandsShaped(POSITION, motorId);
                motorCommandsArray[motorId * 5 + 1] = motorCommandsShaped(KP, motorId);
                motorCommandsArray[motorId * 5 + 2] =
                    motorCommandsShaped(VELOCITY, motorId);
                motorCommandsArray[motorId * 5 + 3] = motorCommandsShaped(KD, motorId);
                motorCommandsArray[motorId * 5 + 4] =
                    motorCommandsShaped(TORQUE, motorId);
            }
        } else {
            // todo
        }

        //    std::cout << "motorCommandsArray:\n" << std::endl;
        //    for (int index = 0; index < motorCommandsArray.size(); index++) {
        //        std::cout << motorCommandsArray[index] << " ";
        //    }
        //    std::cout << std::endl;
        robotInterface.SendCommand(motorCommandsArray);
    }

    void RobotGO1::ApplyAction(const std::vector<MotorCommand> &motorCommands,
                              MotorMode motorControlMode)
    {
        //    std::cout << "apply:\n" << motorCommands << std::endl;
        std::array<float, 60> motorCommandsArray = {0};
        for (int motorId = 0; motorId < numMotors; motorId++) {
            motorCommandsArray[motorId * 5] = motorCommands[motorId].p;
            motorCommandsArray[motorId * 5 + 1] = motorCommands[motorId].Kp;
            motorCommandsArray[motorId * 5 + 2] = motorCommands[motorId].d;
            motorCommandsArray[motorId * 5 + 3] = motorCommands[motorId].Kd;
            motorCommandsArray[motorId * 5 + 4] = motorCommands[motorId].tua;
        }
        robotInterface.SendCommand(motorCommandsArray);
    }

    void RobotGO1::Step(const Eigen::MatrixXf &action,
                       MotorMode motorControlMode)
    {
        ReceiveObservation();
        ApplyAction(action, motorControlMode);
    }

    void RobotGO1::Step(const std::vector<MotorCommand> &motorCommands,
                       MotorMode motorControlMode)
    {
        ReceiveObservation();
        ApplyAction(motorCommands, motorControlMode);
    }

    LowState& RobotGO1::GetLowState()
    {
        return lowState;
    }
} // Quadruped


