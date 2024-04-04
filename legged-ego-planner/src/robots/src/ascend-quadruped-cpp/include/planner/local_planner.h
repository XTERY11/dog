/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_LOCAL_PLANNER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_LOCAL_PLANNER_H_

#include "planner/foot_stepper.h"
namespace Quadruped {
class LocalPlanner {
public:

    LocalPlanner(Robot *robotIn);

    ~LocalPlanner() = default;

    void Reset();

    void Update()
    {}

    void UpdateOnce(Eigen::Matrix<float, 3, 4> currentFootholds,
                    Eigen::Matrix<float, 6, 1> currentComPose = Eigen::Matrix<float, 6, 1>::Zero());

    inline bool LoadTerrain(Eigen::MatrixXf terrian)
    {
        if (terrainType == TerrainType::PLUM_PILES) {
            costMap = terrian.block<N, N>(0, 0);
        }
        return true;
    }

    Eigen::Matrix<float, 3, 1> ComputeNextFootholdOffset(Eigen::Matrix<float, 3, 1> currentFoothold, int legId);

    Eigen::Matrix<float, 3, 4> ComputeNextFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

    Eigen::Matrix<float, 3, 4> ComputeFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds,
                                                      Eigen::Matrix<float, 6, 1> currentComPose,
                                                      Eigen::Matrix<float, 6, 1> desiredComPose);

    inline Eigen::Matrix<float, 6, 1> GetComPose()
    {
        comPose << robot->GetBasePosition(), robot->GetBaseRollPitchYaw();
        return comPose;
    }

    inline const Eigen::Matrix<float, 6, 1> &GetDesiredComPose() const
    {
        return desiredComPose;
    }

    inline const Eigen::Matrix<float, 3, 4> &GetFootholdsOffset() const
    {
        return desiredFootholdsOffset;
    }

    inline Eigen::Matrix<float, 6, 1> GetComGoal(Eigen::Matrix<float, 6, 1> currentComPose)
    {
        desiredComPose << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
        return desiredComPose;
    }

protected:

    Robot *robot;
    FootStepper footholdPlanner;

    float resetTime;
    float timeSinceReset;

    TerrainType terrainType;
    constexpr static int N = 50; // default map size
    Eigen::Matrix<float, N, N> costMap;

    long long unsigned stepCount;
    Eigen::Matrix<float, 6, 1> comPose;
    Eigen::Matrix<float, 6, 1> desiredComPose;
    Eigen::Matrix<float, 3, 4> desiredFootholdsOffset;

};
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_LOCAL_PLANNER_H_
