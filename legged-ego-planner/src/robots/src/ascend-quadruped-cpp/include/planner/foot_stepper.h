/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie;
*           
*/

#ifndef ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_
#define ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <tuple>

#include "robots/robot.h"
#include "utils/se3.h"

namespace Quadruped {
struct Gap {
    float distance; // between COM and center of the gap.
    float width; // of the gap
    Eigen::Matrix<float, 3, 1> startPoint; // the closest point on the gap margin in base frame.

    Gap(float d, float w, Eigen::Matrix<float, 3, 1> p) : distance(d), width(w), startPoint(p)
    {}
};

// todo :different terrains may apply to different FootStepper, using factory method.
class FootStepper {
public:
    FootStepper(TerrainType terrainType, float gapWidth, float defaultFootholdOffset, std::string level);

    void Reset(float timeSinceReset)
    {}

    inline Eigen::Matrix<float, 3, 1> GetDefaultFootholdOffset(int legId)
    {
        return {defaultFootholdDelta, 0.f, 0.f}; // todo : 1 DIM
    }

    /**
     * @brief Find a optimal foot placement for swing legs, usually larger then zero.
     * @param Eigen::Matrix<float, 3, 4> feet positions in world frame when all stance at ground.
     * @note Assuming that foot offset L = L0 + x, gap width is W,
     *         the cost objective is F = x^T * G * x + a^T * x = x^2,
     *          this means we want the increment for default offset to be small.
     *        the constrain inequalities is denoted as :
     *          C^T * x >= b
     *      Case 1: if front leg is possible to meet the gap with default offset,
     *              then x should statifies condition: L0 - d(foot, center of gap) + x >= W/2 or <=-W/2;
     *              This means the front leg either (1.a)walk through the gap or (1.b)not, respectively.
     *              At the mean time, the back legs DO NOT walk over the gap.
     *              To express (1.b) in matrix form,
     *                  [1, -1, -1, -1, -1]^T * x = [x   >= b = [ -L0
     *                                               -x
     *                                               -x           L0-p(gap)+p(foot)
     *                                               -x
     *                                               -x]         ]
     */
    Eigen::Matrix<float, 3, 4> GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds);

protected:
    std::vector<Gap> gaps;
    bool meetGpa; // are any feet meet gap?
    float defaultFootholdDelta;
    Eigen::Matrix<float, 3, 4> nextFootholdsOffset;
    Eigen::Matrix<float, 3, 4> lastFootholdsOffset;
};
} // namespace Quadruped
#endif //ASCEND_QUADRUPED_CPP_INCLUDE_PLANNER_FOOT_STEPPER_H_
