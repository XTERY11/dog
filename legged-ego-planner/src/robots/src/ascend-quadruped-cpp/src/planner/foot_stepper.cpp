/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: estimate the vel of quadruped.
* Author: Zhao Yao & Zhu Yijie
* Create: 2021-11-08
* Notes: xx
* Modify: init the file. @ Zhu Yijie
*/

#include "planner/foot_stepper.h"
#include "QuadProg++.hh"
#include "Array.hh"
namespace Quadruped {
FootStepper::FootStepper(TerrainType terrainType, float gapWidth, float defaultFootholdOffset, std::string level)
{
    // terrainType
//    Gap gap0(0.35, gapWidth, {0.35, 0, 0});
//    Gap gap1(1.05, gapWidth, {1.05, 0, 0});
//    Gap gap2(1.8505, gapWidth, {1.8505, 0, 0});
//    Gap gap3(2.45, gapWidth, {2.45, 0, 0});

    Gap gap0(0.42, gapWidth, {0, 0, 0});
    Gap gap1(1.12, gapWidth, {0, 0, 0});

    gaps.push_back(gap0);
    gaps.push_back(gap1);
//    gaps.push_back(gap2);
//    gaps.push_back(gap3);
    defaultFootholdDelta = defaultFootholdOffset; // only alone to the X axis.
    meetGpa = false;
    lastFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    lastFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    nextFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    nextFootholdsOffset.row(0)
        << defaultFootholdOffset, defaultFootholdOffset, defaultFootholdOffset, defaultFootholdOffset; // x DIRECTION
}

Eigen::Matrix<float, 3, 4> FootStepper::GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds)
{
    Eigen::Matrix<float, 1, 4> currentFootholdsX = currentFootholds.row(0);
    std::cout << "currentFootholdsX = " << currentFootholdsX << std::endl;
    Eigen::Matrix<float, 1, 4> defaultNextFootholdsX = currentFootholdsX.array() + defaultFootholdDelta;
    std::cout << "defaultNextFootholdsX = " << defaultNextFootholdsX << std::endl;
    quadprogpp::Matrix<double> G(1, 1);
    G[0][0] = 1.0;
    quadprogpp::Vector<double> a(1);
    a[0] = 0.0;
    quadprogpp::Matrix<double> CE(1, 0);
    quadprogpp::Vector<double> e(0);
    quadprogpp::Matrix<double> CI(1, 5);
    CI[0][0] = 1.0;
    for (int i = 1; i < 5; ++i) {
        CI[0][i] = -1.0;
    }
    quadprogpp::Vector<double> b(5);
    b[0] = -defaultFootholdDelta;
    quadprogpp::Vector<double> x(1);
    x[0] = 0.0;
    for (Gap &gap: gaps) {
        float deltaX = 0.0;
        for (int legId = 0; legId < 4; ++legId) {
            float defaultNextFootholdX = defaultNextFootholdsX[legId];
            if (std::abs(defaultNextFootholdX - gap.distance) <= gap.width / 2) {
                printf("meet gap!, gap = %f, legId = %d \n", gap.distance, legId);
                std::cout << "distance between defaultNextFootholdX and gap: " << gap.distance - defaultNextFootholdX
                          << std::endl;
                float stepDeltaX = 0.f;
                if (legId <= 1) { // front leg meet the gap
                    // may cause exception/crash .
                    // front no, back no
                    std::cout << "Case 1 -=-=-=-=-=-=-=-==-=-" << std::endl;
                    try {
                        for (int i = 1; i < 5; ++i) {
                            CI[0][i] = -1.0;
                            b[i] = defaultFootholdDelta - (gap.distance - gap.width / 2.0) + currentFootholdsX[i - 1];
                        }
                        quadprogpp::solve_quadprog(G, a, CE, e, CI, -b, x);
                        deltaX = x[0];
                        stepDeltaX = defaultFootholdDelta + deltaX;
                        Eigen::Matrix<float, 1, 4> stepDeltaXList = {stepDeltaX, stepDeltaX, stepDeltaX, stepDeltaX};
                        nextFootholdsOffset.row(0) = stepDeltaXList;
                    } catch (std::overflow_error) {
                        std::cout << "overflow_error when compute foothold" << std::endl;
                        nextFootholdsOffset(0, legId) = 0.f;
                    } //catch(...) {
                    //     std::cout << "unknow exception" << std::endl;
                    //     nextFootholdsOffset(0, legId) = 0.f;
                    // }

                    // front yes, back no
                    std::cout << "Case 2 -=-=-=-=-=-=-=-==-=-" << std::endl;
                    try {
                        for (int i = 1; i < 5; ++i) {
                            if (i <= 2) {
                                CI[0][i] = 1.0;
                            } else {
                                CI[0][i] = -1.0;
                            }
                            b[i] = -CI[0][i] * (defaultFootholdDelta - (gap.distance + gap.width / 2.0 * CI[0][i])
                                + currentFootholdsX[i - 1]);
                        }
                        quadprogpp::solve_quadprog(G, a, CE, e, CI, -b, x);
                        if (abs(x[0]) < abs(deltaX)) {
                            deltaX = x[0];
                            stepDeltaX = defaultFootholdDelta + deltaX;
                            Eigen::Matrix<float, 1, 4>
                                stepDeltaXList = {stepDeltaX, stepDeltaX, stepDeltaX, stepDeltaX};
                            nextFootholdsOffset.row(0) = stepDeltaXList;
                            printf("front yes, back no\n");
                        } else {
                            printf("front no, back no\n");
                        }
                    } catch (std::overflow_error) {
                        throw;
                    }
                } else { // back leg meet gap
                    // front yes, back no
                    std::cout << "Case 3 -=-=-=-=-=-=-=-==-=-" << std::endl;
                    try {
                        for (int i = 1; i < 5; ++i) {
                            if (i <= 2) {
                                CI[0][i] = 1.0;
                            } else {
                                CI[0][i] = -1.0;
                            }
                            b[i] = -CI[0][i] * (defaultFootholdDelta - (gap.distance + gap.width / 2.0 * CI[0][i])
                                + currentFootholdsX[i - 1]);
                        }
                        quadprogpp::solve_quadprog(G, a, CE, e, CI, -b, x);
                        deltaX = x[0];
                        stepDeltaX = defaultFootholdDelta + deltaX;
                        Eigen::Matrix<float, 1, 4> stepDeltaXList = {stepDeltaX, stepDeltaX, stepDeltaX, stepDeltaX};
                        nextFootholdsOffset.row(0) = stepDeltaXList;
                    } catch (std::overflow_error) {
                        std::cout << "overflow_error when compute foothold" << std::endl;
                        stepDeltaX = 0.f;
                    }

                    // front yes, back yes
                    std::cout << "Case 4 -=-=-=-=-=-=-=-==-=-" << std::endl;
                    try {
                        for (int i = 1; i < 5; ++i) {
                            CI[0][i] = 1.0;
                            b[i] =
                                -1.0 * (defaultFootholdDelta - (gap.distance + gap.width / 2.0)
                                    + currentFootholdsX[i - 1]);
                        }
                        quadprogpp::solve_quadprog(G, a, CE, e, CI, -b, x);

                        if (abs(x[0]) < abs(deltaX)) {
                            deltaX = x[0];
                            stepDeltaX = defaultFootholdDelta + deltaX;
                            Eigen::Matrix<float, 1, 4>
                                stepDeltaXList = {stepDeltaX, stepDeltaX, stepDeltaX, stepDeltaX};
                            nextFootholdsOffset.row(0) = stepDeltaXList;
                            printf("front yes, back yes\n");
                        } else {
                            printf("front yes, back no\n");
                        }
                        nextFootholdsOffset(0, legId) = stepDeltaX;
                    } catch (std::overflow_error) {
                        printf("overflow error!\n");
                    }
                }
                std::cout << "nextFootholdsOffset = \n" << nextFootholdsOffset << std::endl;
                return nextFootholdsOffset;
            }
        }
    }
    printf("no gap!\n");
    nextFootholdsOffset.row(0) <<
                               defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta;
    return nextFootholdsOffset;
}
} // namespace Quadruped
