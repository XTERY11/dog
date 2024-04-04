/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: seing foot tarjectory generator.
* Author: Zhu Yijie
* Create: 2021-11-20
* Notes: this is for swing foot.
* Modify: init the file. @ Zhu Yijie;
*/

#include "mpc_controller/foot_trajectory_generator.h"

namespace Quadruped {
    FootSplinePatternGenerator::FootSplinePatternGenerator() : initial_time_(0.), duration_(0.) {}

    FootSplinePatternGenerator::~FootSplinePatternGenerator() {}

    void FootSplinePatternGenerator::SetParameters(const float initial_time,
                                                const Eigen::Vector3f& initial_pos,
                                                const Eigen::Vector3f& target_pos,
                                                const StepParameters& params)
    {
        // Setting the initial time and duration of the swing movements
        initial_time_ = initial_time;
        duration_ = params.duration;

        // Computing the appex of the swing movement
        Eigen::Vector3f step_delta = target_pos - initial_pos;
        float height_dist = fabs((float) step_delta(2));
        float step2d_dist = fabs(step_delta.head<2>().norm());
        float step_theta = atan(height_dist / step2d_dist);
        float target_appex;
        if (target_pos(2) >= initial_pos(2)) {
            target_appex = target_pos(2) + params.height * cos(step_theta);
        } else {
            target_appex = initial_pos(2) + params.height * cos(step_theta);
        }
        // Setting the spline boundaries
        foot_spliner_x_.setBoundary(initial_time,
                                    params.duration,
                                    initial_pos(0),
                                    target_pos(0));
        foot_spliner_y_.setBoundary(initial_time,
                                    params.duration,
                                    initial_pos(1),
                                    target_pos(1));
        foot_spliner_up_z_.setBoundary(initial_time,
                                    params.duration/2.0f,
                                    initial_pos(2),
                                    target_appex);
        foot_spliner_down_z_.setBoundary(initial_time + params.duration / 2.0f,
                                        params.duration/2.0f,
                                        target_appex,
                                        target_pos(2)-params.penetration);
    }


    bool FootSplinePatternGenerator::GenerateTrajectory(Vec3<float>& foot_pos,
                                                        Vec3<float>& foot_vel,
                                                        Vec3<float>& foot_acc,
                                                        const float& time)
    {
        if (time < initial_time_ - 1e-3)  // the float number is not exactly representation, so add 1e-3.
            return false; // duration it's always positive, and makes sense when
                        // is bigger than the sample time
        // Computing the time that allows us to discriminate the swing-up or swing-down phase
        robotics::math::Spline::Point swing_traj_x, swing_traj_y, swing_traj_z;
        float dt = time - initial_time_;
        foot_spliner_x_.getPoint(time, swing_traj_x);
        foot_spliner_y_.getPoint(time, swing_traj_y);

        if (dt <= (duration_ / 2.0f))
            foot_spliner_up_z_.getPoint(time, swing_traj_z);
        else
            foot_spliner_down_z_.getPoint(time, swing_traj_z);

        // Setting the foot state
        foot_pos << swing_traj_x.x, swing_traj_y.x, swing_traj_z.x;
        foot_vel << swing_traj_x.xd, swing_traj_y.xd, swing_traj_z.xd;
        foot_acc << swing_traj_x.xdd, swing_traj_y.xdd, swing_traj_z.xdd;

        if (time >= initial_time_ + duration_ + 1e-3)
            return false;

        return true;
    }



    SwingFootTrajectory::SwingFootTrajectory(Vec3<float> startPosIn, 
                                            Vec3<float> endPosIn, 
                                            float duration,
                                            float maxClearance)
        : startPos(startPosIn), endPos(endPosIn), stepParams(duration, 0., 0.)
    {
        mid = std::max(endPos[2], startPos[2]) + maxClearance;
        // stepParams = StepParameters(duration, mid, 0.);
        stepParams.height = maxClearance;
        footTarjGen.SetParameters(0., startPos, endPos, stepParams);
    }

    SwingFootTrajectory::SwingFootTrajectory(const SwingFootTrajectory& item)
    {
        mid = item.mid;
        stepParams = item.stepParams;
        footTarjGen.SetParameters(0., item.startPos, item.endPos, stepParams);
    }

    SwingFootTrajectory::~SwingFootTrajectory() {}

    bool SwingFootTrajectory::GenerateTrajectoryPoint(Vec3<float>& footPos,
                                                    Vec3<float>& footV,
                                                    Vec3<float>& footA,
                                                    float t,
                                                    bool phaseModule)
    {   
        float inputPhase = t;  // 0<=t<=1
        float phase;
        if (phaseModule) {
            if (inputPhase <= 0.5) {
                phase = 0.8 * std::sin(inputPhase * M_PI);
            } else {
                phase = 0.8 + (inputPhase - 0.5) * 0.4;
            }
        } else {
            phase = inputPhase;
        }
        bool flag = footTarjGen.GenerateTrajectory(footPos, footV, footA, phase);
        
        return flag; // return # p,v,a
    }

} // Quadruped