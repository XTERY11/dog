#include <iostream>
#include <yaml-cpp/yaml.h>
#include "../config/types.h"
#include "sim/a1_sim.h"
#include "state_estimator/robot_estimator.h"
#include "mpc_controller/openloop_gait_generator.h"
#include "mpc_controller/raibert_swing_leg_controller.h"
#include "mpc_controller/torque_stance_leg_controller.h"
#include "mpc_controller/locomotion_controller.h"
#include "planner/com_adjuster.h"
#include "planner/local_planner.h"
#include "action/action.h"
#include "ros/cmd_vel_receiver.h"
#include "ros/slam_pose_receiver.h"
#include "ros/robot_odom_estimator.h"

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>

using namespace std;
using namespace Quadruped;

#define MAX_TIME_SECONDS 3000.0f

LocomotionController *setUpController(Robot *quadruped);

void updateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

Eigen::Matrix<float, 3, 1> desiredSpeed = {0.f, 0.f, 0.f};
float desiredTwistingSpeed = 0.f;
float footClearance = 0.01f;

LocomotionController *setUpController(Robot *quadruped) {

    OpenloopGaitGenerator *gaitGenerator = new OpenloopGaitGenerator(quadruped,
                                                                     "config/a1_sim/openloop_gait_generator.yaml");

    cout << "init gaitGenerator finish\n" << endl;

    RobotEstimator *stateEstimator = new RobotEstimator(quadruped, gaitGenerator);

    cout << "init robotEstimator finish\n" << endl;

    ComAdjuster *comAdjuster = new ComAdjuster(quadruped, gaitGenerator, stateEstimator);

    cout << "init comAdjuster finish\n" << endl;

    LocalPlanner *localPlanner = new LocalPlanner(quadruped);

    cout << "init localPlanner finish\n" << endl;

    RaibertSwingLegController *swingLegController = new RaibertSwingLegController(quadruped,
                                                                                  gaitGenerator,
                                                                                  stateEstimator,
                                                                                  localPlanner,
                                                                                  desiredSpeed,
                                                                                  desiredTwistingSpeed,
                                                                                  quadruped->bodyHeight);

    cout << "init swingLegController finish\n" << endl;

    TorqueStanceLegController *stanceLegController = new TorqueStanceLegController(quadruped,
                                                                                   gaitGenerator,
                                                                                   stateEstimator,
                                                                                   comAdjuster,
                                                                                   localPlanner,
                                                                                   desiredSpeed,
                                                                                   desiredTwistingSpeed,
                                                                                   quadruped->bodyHeight,
                                                                                   quadruped->numLegs,
                                                                                   "config/a1_sim/stance_leg_controller.yaml");

    cout << "init stanceLegController finish\n" << endl;

    LocomotionController *locomotionController = new LocomotionController(quadruped,
                                                                          gaitGenerator,
                                                                          stateEstimator,
                                                                          comAdjuster,
                                                                          swingLegController,
                                                                          stanceLegController);

    cout << "init locomotionController finish\n" << endl;

    return locomotionController;
}

void updateControllerParams(LocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed) {
    controller->swingLegController->desiredSpeed = linSpeed;
    controller->swingLegController->desiredTwistingSpeed = angSpeed;
    controller->stanceLegController->desiredSpeed = linSpeed;
    controller->stanceLegController->desiredTwistingSpeed = angSpeed;
}

bool startControllers(ros::NodeHandle &n, std::string serviceName, std::vector<std::string> &controllers_to_start) {
    ros::service::waitForService(serviceName, -1);
    ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switch_controller_msg;
    switch_controller_msg.request.start_controllers = controllers_to_start;
    switch_controller_msg.request.strictness = switch_controller_msg.request.STRICT;
    switch_controller.call(switch_controller_msg);
    return switch_controller_msg.response.ok;
}

int main(int argc, char **argv) {
    YAML::Node mainConfig = YAML::LoadFile("config/a1_sim/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    cout << "!!! twistMode: " << twistMode << endl;
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float>>();

    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();

    ros::init(argc, argv, "a1_sim");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    std::vector<std::string> controllerList = {"joint_state_controller", "FL_hip_controller", "FL_thigh_controller",
                                               "FL_calf_controller", "FR_hip_controller", "FR_thigh_controller",
                                               "FR_calf_controller", "RL_hip_controller", "RL_thigh_controller",
                                               "RL_calf_controller", "RR_hip_controller", "RR_thigh_controller",
                                               "RR_calf_controller"};
    startControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);

    ros::AsyncSpinner spinner(4); // one threads
    spinner.start();
    std::cout << "---------ROS node init finished---------" << std::endl;

    Robot *quadruped = new A1Sim(nh, privateNh, "config/a1_sim/a1_sim.yaml");
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    //    Action::SitDown(quadruped, 3, 0.001);
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);

    LocomotionController *locomotionController = setUpController(quadruped);
    std::cout << "---------LocomotionController Init Finished---------" << std::endl;
    locomotionController->Reset();
    std::cout << "---------LocomotionController Reset Finished---------" << std::endl;

    // ros module init
    RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(quadruped, locomotionController, nh);
    CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    std::cout << "---------ROS Modules Init Finished---------" << std::endl;

    std::cout << "---------TimeSinceReset: " << quadruped->GetTimeSinceReset() << std::endl;

    updateControllerParams(locomotionController, {0., 0., 0.}, 0.);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;

    std::cout << "------------------start control loop------------------ " << std::endl;
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
        if (twistMode == TwistMode::CONST) {
            updateControllerParams(locomotionController,
                                   desiredSpeed,
                                   desiredTwistingSpeed); // const velocity

        } else if (twistMode == TwistMode::ROS) {
            updateControllerParams(locomotionController,
                                   cmdVelReceiver->GetLinearVelocity(),
                                   cmdVelReceiver->GetAngularVelocity()); // ros velocity
                                   
        }

        locomotionController->Update();
        auto[hybridAction, qpSol] = locomotionController->GetAction();
        quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);
        // legOdom->PublishOdometry();

        currentTime = quadruped->GetTimeSinceReset();
        if (abs(quadruped->baseRollPitchYaw[0]) > 0.5f || abs(quadruped->baseRollPitchYaw[1]) > 0.5f) {
            exit(0);
        }

        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    }

    return 0;
}
