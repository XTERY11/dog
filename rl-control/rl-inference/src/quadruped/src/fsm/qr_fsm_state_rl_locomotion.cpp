// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "fsm/qr_fsm_state_rl_locomotion.hpp"

using namespace Quadruped;

extern void UpdateControllerParams(qrLocomotionController *controller, Eigen::Vector3f linSpeed, float angSpeed);

template<typename T>
qrFSMStateRLLocomotion<T>::qrFSMStateRLLocomotion(qrControlFSMData<T> *controlFSMData, qrLocomotionController* locomotionController_):
    qrFSMState<T>(controlFSMData, FSM_StateName::RL_LOCOMOTION, "RL_LOCOMOTION"), locomotionController(locomotionController_), locomotionControllerRLWrapper(locomotionController_)
{
    MITTimer tim;
    locomotionControllerRLWrapper.BindCommand(controlFSMData->networkPath);
    printf("RLWrapper Init Finished, cost %.3f [ms]\n", tim.getMs());

    this->TurnOnAllSafetyChecks();

    /* Turn off Foot pos command since it is set in WBC as operational task. */
    this->checkPDesFoot = false;

}


template<typename T>
void qrFSMStateRLLocomotion<T>::OnEnter()
{
    RC_MODE ctrlState = this->_data->desiredStateCommand->getJoyCtrlState();
    std::cout << "[FSM-RL] On Enter State: " << int(ctrlState) << std::endl;
    this->nextStateName = this->stateName;
    this->transitionData.Zero();
     
    // reset the robot control_mode
    // this->_data->_gaitScheduler->gaitData._nextGait = LocomotionMode::VELOCITY;

    if (ctrlState != RC_MODE::HARD_CODE) {
        /* This control frequency can be adjusted by user. */
        this->_data->userParameters->controlFrequency = 333;

        /* JOY_TROT: trotting by Force Balance controller.
         * JOY_ADVANCED_TROT: trotting by MPC or MPC-WBC controller.
         * WALK_LOCOMOTION: use walk gait.
         * JOY_STAND: keep standing.
         */
        switch (ctrlState) {
        case RC_MODE::RL_TROT: // 
            this->_data->quadruped->controlParams["mode"] = LocomotionMode::RL_LOCOMOTION;
            this->_data->gaitGenerator->gait = "rl_trot";
            break;
        default:
            this->_data->gaitGenerator->gait = "stand";
            break;
        }

        /* Reset the locomotion controller entering the locomotion state. */
        this->_data->quadruped->Reset();
        this->_data->quadruped->timeStep = 1.0 / this->_data->userParameters->controlFrequency;

        std::cout << "Time Step: " << this->_data->quadruped->timeStep << std::endl;

        locomotionControllerRLWrapper.Reset();
        this->_data->stateEstimators->Reset();
        count = 0;
    }
    printf("[FSM RL_LOCOMOTION] On Enter End\n");
}


template<typename T>
void qrFSMStateRLLocomotion<T>::Run()
{
    /* Call the locomotion control logic for this iteration and get the results, including commands and reaction force
     * Save the commands into FSM Data structure.
     */
    // std::cout << "iter >>>>>>>>>>>>>>>>>>>> " << iter_ << ", " << qrFSMState<T>::enableStateEstimation << std::endl;
    locomotionControllerRLWrapper.RLUpdate(count==0);
    std::vector<qrMotorCommand> hybridAction = locomotionControllerRLWrapper.GetRLAction();
    this->_data->legCmd = std::move(hybridAction);
    
    if (++count > 3) {
        count = 0;
        // qrFSMState<T>::enableStateEstimation = true;
    }
    // else {
    //     qrFSMState<T>::enableStateEstimation = true;
    // }
    // if (++iter_ > 30000) {
    //     throw std::logic_error("126");
    // }
}


template<typename T>
FSM_StateName qrFSMStateRLLocomotion<T>::CheckTransition()
{
    iter++;
    // printf("CheckTransition = %d\n", this->_data->quadruped->fsmMode); // K_LOCOMOTION
    if (LocomotionSafe()) {

        switch (this->_data->quadruped->fsmMode) {
        case K_LOCOMOTION:
            break;

        /* GAIT_TRANSITION and LOCOMOTION_STAND happen in the locomotion state.
         * Different gaits and MPC-WBC standing can be considered as substates of locomotion.
         */
        case GAIT_TRANSITION:
        {
            this->transitionDuration = 2.0;
            iter = 0;
            printf("FSM_State_Locomotion: reset iter for GAIT_TRANSITION!!!\n");
            // const RC_MODE ctrlState = this->_data->desiredStateCommand->getJoyCtrlState();
            // if (ctrlState != RC_MODE::RL_TROT) {
                this->nextStateName = FSM_StateName::LOCOMOTION;
            // }
            break;
        }
        case LOCOMOTION_STAND:
            this->transitionDuration = 2.0;
            iter = 0;
            this->nextStateName = FSM_StateName::LOCOMOTION;
            printf("FSM_State_Locomotion: reset iter for LOCOMOTION_STAND!!!\n");
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << "K_LOCOMOTION" << " to "
                      << this->_data->quadruped->fsmMode << std::endl;
        }
    } else {
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.0;

    }

    return this->nextStateName;
}


template<typename T>
qrTransitionData<T> qrFSMStateRLLocomotion<T>::Transition()
{   
    switch (this->nextStateName) {
    case FSM_StateName::LOCOMOTION:
        /* If transfer from trot to walk, gait transition happens and quadruped needs switch mode.
         * Or will enter MPC standing.
         */
        // if ((int)this->_data->quadruped->fsmMode==GAIT_TRANSITION) {
        //     SwitchMode();
        // } else {
            StandLoop();
            // robot->fsmMode = K_LOCOMOTION;
            // iter = 0;
            // this->transitionData.done = true;
            // this->transitionDuration = 0;
            // ROS_INFO("transitionData.done");
            // return true;
        // }
        iter++;
        break;
    case FSM_StateName::BALANCE_STAND:
        iter++;
        if (iter >= this->transitionDuration * 1000) {
            this->transitionData.done = true;
        } else {
            this->transitionData.done = false;
        }
        break;
    default:
        printf("[CONTROL FSM] Wrong in transition\n");
        throw std::runtime_error("191");
    }

    return this->transitionData;
}


template <typename T>
bool qrFSMStateRLLocomotion<T>::SwitchMode()
{   
    qrRobot *robot = this->_data->quadruped;

    if (iter >= this->transitionDuration * 1000) {
        robot->fsmMode = K_LOCOMOTION;
        iter = 0;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }

    int N = robot->GetFootContact().cast<int>().sum();

    if (iter < 1000) {
        /* Slow Down the velocity of the quadruped in 1000 iterations if the quadruped is trotting. */
        UpdateControllerParams(locomotionController, {0.f, 0.f, 0.f}, 0.f);
        this->_data->desiredStateCommand->stateDes.segment(6, 6) << 0, 0, 0, 0, 0, 0;
        
        locomotionController->Update();

        auto [hybridAction, qpSol] = locomotionController->GetAction();

        this->transitionData.legCommand = std::move(hybridAction);
        /* If four feet are on the groud, then continue to next stage. */
        if (N == 4) {
            iter = 1000;
        }
    } else {
        /* Keep stance for 1s. */
        auto angles = robot->GetMotorAngles();
        float ratio = std::max(0.45f, (iter-1000)/1000.0f);
        for (int i = 0; i < NumMotor; ++i) {
            this->transitionData.legCommand[i] = {robot->standUpMotorAngles[i]*ratio + (1-ratio)*angles[i], robot->motorKps[i], 0, robot->motorKds[i], 0};
        }
    }
    return true;
}


template <typename T>
bool qrFSMStateRLLocomotion<T>::StandLoop()
{    
    /* Similar logic to %SwitchMode(). */
    qrRobot *robot = this->_data->quadruped;
    if (iter >= 1000) {
        robot->fsmMode = K_LOCOMOTION;
        iter = 0;
        this->transitionData.done = true;
        this->transitionDuration = 0;
        ROS_INFO("transitionData.done");
        return true;
    }
    
    int N = robot->GetFootContact().cast<int>().sum();
    if (iter < this->transitionDuration * 1000) {
        UpdateControllerParams(locomotionController, {0.f, 0.f, 0.f}, 0.f);
        this->_data->desiredStateCommand->stateDes.segment(6, 6) << 0, 0, 0, 0, 0, 0;
        
        locomotionControllerRLWrapper.RLUpdate(false);
        std::vector<qrMotorCommand> hybridAction = locomotionControllerRLWrapper.GetRLAction();
        this->transitionData.legCommand = std::move(hybridAction);
        
        // locomotionController->Update();
        // auto [hybridAction, qpSol] = locomotionController->GetAction();
        // this->transitionData.legCommand = std::move(hybridAction);
        if (N == 4) {
            iter = 1000;
        }
    }
    return true;
}


template<typename T>
bool qrFSMStateRLLocomotion<T>::LocomotionSafe()
{
    return true;
}


template<typename T>
void qrFSMStateRLLocomotion<T>::OnExit()
{
    /* Standup state does nothing when exitting */
    iter = 0;
}

template class qrFSMStateRLLocomotion<float>;
