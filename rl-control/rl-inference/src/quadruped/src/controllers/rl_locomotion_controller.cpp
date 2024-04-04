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

#include "controllers/rl_locomotion_controller.h"

std::vector<std::string> split (const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline(ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}

#if USE_NPU // OM
static int32_t deviceId = 0;
static uint32_t modelId;
static size_t dataSize = 0;
static void *hostData = nullptr;
static void *deviceData = nullptr;
static size_t dataSize1 = 0;
static void *hostData1 = nullptr;
static void *deviceData1 = nullptr;

static void *outputHostData = nullptr;
static size_t outputDataSize = 0;
static void *outputDeviceData = nullptr;

static aclmdlDataset *inputDataSet;
static aclDataBuffer *inputDataBuffer;
static aclmdlDataset *inputDataSet1;
static aclDataBuffer *inputDataBuffer1;
static aclmdlDataset *outputDataSet;
static aclDataBuffer *outputDataBuffer;
static aclmdlDesc *modelDesc;

void InitResource()
{
    //指定当前进程或线程中用于运算的Device，同时隐式创建默认Context。同步接口。
    aclError ret = aclInit(nullptr);
    ret = aclrtSetDevice(deviceId);
}

void LoadModel(const char* modelPath)
{
    //模型ID的指针。
    //系统成功加载模型后会返回的模型ID。
    aclError ret = aclmdlLoadFromFile(modelPath, &modelId);
    printf("LoadModel %d, modelid = %d\n",ret, modelId);
}

void CreateHostData()
{
    dataSize = Quadruped::OBS_SIZE * sizeof(float);
    aclError ret = aclrtMallocHost(&hostData, dataSize);
    // hostData = malloc(dataSize);
    dataSize1 = Quadruped::HISTORY_STEPS * dataSize;
    ret = aclrtMallocHost(&hostData1, dataSize1);
    // hostData1 = malloc(dataSize1);
}

//申请Device侧的内存，再以内存复制的方式将内存中的图片数据传输到Device
void CopyDataFromHostToDevice()
{
    //第三个参数代表申请内存的相关策略
    aclError ret;
    if (deviceData == nullptr) {
        ret = aclrtMalloc(&deviceData, dataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    }
    //进行主机内存到设备内存间的复制
    ret = aclrtMemcpy(deviceData, dataSize, hostData, dataSize, ACL_MEMCPY_HOST_TO_DEVICE);
    if (deviceData1 == nullptr) {
        ret = aclrtMalloc(&deviceData1, dataSize1, ACL_MEM_MALLOC_HUGE_FIRST);
    }
    ret = aclrtMemcpy(deviceData1, dataSize1, hostData1, dataSize1, ACL_MEMCPY_HOST_TO_DEVICE);
}

// 准备模型推理的输入数据结构
void CreateModelInput()
{
    //创建aclmdlDataset类型的数据，描述模型推理的输入
    //使用aclmdlDesc类型的数据描述模型基本信息（例如输入/输出的个数、名称、数
    //据类型、Format、维度信息等）。
    //模型加载成功后，用户可根据模型的ID，调用该数据类型下的操作接口获取该模
    //型的描述信息，进而从模型的描述信息中获取模型输入/输出的个数、内存大小、
    //维度信息、Format、数据类型等信息。
    //● 使用aclDataBuffer类型的数据来描述每个输入/输出的内存地址、内存大小。
    //调用aclDataBuffer类型下的操作接口获取内存地址、内存大小等，便于向内存中存放输入数据、获取输出数据。
    //● 使用aclmdlDataset类型的数据描述模型的输入/输出数据。
    //模型可能存在多个输入、多个输出，调用aclmdlDataset类型的操作接口添加多个aclDataBuffer类型的数据。
    inputDataSet = aclmdlCreateDataset();

    inputDataBuffer = aclCreateDataBuffer(deviceData, dataSize);
    aclError ret = aclmdlAddDatasetBuffer(inputDataSet, inputDataBuffer);
    
    inputDataBuffer1 = aclCreateDataBuffer(deviceData1, dataSize1);
    ret = aclmdlAddDatasetBuffer(inputDataSet, inputDataBuffer1);
}

// 准备模型推理的输出数据结构
void CreateModelOutput()
{
    // 创建模型描述信息
    modelDesc = aclmdlCreateDesc();
    aclError ret = aclmdlGetDesc(modelDesc, modelId);
    // 创建aclmdlDataset类型的数据，描述模型推理的输出
    outputDataSet = aclmdlCreateDataset();
    // 获取模型输出数据需占用的内存大小，单位为Byte
    outputDataSize = aclmdlGetOutputSizeByIndex(modelDesc, 0);
    printf("outputDataSize = %lu\n", outputDataSize);
    if (outputDataSize == 0) {
        throw std::runtime_error("om model is not right!");
    }
    // 申请输出内存
    ret = aclrtMalloc(&outputDeviceData, outputDataSize, ACL_MEM_MALLOC_HUGE_FIRST);
    
    outputDataBuffer = aclCreateDataBuffer(outputDeviceData, outputDataSize);
    ret = aclmdlAddDatasetBuffer(outputDataSet, outputDataBuffer);
}

void ACLInference()
{
    if (outputDeviceData == nullptr) {
        CreateModelInput();
        CreateModelOutput();
    }
    // printf("Start Inference...\n");
    // start();
    aclError ret = aclmdlExecute(modelId, inputDataSet, outputDataSet);
    // printf("cost t = %.3f [ms]\n", getMs());
    // printf("End Inference !\n");
    // 获取推理结果数据
    if (outputHostData == nullptr) {
        ret = aclrtMallocHost(&outputHostData, outputDataSize);
    }
    ret = aclrtMemcpy(outputHostData, outputDataSize, outputDeviceData, outputDataSize, ACL_MEMCPY_DEVICE_TO_HOST);
}

void UnloadModel()
{
    // 释放模型描述信息
    aclmdlDestroyDesc(modelDesc);
    // 卸载模型
    aclmdlUnload(modelId);
}

void UnloadData()
{
    aclError ret = aclrtFreeHost(hostData);
    hostData = nullptr;
    ret = aclrtFree(deviceData);
    deviceData = nullptr;
    aclDestroyDataBuffer(inputDataBuffer);
    inputDataBuffer = nullptr;
    aclmdlDestroyDataset(inputDataSet);
    inputDataSet = nullptr;

    ret = aclrtFreeHost(hostData1);
    hostData1 = nullptr;
    ret = aclrtFree(deviceData1);
    deviceData1 = nullptr;
    aclDestroyDataBuffer(inputDataBuffer1);
    inputDataBuffer1 = nullptr;
    aclmdlDestroyDataset(inputDataSet1);
    inputDataSet1 = nullptr;

    ret = aclrtFreeHost(outputHostData);
    outputHostData = nullptr;
    ret = aclrtFree(outputDeviceData);
    outputDeviceData = nullptr;
    aclDestroyDataBuffer(outputDataBuffer);
    outputDataBuffer = nullptr;
    aclmdlDestroyDataset(outputDataSet);
    outputDataSet = nullptr;
}

void DestroyResource()
{
    //复位当前运算的Device，释放Device上的资源，包括默认Context、默认Stream以及默认Context下创建的所有Stream，同步接口。若默认Context或默认Stream下的任务还未完成，系统会等待任务完成后再释放。
    aclError ret = aclrtResetDevice(deviceId);
    aclFinalize();
}

#endif

namespace Quadruped {

float CubicUp(float x) {
    return -16 * pow(x, 3) + 12 * pow(x, 2);
}

float CubicDown(float x) {
    return 16 * pow(x, 3) - 36 * pow(x, 2) + 24 * x - 4;
}

void SwapFourLeg(Vec12<float>& data) {
    Vec3<float> temp;
    for (int i=0; i < 12; i = i + 6) {
        temp = data.block<3, 1>(0 + i, 0);
        data.block<3, 1>(0 + i, 0) = data.block<3, 1>(3 + i,0);
        data.block<3, 1>(3 + i, 0) = temp;
    }
}

LocomotionControllerRLWrapper::LocomotionControllerRLWrapper(qrLocomotionController* locomotionController_)
    :locomotionController(locomotionController_), robot(locomotionController_->robot)
{
    command_scale = 1;
    rpy_scale = 1;
    v_scale = 1;
    w_scale = 1;
    dp_scale = 1;
    dv_scale = 0.1;
    cpg_scale = 1;
    height_scale = 5;
    obs_history.reserve(HISTORY_STEPS * OBS_SIZE);
    auto trot_info = locomotionController_->GetStanceLegController()->param["stance_leg_params"]["rl_trot"];
    kps = trot_info["KP"].as<std::vector<float>>();
    kds = trot_info["KD"].as<std::vector<float>>();
    MaxClearance = trot_info["max_clearance"].as<float>();
    MaxHorizontalOffset = trot_info["max_horizontal_offset"].as<float>();
    assertm(kps.size() == 12 && kds.size() == 12, "size not 12!");
}

void LocomotionControllerRLWrapper::Reset()
{
    locomotionController->Reset();

    deque_dof_p.clear();
    deque_dof_v.clear();
    deque_dof_p_target.clear();
    obs_history.clear();
    gaitFreq = 1.f/locomotionController->gaitGenerator->fullCyclePeriod[0];
    
    target_joint_angles.setZero();
    deltaPhi.setZero();
    phi.setZero();
    residualAngle.setZero();
    proprioceptiveObs.setZero();
    exteroceptiveObs.setZero();
    total_obs.setZero();
    command.setZero();
    PMTGStep();
}

LocomotionControllerRLWrapper::~LocomotionControllerRLWrapper()
{
    #if USE_NPU
    UnloadModel();
    UnloadData();
    DestroyResource();
    #endif
}


void LocomotionControllerRLWrapper::BindCommand(std::string networkPath) {
    locomotionController->BindCommand();
    
    if (networkPath.size() == 0) {
        throw std::logic_error("");
    } else {
        std::vector<std::string> words = split(networkPath, '.');
        std::string& networkType = words.back();
        // LoadModel
        if (networkType == "onnx") {
            #if USE_ONNX
            // ONNX LOAD FUNCTION
            onnxEnv = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test");
            Ort::SessionOptions session_options;
            onnxSession = Ort::Session(onnxEnv, networkPath.c_str(), session_options);
            #endif
        } else if (networkType == "om") {
            #if USE_NPU
            // ACL LOAD FUNCTION
            if (dataSize == 0) { // cost 500 ms
                InitResource();
                LoadModel(networkPath.c_str());
                CreateHostData();
            }
            #endif
        } else {
            // CKPT LOAD FUNCTION, TODO
            throw std::logic_error("");
        }
    }    
}


void LocomotionControllerRLWrapper::RLUpdate(bool enableInference)
{
    if (!robot->stop) {
        locomotionController->timeSinceReset = robot->GetTimeSinceReset() - locomotionController->resetTime;
    }
    if (enableInference) {
        if (allowUpdateObs) {
            CollectProprioceptiveObs();
            total_obs << proprioceptiveObs,
                        exteroceptiveObs;

            // allowUpdateObs = false;
        }
        // MITTimer tik;
        // Inference(proprioceptiveObs);
        Inference(total_obs);
        // printf("Inference TIME: %.3f [ms]\n", tik.getMs());   
    }
    
    // pmtg update, not swing controllers
    locomotionController->gaitGenerator->Update(locomotionController->timeSinceReset);
    target_joint_angles.setZero();
    PMTGStep();

    // std::cout << "target_joint = " << target_joint_angles.transpose() << std::endl;
    if (deque_dof_p_target.size() < 2) {
        Vec12<float> swaped_target_joint_angles = target_joint_angles;
        SwapFourLeg(swaped_target_joint_angles);
        deque_dof_p_target.push_back(swaped_target_joint_angles);
    }
}


void LocomotionControllerRLWrapper::Inference(Eigen::Matrix<float, OBS_SIZE, 1>& obs)
{
    if (obs_history.empty()) {
        Eigen::Matrix<float, HISTORY_SIZE, 1> obs_history_data = obs.replicate<HISTORY_STEPS,1>();
        obs_history.insert(obs_history.end(), obs_history_data.data(),  obs_history_data.data() + obs_history_data.size());
    } else {
        auto it = obs_history.begin();
        obs_history.erase(it, it+OBS_SIZE);
        obs_history.insert(obs_history.end(), obs.data(), obs.data()+obs.size());
    }
    // pretty_print(obs_history.data() + 320*38, "obs_his[-2]", 320);
    // pretty_print(obs_history.data() + 320*39, "obs_his[-1]", 320);
    // std::cout << "obs_his[-2] = " << obs_history.block<1, 320>(0, 320*38) << std::endl;
    // std::cout << "obs_his[-1] = " << obs_history.block<1, 320>(0, 320*39) << std::endl;
    
    #if USE_NPU
    memcpy(hostData, obs.data(), dataSize);
    memcpy(hostData1, obs_history.data(), dataSize1);
    CopyDataFromHostToDevice();
    ACLInference();
    float* results_ = reinterpret_cast<float*>(outputHostData);
    
    #else
    // std::cout << "dof p = " << robot->GetMotorAngles().transpose() << std::endl;
    std::vector<int64_t> input0_node_dims = {1, OBS_SIZE};
    std::vector<float> input0_tensor_values(obs.data(), obs.data() + OBS_SIZE);
    std::vector<int64_t> input1_node_dims = {1, HISTORY_SIZE};
    // std::cout << "obs = " << obs.block<133, 1>(0,0).transpose() << std::endl;

    // std::vector<float> input1_tensor_values(obs_history.data(), obs_history.data() + obs_history.rows() * obs_history.cols());
    std::array<float, 16> results_{};
    std::array<int64_t, 2> output_shape_{1, 16};
    // std::vector<float> output_tensor_values(results_.data(), results_.size());

    // Fill input_tensor_values with your input data.
    std::vector<Ort::Value> input_tensor;
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    // Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    // Ort::Value input0_tensor = Ort::Value::CreateTensor<float>(memory_info, input0_tensor_values.data(), 320, input0_node_dims.data(), input0_node_dims.size());
    // Ort::Value input1_tensor = Ort::Value::CreateTensor<float>(memory_info, obs_history.data(), 320*40, input1_node_dims.data(), input1_node_dims.size());
    input_tensor.emplace_back(Ort::Value::CreateTensor<float>(memory_info, input0_tensor_values.data(), OBS_SIZE, input0_node_dims.data(), input0_node_dims.size()));
    input_tensor.emplace_back(Ort::Value::CreateTensor<float>(memory_info, obs_history.data(), HISTORY_SIZE, input1_node_dims.data(), input1_node_dims.size()));
    Ort::Value output_tensor = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(),
                                                     output_shape_.data(), output_shape_.size());

    Ort::AllocatedStringPtr inputNodeName0 = onnxSession.GetInputNameAllocated(0, onnxAllocator);
    Ort::AllocatedStringPtr inputNodeName1 = onnxSession.GetInputNameAllocated(1, onnxAllocator);
    Ort::AllocatedStringPtr outputNodeName = onnxSession.GetOutputNameAllocated(0, onnxAllocator);
    const char* inputName0 = inputNodeName0.get();
    // std::cout << "Input Name0: " << inputName0 << std::endl;
    const char* inputName1 = inputNodeName1.get();
    // std::cout << "Input Name1: " << inputName1 << std::endl;
    const char* outputName = outputNodeName.get();
    // std::cout << "Output Name: " << outputName << std::endl;
    
    //Run Inference
    std::vector<const char*> inputNames{inputName0, inputName1};
    std::vector<const char*> outputNames{outputName};
    // std::vector<Ort::Value*> input_tensor{&input0_tensor, &input1_tensor};
    // input_tensor.push_back(input0_tensor);
    // input_tensor.push_back(input1_tensor);
    
    // Run inference.
    // onnxSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), *((const Ort::Value* const*)input_tensor.data()), inputNames.size(), outputNames.data(),  &output_tensor, outputNames.size());
    onnxSession.Run(Ort::RunOptions{nullptr}, inputNames.data(), input_tensor.data(), inputNames.size(), outputNames.data(),  &output_tensor, outputNames.size());
    #endif
    
    // auto& output_tensor = output_tensors.front().Get<Tensor>();
    // auto output_shape = output_tensor.Shape();
    // std::vector<float> output_tensor_values(output_shape.Size());
    // memcpy(output_tensor_values.data(), output_tensor.Data<float>(), output_shape.Size() * sizeof(float));
    
    // deltaPhi = output[:4];
    // residualAngle = output[4:16];
    for(int i=0; i<4; ++i) {
        int j = (i%2==0 ? i+1:i-1); // todo
        deltaPhi[i] = results_[j] * 0.2f;
        for (int k=0; k < 3; ++k) {
            residualAngle[3*i+k] = results_[4+3*j+k] * 0.2f;
        }
    }
    // Visualization2D& vis = robot->stateDataFlow.visualizer;
    // vis.datax.push_back(timeSinceReset);
    // vis.datay1.push_back(deltaPhi[0]);
    // vis.datay2.push_back(deltaPhi[1]);
    // vis.datay3.push_back(deltaPhi[2]);
    // vis.datay4.push_back(deltaPhi[3]);
    // vis.datay5.push_back(residualAngle[0]);
    // vis.datay6.push_back(residualAngle[1]);
    
    // memcpy(deltaPhi, results_.data(), 4*sizeof(float));
    // memcpy(residualAngle, results_.data() + 4, 12*sizeof(float));

    // std::cout << "results_  = ";
    // for (int i=0; i<16; ++i) {
    //     std::cout << " " <<  results_[i];
    // }
    // std::cout <<'\n';

}

void LocomotionControllerRLWrapper::CollectProprioceptiveObs()
{
    const Vec12<float>& stateDes = locomotionController->desiredStateCommand->stateDes;
    command << stateDes(6),
               stateDes(7),
               stateDes(11); 
    Vec12<float> dof_pos = robot->GetMotorAngles();
    Vec12<float> dof_v = robot->GetMotorVelocities();
    SwapFourLeg(dof_pos); 
    SwapFourLeg(dof_v);
    // std::cout << "dof_pos = "<< dof_pos.transpose() << std::endl;
    
    while (deque_dof_p.size() < 4) {
        deque_dof_p.push_back(dof_pos);
    }
    while (deque_dof_v.size() < 3) {
        deque_dof_v.push_back(dof_v);
    }
    while (deque_dof_p_target.size() < 2) {
        deque_dof_p_target.push_back(dof_pos);
    }

    for (int i=0; i<3; ++i) {
        dof_p_history.block(12*i, 0, 12, 1) = deque_dof_p[i];
    }
    for (int i=0; i<2; ++i) {
        dof_v_history.block(12*i, 0, 12, 1) = deque_dof_v[i];
    }
    for (int i=0; i<2; ++i) {
        dof_p_target_history.block(12*i, 0, 12, 1) = deque_dof_p_target[i];
    }
    deque_dof_p.pop_front();
    deque_dof_v.pop_front();
    deque_dof_p_target.pop_front();

    /*
     0          0          0 
     -0.0231767  -0.120742   0.108926   
     0.126287  0.0200281  0.0321777  
     -0.113515   0.360507  -0.031097  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     -0.030091   0.232572  -0.269918  0.0227078   0.172648 -0.0502232  
     0.0922905   0.127618   0.278503  0.0641741    0.39569   0.446557  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823  
     0.0013907    0.84213   -1.62425 0.00720632   0.839306   -1.62602 
     -0.0812504   0.736395   -1.68194  0.0730327   0.707148   -1.71823          
     0          0          0          0          1          1          1          1          0          0          0          0        1.4
    */
    Vec3<float> v = robot->baseVelocityInBaseFrame;
    // if (robot->robotName == "lite3")
    //     v[0] = 0; // todo
    proprioceptiveObs <<  command * command_scale,
                        robot->baseRollPitchYaw * rpy_scale,
                        v * v_scale,
                        robot->baseRollPitchYawRate * w_scale,
                        dof_pos * dp_scale,
                        dof_v * dv_scale,
                        dof_p_history * dp_scale,
                        dof_v_history * dv_scale,
                        dof_p_target_history * dp_scale, // 
                        cpg_info[1] * cpg_scale, cpg_info[0] * cpg_scale, cpg_info[3] * cpg_scale, cpg_info[2] * cpg_scale,
                        cpg_info[5] * cpg_scale, cpg_info[4] * cpg_scale, cpg_info[7] * cpg_scale, cpg_info[6] * cpg_scale,
                        cpg_info[9] * cpg_scale, cpg_info[8] * cpg_scale, cpg_info[11] * cpg_scale, cpg_info[10] * cpg_scale,
                        cpg_info[12] * cpg_scale;  // 13

    // std::cout << "proprioceptiveObs= " << proprioceptiveObs.transpose() << std::endl;
}

void LocomotionControllerRLWrapper::PMTGStep()
{
    // pretty_print(delta_phi_, "delta_phi_", 4);
    // pretty_print(residual_angle_, "residual_angle_", 12);
    // gen_foot_target_position_in_horizontal_hip_frame(delta_phi, residual_xyz, **kwargs)
    // foot_trajectory = gen_foot_trajectory_axis_z(delta_phi, timeSinceReset);
    // Eigen::Map<Eigen::MatrixXf> residual_angle(residual_angle_,12, 1);;
    // Eigen::Map<Eigen::MatrixXf> delta_phi(delta_phi_,4, 1);;

    // std::cout << gaitGenerator->phaseInFullCycle << std::endl;
    phi = 2 * M_PI * locomotionController->gaitGenerator->phaseInFullCycle + deltaPhi;
    // phi = phi.unaryExpr([](const float x) { return fmod(x, 3.1415f*2);});
    // std::cout << "phi = " << phi.transpose() << std::endl;
    cpg_info << deltaPhi, phi.array().cos(), phi.array().sin(), gaitFreq;

    Vec4<bool> is_swing = locomotionController->gaitGenerator->desiredLegState.cwiseEqual(0); //swing:0, stance: 1
    // is_swing.setZero();
    // std::cout << "is_swing = " << is_swing.transpose() << std::endl;
    Vec4<float> swing_phi = locomotionController->gaitGenerator->normalizedPhase;  // [0,1)
    Vec4<float> sin_swing_phi = (swing_phi * M_2PI).array().sin();
    // std::cout << "sin_swing_phi = " <<  sin_swing_phi.transpose() << std::endl;
    Vec4<float> factor{0,0,0,0};
    for (int i=0; i<4; ++i) {
        if(is_swing[i]) {
            factor[i] = swing_phi[i] < 0.5 ? CubicUp(swing_phi[i]):CubicDown(swing_phi[i]);
        }
    }

    foot_target_position_in_hip_frame.setZero();
    foot_target_position_in_hip_frame.col(2) = factor.cwiseProduct(is_swing.cast<float>() * MaxClearance).array() - robot->bodyHeight;
    // if (robot->robotName == "lite3" && (command[0] < -5  ||  robot->baseRollPitchYaw[1] < -0.05 ) ) {
    //     foot_target_position_in_hip_frame(2, 2) -= 0.02;
    //     foot_target_position_in_hip_frame(3, 2) -= 0.02;
    //     // foot_target_position_in_hip_frame(2, 0) -= 0.02;
    //     // foot_target_position_in_hip_frame(3, 0) -= 0.02;
    // }
    foot_target_position_in_hip_frame.col(0) = -MaxHorizontalOffset * sin_swing_phi.cwiseProduct(is_swing.cast<float>());
    Mat3<float> RT = robotics::math::coordinateRotation(robotics::math::CoordinateAxis::X, robot->baseRollPitchYaw[0]) *
                     robotics::math::coordinateRotation(robotics::math::CoordinateAxis::Y, robot->baseRollPitchYaw[1]);
    Eigen::Matrix<float, 4, 3> foot_target_position_in_base_frame = robot->defaultHipPosition.transpose() +  foot_target_position_in_hip_frame * RT.transpose();  // WORLD --> BASE  [0.185, 0.135, 0]

    Vec3<int> joint_idx;
    Vec3<float> target_joint_angles_per_leg;
    for (int i = 0; i < 4; ++i) {
        robot->ComputeMotorAnglesFromFootLocalPosition(i, foot_target_position_in_base_frame.row(i), joint_idx, target_joint_angles_per_leg);
        for (int j = 0; j < 3; ++j) {
            target_joint_angles(joint_idx[j]) = target_joint_angles_per_leg(j) + residualAngle(joint_idx[j]);
        }
    }
    
}

std::vector<qrMotorCommand> LocomotionControllerRLWrapper::GetRLAction()
{
    std::vector<qrMotorCommand> action;
    target_joint_angles = target_joint_angles.cwiseMax(-3.0f).cwiseMin(3.0f);

    for (int jointId(0); jointId < NumMotor; ++jointId) {
        float kp = kps[jointId];
        float p = target_joint_angles[jointId];
        float tua = 0;
        // if (robot->robotName == "lite3") {
        //     if (command[0] < 0.0) {
        //         // if (jointId == 7 || jointId == 10) {
        //         //     p += 0.13;
        //         // }
        //         if (jointId == 8 || jointId == 11) {
        //             p += 0.13;
        //         }
        //     }
        //     // if (common_step_counter < 250) {
        //     //     p[8] += 0.2
        //     //     p[11] += 0.2
        //     // }
        //     float phi_ = locomotionController->gaitGenerator->phaseInFullCycle[jointId/3]; //phi[jointId/3];
        //     // phi_ = fmod(phi_ + 6.283, 6.283);
        //     float sin_phi = std::sin(phi_ / 1.2);
        //     if (phi_ < 0.6)
        //         sin_phi = std::sin(phi_*2*301415 / 1.2);
        //     else
        //         sin_phi = -std::sin((phi_ - 0.6)*2*301415 / 0.8);
            
            
        //     float ratio = 1.0;
        //     if (sin_phi > 0) {
        //         ratio *= 3;
        //     }
        //     if (jointId == 2 || jointId == 5) {
        //         kp = 18 + 2 * ratio * sin_phi;
        //     } else if (jointId == 8 || jointId == 11) {
        //         kp = 21 + 2 * ratio * sin_phi;
        //     } else if(jointId == 1 || jointId == 4) {
        //         kp = 18 + 2 * ratio * sin_phi;
        //     } else if (jointId == 7 || jointId == 10) {
        //         kp = 21 + 2 * ratio * sin_phi;
        //     } else {
        //         ;
        //     }
            
        // }
        action.push_back({p, kp, 0, kds[jointId], tua});
    }
    return action;
}

} // Namespace Quadruped
