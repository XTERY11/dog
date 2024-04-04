cmake_minimum_required(VERSION 3.10)
project(ascend_quadruped_cpp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "RELEASE")
#set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-D_cplusplus)

option(USE_GO1                            "WHICH ROBOT"                     OFF)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}") # x86_64, amd64, arm64
if(${ARCHITECTURE} STREQUAL "x86_64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(${ARCHITECTURE} STREQUAL "amd64") # 64Bit, 32Bit
    set(UNITREE_SDK_LIB "unitree_legged_sdk_amd64.so")
elseif(${ARCHITECTURE} STREQUAL "arm64")
    set(UNITREE_SDK_LIB "unitree_legged_sdk_arm64.so")
else()
    message(FATAL_ERROR "unrecongnized arch.")
endif()

include_directories("include/")                 # common includes
include_directories("config/")                 # common includes
include_directories("/usr/include/eigen3")   # eigen3 includes

find_package(yaml-cpp REQUIRED) # yaml includes
include_directories(${YAML_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files

## Third party Library
# Unitree SDK
if(${USE_GO1})
set(UNITREE_SDK_GO1_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk_3.4)
add_subdirectory(${UNITREE_SDK_GO1_DIR})
include_directories("${UNITREE_SDK_GO1_DIR}/include")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk_3.4)
link_directories(${UNITREE_SDK_GO1_DIR}/lib)
else() #A1 & AligenGo
set(UNITREE_SDK_DIR ${PROJECT_SOURCE_DIR}/third_party/unitree_legged_sdk)
add_subdirectory(${UNITREE_SDK_DIR})
include_directories("${UNITREE_SDK_DIR}/include")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/unitree_legged_sdk)
link_directories(${UNITREE_SDK_DIR}/lib)
endif()

# TinyEKF
set(TinyEKF_DIR ${PROJECT_SOURCE_DIR}/third_party/TinyEKF)
include_directories("${TinyEKF_DIR}/src")
file(GLOB_RECURSE TinyEKF_SRC "${TinyEKF_DIR}/src/*.c")                   # TinyEKF library cpp files
# QuadProgpp
set(QuadProgpp_DIR ${PROJECT_SOURCE_DIR}/third_party/QuadProgpp)
add_subdirectory(${QuadProgpp_DIR})
include_directories("${QuadProgpp_DIR}/src")
link_directories(${CMAKE_CURRENT_BINARY_DIR}/third_party/QuadProgpp)

#ROS
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        nav_msgs
        tf
        pcl_ros
        pcl_conversions
        unitree_legged_msgs
        eigen_conversions
        )
        find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS
  include
  ${Eigen_INCLUDE_DIRS}
 LIBRARIES
  ${PROJECT_NAME}_library
 CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  roscpp
  std_msgs
  message_runtime
  tf
  tf_conversions
  eigen_conversions
 DEPENDS
  system_lib
 
)

find_package(Eigen3 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

if(${USE_GO1})

else() #A1 & AligenGo

    ## Add Runtime lib
    add_library(robot_runtime SHARED ${sources} ${TinyEKF_SRC})
    target_link_libraries(robot_runtime ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES}
            ${UNITREE_SDK_LIB} robot_interface quadprog lcm)
    add_dependencies(robot_runtime ${catkin_EXPORTED_TARGETS})

    # ## Add Executable
    # add_executable(a1_real a1_real.cpp)
    # target_link_libraries(a1_real robot_runtime)
    # #add_dependencies(ascend_quadruped_cpp ${catkin_EXPORTED_TARGETS})

    # add_executable(aligengo_real aligengo_real.cpp)
    # target_link_libraries(aligengo_real robot_runtime)
    # #add_dependencies(ascend_quadruped_cpp ${catkin_EXPORTED_TARGETS})

    add_executable(a1_sim a1_sim.cpp)
    target_link_libraries(a1_sim robot_runtime)

    add_executable(state_from_gazebo state_from_gazebo.cpp)
    target_link_libraries(state_from_gazebo ${catkin_LIBRARIES})

endif()

# add_executable(UrdfToTfFromGazeboNode UrdfToTfFromGazeboNode.cpp)
# target_link_libraries(UrdfToTfFromGazeboNode ${catkin_LIBRARIES})
# #add_dependencies(sim_main ${catkin_EXPORTED_TARGETS})

# add_executable(opencv_visual opencv_visual.cpp)
# target_link_libraries(opencv_visual ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(sim_main ${catkin_EXPORTED_TARGETS})


# add_executable(tf_fake_pub tf_fake_pub.cpp)
# target_link_libraries(tf_fake_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_dependencies(tf_fake_pub ${catkin_EXPORTED_TARGETS})

#################################TEST#################################
#find_package(GTest REQUIRED)
#find_package(Threads REQUIRED)
#
#include_directories(${GTEST_INCLUDE_DIRS})
#
#add_executable(Test_Robot tests/ut/test_robot.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_Gait tests/ut/test_openloop_gait_generator.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_SwingLeg tests/ut/test_swing_leg_controller.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_stance_controller tests/ut/test_torque_stance_leg_controller.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_qp_solver tests/ut/test_qp_torque_optimizer.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_vel_estimator tests/ut/test_robot_velocity_estimator.cpp ${sources} ${TinyEKF_SRC})
#add_executable(Test_locomotion_controller tests/ut/test_locomotion_controller.cpp ${sources} ${TinyEKF_SRC})
#
#target_link_libraries(Test_Robot
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_Gait
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_SwingLeg
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_stance_controller
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_qp_solver
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_vel_estimator
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)
#
#target_link_libraries(Test_locomotion_controller
#        ${GTEST_BOTH_LIBRARIES}
#        ${CMAKE_THREAD_LIBS_INIT}
#        ${YAML_CPP_LIBRARIES}
#        robot_interface unitree_legged_sdk_amd64 quadprog lcm)

#add_test(Test Test_stance_controller)
#add_test(Test Test_qp_solver)
#add_test(Test Test_Robot)
#add_test(Test Test_Gait)
#add_test(Test Test_SwingLeg)
#add_test(Test Test_vel_estimator)
#add_test(Test Test_locomotion_controller)
#enable_testing()