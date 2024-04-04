#include "gtest/gtest.h"
#include "robots/timer.h"
#include "mpc_controller/locomotion_controller.h"

using namespace std;

// 定义测试类FooTest
class TestLocomotionController: public testing::Test {
protected:
    // Code here will be called immediately after the constructor (right before each test)
    void SetUp()
    {
    	cout << "test start" << endl;
    }
 
    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
	    cout << "test end" << endl;
    }
 
public:
    LocomotionController* controller;
    Timer timer;
    LocomotionController* buildController() {
        return new LocomotionController(nullptr, 
                                nullptr,
                                nullptr,
                                nullptr,
                                nullptr,
                                timer);
    }

};
 

// int TestLocomotionController::(const int & nNum)
// {

TEST_F(TestLocomotionController, Test1)
{
    controller = buildController();
    EXPECT_NEAR(controller->GetTime(), timer.GetTime(), 1E-1);
}

int main(int argc, char** argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);
 
    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}