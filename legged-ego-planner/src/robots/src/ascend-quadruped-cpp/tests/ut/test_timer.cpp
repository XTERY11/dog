#include "gtest/gtest.h"
#include "robots/timer.h"
#include "utils/se3.h"
#include "state_estimator/robot_velocity_estimator.h"

using namespace std;

// 定义测试类FooTest
class TestTimer: public testing::Test {
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
    MovingWindowFilter* winFilter;
    Timer timer;
    MovingWindowFilter* build() 
    {
        return new MovingWindowFilter(120);
    }

};
 
TEST_F(TestTimer, TestGetTime)
{
    auto tik = timer.GetTime();
    winFilter = build();
    double Values[5] = {0.75014373, 0.90864498, 0.03528129, 0.74184424, 0.99092506};
    double res;
    long long cycle = 100000;
    for(int i = 0; i < cycle; ++i) {
        res = winFilter->CalculateAverage(Values[i%5]);
    }
    auto tok = timer.GetTime();
    printf("duration = %f s\n", tok-tik);

}


int main(int argc, char** argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);
 
    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}