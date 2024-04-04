#ifndef ASCEND_QUADRUPED_CPP_TIMER_TIMER_H
#define ASCEND_QUADRUPED_CPP_TIMER_TIMER_H

#include <time.h>

class Timer {
public:
    Timer() {
        start = clock();
        startTime = (double) start;
    }

    inline double GetTimeSinceReset() {
        finish = clock();
        double timeSinceReset = (double) (finish - startTime) / CLOCKS_PER_SEC; // second(s)
        return timeSinceReset;
    }

    inline void ResetStartTime() {
        start = clock();
        startTime = (double) start;
    }


private:
    clock_t start;
    clock_t finish;
    double startTime;
};

#endif // ASCEND_QUADRUPED_CPP_TIMER_TIMER_H