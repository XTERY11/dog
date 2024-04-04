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

#ifndef QR_TIMER_H
#define QR_TIMER_H

#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <ros/ros.h>


class qrTimer {

public:

    /**
     * @brief Constructor of class qrTimer
     */
    qrTimer() {
        start = clock();
        startTime = (double)start;
    };

    virtual ~qrTimer() = default;

    /**
     * @brief Get time since robot reset.
     * @return time since reset.
     */
    virtual double GetTimeSinceReset() {
        finish = clock();
        double timeSinceReset = (double)(finish - startTime) / CLOCKS_PER_SEC; // second(s)
        return timeSinceReset;
    };

    /**
     * @brief Set current time as start time
     */
    virtual double ResetStartTime() {
        start = clock();
        startTime = (double)start;
        return startTime;
    };

private:

    /**
     * @brief Start time.
     */
    clock_t start;

    /**
     * @brief Finish time.
     */
    clock_t finish;

    /**
     * @brief Start time
     */
    double startTime;

};


class qrRosTimer: public qrTimer {

public:

    /**
     * @brief Constructor of class qrRosTimer
     */
    qrRosTimer() {
        startRos = ros::Time::now().toSec();
    };

    virtual ~qrRosTimer() = default;

    /**
     * @see qrTimer::GetTimeSinceReset
     */
    virtual double GetTimeSinceReset() {
        double timeSinceReset = ros::Time::now().toSec() - startRos;
        return timeSinceReset;
    };

    /**
     * @see qrTimer::ResetStartTime
     */
    virtual double ResetStartTime() {
        startRos = ros::Time::now().toSec();
        return startRos;
    };

private:

    /**
     * @brief Start time in ROS.
     */
    double startRos;

};


class qrTimerInterface {

public:

    /**
     * @brief Constructor of class TimerInterface.
     * @param useRosTimeIn: whether to use ROS timer tools.
     */
    qrTimerInterface(bool useRosTimeIn=false) : useRosTime(useRosTimeIn), timerPtr(nullptr) {
        if (useRosTime) {
            timerPtr = new qrRosTimer();
        } else {
            timerPtr = new qrTimer();
        }

        startTime = 0;
        timeSinceReset = 0;
    };

    /**
     * @brief Destructor of qrTimerInterface.
     */
    ~qrTimerInterface() {
        delete timerPtr;
    };

    double GetTimeSinceReset() {
        timeSinceReset = timerPtr->GetTimeSinceReset();
        return timeSinceReset;
    };

    void ResetStartTime() {
        startTime = timerPtr->ResetStartTime();
    };

private:

    bool useRosTime;

    qrTimer* timerPtr;

    double startTime;

    double timeSinceReset;

};


class MITTimer {
 public:

  /*!
   * Construct and start timer
   */
  explicit MITTimer() { start(); }

  /*!
   * Start the timer
   */
  void start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double getMs() { return (double)getNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t getNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double getSeconds() { return (double)getNs() / 1.e9; }

  struct timespec _startTime;
};


#endif // QR_TIMER_H
