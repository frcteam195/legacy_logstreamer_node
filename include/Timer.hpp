#pragma once

#include <chrono>
#include <mutex>
#include <math.h>

namespace ck
{
    class ElapsedTimer {
    public:
        ElapsedTimer() {};
        void start() {
            startTime = std::chrono::system_clock::now();
        };

        double hasElapsed() {
            return (std::chrono::system_clock::now() - startTime).count();
        };
    private:
        std::chrono::_V2::system_clock::time_point startTime;
    };

    class TimeoutTimer {
    public:
        TimeoutTimer(double timeout)
        {
            this->timeout = timeout;
            setFirstRun(true);
        };

        bool isTimedOut()
        {
            if (firstRun) {
                eTimer.start();
                setFirstRun(false);
            }
            return eTimer.hasElapsed() > timeout;
        }

        void reset()
        {
            setFirstRun(true);
        }
        
        double getTimeLeft()
        {
            return std::fmax(timeout - eTimer.hasElapsed(), 0.0);
        }

    private:
        double timeout;
        bool firstRun;
        ElapsedTimer eTimer;

        std::mutex mtx; 

        void setFirstRun(bool firstRun)
        {
            std::lock_guard<std::mutex> lock(mtx);
            this->firstRun = firstRun;
        }
    };
};