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
            startTime = std::chrono::high_resolution_clock::now();
        };

        double hasElapsed() {
            std::chrono::duration<double, std::nano> d = (std::chrono::high_resolution_clock::now() - startTime);
            return d.count() / 1000000000.0;
        };
    private:
        std::chrono::_V2::high_resolution_clock::time_point startTime;
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

        double getTimeoutPeriod() const
        {
            return timeout;
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