#pragma once

#include <arpa/inet.h>
#include <string>

#include "ck_utilities/CKTimer.hpp"

class ReportRequestor
{
public:
    ReportRequestor(sockaddr_in &inetAddress, int heartbeatTimeout = 4);
    
    sockaddr_in getIpAddr() const;
    bool isExpired();
    void pumpHeartbeat();
    int getHeartbeatPeriod() const;

    bool operator==(const ReportRequestor &other) const;

private:
    sockaddr_in mIPAddr;
    ck::TimeoutTimer mTimeoutTimer;
};