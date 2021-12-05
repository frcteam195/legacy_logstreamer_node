#pragma once

#include "arpa/inet.h"
#include "Timer.hpp"

class ReportRequestor
{
public:
    ReportRequestor(sockaddr_in inetAddress);
    ReportRequestor(sockaddr_in inetAddress, int heartbeatTimeout);
    
    sockaddr_in getIpAddr();
    bool isExpired();
    void pumpHeartbeat();
};