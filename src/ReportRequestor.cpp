#include "ReportRequestor.hpp"

ReportRequestor::ReportRequestor(sockaddr_in &inetAddress, int heartbeatTimeout) : mTimeoutTimer(heartbeatTimeout)
{
    mIPAddr = inetAddress;
}

sockaddr_in ReportRequestor::getIpAddr() const
{
    return mIPAddr;
}

bool ReportRequestor::isExpired()
{
    return mTimeoutTimer.isTimedOut();
}

void ReportRequestor::pumpHeartbeat()
{
    mTimeoutTimer.reset();
}

int ReportRequestor::getHeartbeatPeriod() const
{
    return mTimeoutTimer.getTimeoutPeriod();
}

bool ReportRequestor::operator==(const ReportRequestor &other) const
{
    return mIPAddr.sin_addr.s_addr == other.mIPAddr.sin_addr.s_addr;
}