#include "ReportRequestorSet.hpp"


ReportRequestorSet::ReportRequestorSet()
{
    
}
ReportRequestorSet::~ReportRequestorSet()
{
    for (auto &rPair : mRequestorSet)
    {
        delete rPair.second;
    }
}

bool ReportRequestorSet::add(sockaddr_in &ipAddr)
{
    std::lock_guard<std::mutex> lock(mRequestorLock);
    if (!mRequestorSet.count(ipAddr.sin_addr.s_addr))
    {
        mRequestorSet[ipAddr.sin_addr.s_addr] = new ReportRequestor(ipAddr);
        return true;
    }
    else
    {
        mRequestorSet[ipAddr.sin_addr.s_addr]->pumpHeartbeat();
    }
    return false;
}

void ReportRequestorSet::removeExpiredEntries()
{
    std::lock_guard<std::mutex> lock(mRequestorLock);
    for(auto it = mRequestorSet.begin(); it != mRequestorSet.end(); )
    {
        if (it->second->isExpired()) {
            delete it->second;
            it = mRequestorSet.erase(it);
        } else {
            ++it;
        }
    }
}

void ReportRequestorSet::forEach(std::function<void(ReportRequestor*)> action)
{
    std::lock_guard<std::mutex> lock(mRequestorLock);
    for (auto &rPair : mRequestorSet)
    {
        action(rPair.second);
    }
}