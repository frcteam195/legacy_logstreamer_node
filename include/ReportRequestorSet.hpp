#pragma once

#include <map>
#include <arpa/inet.h>
#include <stdint.h>
#include <mutex>
#include <functional>

#include "ReportRequestor.hpp"

class ReportRequestorSet
{
private:
    std::map<uint32_t, ReportRequestor*> mRequestorSet;
    std::mutex mRequestorLock;

public:
    ReportRequestorSet();
    ~ReportRequestorSet();
    bool add(sockaddr_in &ipAddr);
    void removeExpiredEntries();
    void forEach(std::function<void(ReportRequestor*)> action);
};