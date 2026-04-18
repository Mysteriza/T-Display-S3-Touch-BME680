#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

SemaphoreHandle_t acquireSharedI2cMutex();

class ScopedI2cBusLock
{
public:
    explicit ScopedI2cBusLock(TickType_t timeout_ticks);
    ~ScopedI2cBusLock();

    bool ownsLock() const;

private:
    SemaphoreHandle_t mutex_ = nullptr;
    bool owns_lock_ = false;
};
