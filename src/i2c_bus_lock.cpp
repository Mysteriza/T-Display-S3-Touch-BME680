#include "i2c_bus_lock.h"

namespace
{
    SemaphoreHandle_t g_i2c_mutex = nullptr;
}

SemaphoreHandle_t acquireSharedI2cMutex()
{
    if (g_i2c_mutex == nullptr)
    {
        g_i2c_mutex = xSemaphoreCreateMutex();
    }
    return g_i2c_mutex;
}

ScopedI2cBusLock::ScopedI2cBusLock(TickType_t timeout_ticks)
{
    mutex_ = acquireSharedI2cMutex();
    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, timeout_ticks) == pdTRUE))
    {
        owns_lock_ = true;
    }
}

ScopedI2cBusLock::~ScopedI2cBusLock()
{
    if (owns_lock_ && (mutex_ != nullptr))
    {
        xSemaphoreGive(mutex_);
    }
}

bool ScopedI2cBusLock::ownsLock() const
{
    return owns_lock_;
}
