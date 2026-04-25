#include <cstdint>
#include <algorithm>
#include <cstring>
#include "stubborn_receiver.h"

StubbornReceiver::StubbornReceiver()
{
    ResetState();
    data = nullptr;
    length = 0;
}

void StubbornReceiver::setMaxPackageIndex(uint8_t maxPackageIndex)
{
    if (this->maxPackageIndex != maxPackageIndex)
    {
        this->maxPackageIndex = maxPackageIndex;
        ResetState();
    }
}

void StubbornReceiver::ResetState()
{
    currentPackage = 1;
    currentOffset = 0;
    telemetryConfirm = false;
    finishedData = false;
}

bool StubbornReceiver::GetCurrentConfirm()
{
    return telemetryConfirm;
}

void StubbornReceiver::SetDataToReceive(uint8_t* dataToReceive, uint8_t maxLength)
{
    length = maxLength;
    data = dataToReceive;
    currentPackage = 1;
    currentOffset = 0;
    finishedData = false;
}

void StubbornReceiver::ReceiveData(uint8_t const packageIndex, uint8_t const * const receiveData, uint8_t dataLen)
{
    if (packageIndex == maxPackageIndex)
    {
        telemetryConfirm = !telemetryConfirm;
        currentPackage = 1;
        currentOffset = 0;
        finishedData = false;
        return;
    }

    if (finishedData || data == nullptr)
    {
        return;
    }

    bool acceptData = false;
    if (packageIndex == 0 && currentPackage > 1)
    {
        acceptData = true;
        finishedData = true;
    }
    else if (packageIndex == currentPackage)
    {
        acceptData = true;
    }
    else if (packageIndex == 1 && currentPackage > 1)
    {
        currentPackage = 1;
        currentOffset = 0;
        acceptData = true;
    }

    if (acceptData)
    {
        uint8_t len = std::min((uint8_t)(length - currentOffset), dataLen);
        memcpy(&data[currentOffset], receiveData, len);
        currentPackage++;
        currentOffset += len;
        telemetryConfirm = !telemetryConfirm;
    }
}

bool StubbornReceiver::HasFinishedData()
{
    return finishedData;
}

void StubbornReceiver::Unlock()
{
    if (finishedData)
    {
        currentPackage = 1;
        currentOffset = 0;
        finishedData = false;
    }
}
