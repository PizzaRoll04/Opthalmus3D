#include "DataStreamer.h"

DataStreamer::DataStreamer() {}

DataStreamer::~DataStreamer() {}

bool DataStreamer::addIMUCallback(std::function<void(IMUData[])> callback)
{
    _callbacks_imu.push_back(callback);
    return true;
}

bool DataStreamer::addFrameCallback(std::function<void(Frame[])> callback)
{
    _callbacks_frame.push_back(callback);
    return true;
}

bool DataStreamer::executeFrameCallbacks(Frame frames[])
{
    if (!_b_enable_callbacks) {
        fprintf(stderr, "[DataStreamer] attempted to call executeFrameCallbacks but callbacks bool is false.");
    }
    for (std::function<void(Frame[])> func : _callbacks_frame) {
        func(frames);
    }
}

bool DataStreamer::executeIMUCallbacks(IMUData imu_data[])
{
    if (!_b_enable_callbacks) {
        fprintf(stderr, "[DataStreamer] attempted to call executeIMUCallbacks but callbacks bool is false.");
    }
    for (std::function<void(IMUData[])> func : _callbacks_imu) {
        func(imu_data);
    }
}

void DataStreamer::enableCallbacks(bool enable)
{
    _b_enable_callbacks = enable;
}
