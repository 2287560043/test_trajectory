#pragma once
#include <camera_imu_bridge/CameraFrame.hpp>
#include <camera_imu_bridge/ImuFrame.hpp>
class Syncer {
public:
    Syncer();
    void addImuFrame();
    void addCameraFrame();
    ~Syncer();

private:
};
