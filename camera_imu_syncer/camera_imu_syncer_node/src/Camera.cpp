#include "Camera.hpp"
#include <CameraApi.h>
Camera::Camera(std::shared_ptr<Syncer> _syncer): syncer(_syncer) {
    CameraParams extraParams;
    extraParams.exposure = 2000;
    extraParams.analogGain = 128;
    extraParams.triggerMode = 2;
    frameCallback = [this](tSdkFrameHead* pHead, BYTE* pBuffer) {
        std::shared_ptr<Syncer::CameraInfo> cameraInfo =
            std::make_shared<Syncer::CameraInfo>(pHead, pBuffer);
        syncer->addCameraFrame(cameraInfo);
    };
    camera.open("/home/helios/Desktop/0409_hard.config", frameCallback, &extraParams);
    syncer->hCamera = camera.m_hCamera;
}
Camera::~Camera() {
    camera.close();
}
