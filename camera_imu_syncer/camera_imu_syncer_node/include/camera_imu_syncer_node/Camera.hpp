#pragma once
#include "MindVisionCamera.h"
#include "Syncer.hpp"

class Camera {
private:
    std::shared_ptr<Syncer> syncer;

    MindVisionCamera camera;
    std::function<void(tSdkFrameHead* pHead, BYTE* pBuffer, int timeDiff)> frameCallback;

public:
    std::mutex mtx;
    std::queue<cv::Mat> frames;
    std::condition_variable cv;

    Camera(std::shared_ptr<Syncer> _syncer);
    ~Camera();
};
