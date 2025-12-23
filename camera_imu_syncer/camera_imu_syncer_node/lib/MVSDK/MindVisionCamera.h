#ifndef MINDVISIONCAMERA_H
#define MINDVISIONCAMERA_H

#include "CameraApi.h"
#include <atomic>
#include <functional>
#include <opencv2/opencv.hpp>
#include <string>

struct CameraParams {
    double exposure = -1; // 曝光时间（微秒），-1表示不设置
    int gainR = -1; // 红色增益，-1表示不设置
    int gainG = -1; // 绿色增益，-1表示不设置
    int gainB = -1; // 蓝色增益，-1表示不设置
    int analogGain = -1; // 模拟增益，-1表示不设置
    int gamma = -1; // 伽马值，-1表示不设置
    int contrast = -1; // 对比度，-1表示不设置
    int saturation = -1; // 饱和度，-1表示不设置
    int triggerMode = -1; // 触发模式，-1表示不设置 (0:连续 1:软触发 2:硬触发)
};

class MindVisionCamera {
public:
    // 帧回调函数类型
    typedef std::function<void(tSdkFrameHead* pHead, BYTE* pBuffer,int timeDiff)> FrameCallback;

    // 错误回调函数类型
    typedef std::function<void(const std::string& message, CameraSdkStatus code)> ErrorCallback;

    MindVisionCamera();
    ~MindVisionCamera();

    // 统一的打开相机接口
    // configPath: 配置文件路径（必需）
    // frameCallback: 帧回调函数（必需）
    // params: 额外参数（可选，nullptr表示不设置）
    // errorCallback: 错误回调（可选）
    // index: 相机索引（默认0）
    bool open(
        const std::string& configPath,
        FrameCallback frameCallback,
        const CameraParams* params = nullptr,
        ErrorCallback errorCallback = nullptr,
        int index = 0
    );

    void close();

    // 工具方法
    bool checkCameraStatus();
    void printAllParams();

    // 状态查询
    bool isConnected() const {
        return m_isConnected;
    }
    cv::Size getImageSize() const {
        return m_imageSize;
    }
    CameraHandle m_hCamera;

private:
    cv::Size m_imageSize;
    tSdkCameraCapbility m_capability;

    FrameCallback m_frameCallback;
    ErrorCallback m_errorCallback;

    std::atomic<bool> m_isConnected { false };

    // 内部方法
    bool initCamera(int index);
    bool loadConfig(const std::string& configPath);
    bool setParams(const CameraParams& params);
    void printCameraInfo();
    void reportError(const std::string& funcName, CameraSdkStatus status);

    // 静态回调函数
    static void
    frameCallback_static(CameraHandle hCamera, BYTE* pBuffer, tSdkFrameHead* pHead, PVOID pContext);
    static void
    connectionStatusCallback_static(CameraHandle hCamera, UINT uMsg, UINT uParam, PVOID pContext);
};

// 错误检查宏
#define CHECK_STATUS(func_call) \
    do { \
        CameraSdkStatus status = func_call; \
        if (status != CAMERA_STATUS_SUCCESS) { \
            reportError(#func_call, status); \
            return false; \
        } \
    } while (0)

#define REPORT_ERROR(func_call) \
    do { \
        CameraSdkStatus status = func_call; \
        if (status != CAMERA_STATUS_SUCCESS) { \
            reportError(#func_call, status); \
        } \
    } while (0)

#endif // MINDVISIONCAMERA_H
