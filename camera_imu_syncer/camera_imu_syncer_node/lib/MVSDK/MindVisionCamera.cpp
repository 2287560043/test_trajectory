#include "MindVisionCamera.h"
#include "CameraApi.h"
#include "CameraDefine.h"
#include "CameraStatus.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
MindVisionCamera::MindVisionCamera(): m_hCamera(0), m_imageSize(0, 0), m_capability({}) {
    static std::once_flag sdkInit;
    std::call_once(sdkInit, []() {
        CameraSdkStatus status = CameraSdkInit(1);
        if (status != CAMERA_STATUS_SUCCESS) {
            std::cerr << "[ERROR] SDK initialization failed: " << CameraGetErrorString(status)
                      << std::endl;
        }
    });
}

MindVisionCamera::~MindVisionCamera() {
    close();
}

bool MindVisionCamera::open(
    const std::string& configPath,
    FrameCallback frameCallback,
    const CameraParams* params,
    ErrorCallback errorCallback,
    int index
) {
    if (!frameCallback) {
        std::cerr << "[ERROR] Invalid frame callback function" << std::endl;
        return false;
    }

    m_frameCallback = frameCallback;
    m_errorCallback = errorCallback;

    // 枚举设备
    int deviceCount = CameraEnumerateDeviceEx();
    if (deviceCount <= index) {
        reportError(
            "CameraEnumerateDeviceEx - Camera index " + std::to_string(index) + " not found",
            CAMERA_STATUS_NO_DEVICE_FOUND
        );
        return false;
    }

    // 初始化相机
    CHECK_STATUS(CameraInitEx(index, -1, -1, &m_hCamera));

    // 初始化相机基本设置
    if (!initCamera(index)) {
        CameraUnInit(m_hCamera);
        m_hCamera = 0;
        return false;
    }

    // 加载配置文件
    // if (!loadConfig(configPath)) {
    //     CameraUnInit(m_hCamera);
    //     m_hCamera = 0;
    //     return false;
    // }

    // 如果有额外参数，应用额外参数
    if (params && !setParams(*params)) {
        CameraUnInit(m_hCamera);
        m_hCamera = 0;
        return false;
    }

    // 设置回调函数
    CHECK_STATUS(CameraSetCallbackFunction(m_hCamera, frameCallback_static, this, nullptr));
    REPORT_ERROR(
        CameraSetConnectionStatusCallback(m_hCamera, connectionStatusCallback_static, this)
    );
    printAllParams();
    printCameraInfo();

    m_isConnected = true;

    std::cout << "Camera opened successfully!" << std::endl;
    CHECK_STATUS(CameraPlay(m_hCamera));

    return true;
}

void MindVisionCamera::close() {
    if (m_hCamera) {
        std::cout << "Stopping camera..." << std::endl;

        // 先停止采集
        CameraStop(m_hCamera);

        // 等待一小段时间，让正在处理的回调完成
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 取消回调函数
        CameraSetCallbackFunction(m_hCamera, nullptr, nullptr, nullptr);
        CameraSetConnectionStatusCallback(m_hCamera, nullptr, nullptr);

        // 再等一小段时间确保回调完全退出
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 反初始化
        CameraUnInit(m_hCamera);
        m_hCamera = 0;
        m_isConnected = false;
        std::cout << "Camera closed" << std::endl;
    }
}

bool MindVisionCamera::loadConfig(const std::string& configPath) {
    if (!m_hCamera) {
        reportError("loadConfig - Camera not initialized", CAMERA_STATUS_NOT_INITIALIZED);
        return false;
    }

    std::cout << "Loading camera config: " << configPath << std::endl;
    CHECK_STATUS(CameraReadParameterFromFile(m_hCamera, const_cast<char*>(configPath.c_str())));
    std::cout << "Config loaded successfully!" << std::endl;
    return true;
}

bool MindVisionCamera::setParams(const CameraParams& params) {
    if (!m_hCamera) {
        reportError("setParams - Camera not initialized", CAMERA_STATUS_NOT_INITIALIZED);
        return false;
    }

    std::cout << "Setting camera parameters..." << std::endl;

    if (params.exposure >= 0) {
        CHECK_STATUS(CameraSetExposureTime(m_hCamera, params.exposure));
        std::cout << "Exposure: " << params.exposure << " μs" << std::endl;
    }

    if (params.gainR >= 0 && params.gainG >= 0 && params.gainB >= 0) {
        CHECK_STATUS(CameraSetGain(m_hCamera, params.gainR, params.gainG, params.gainB));
        std::cout << "RGB Gain: R=" << params.gainR << " G=" << params.gainG
                  << " B=" << params.gainB << std::endl;
    }

    if (params.analogGain >= 0) {
        CHECK_STATUS(CameraSetAnalogGain(m_hCamera, params.analogGain));
        std::cout << "Analog Gain: " << params.analogGain << std::endl;
    }

    if (params.gamma >= 0) {
        CHECK_STATUS(CameraSetGamma(m_hCamera, params.gamma));
        std::cout << "Gamma: " << params.gamma << std::endl;
    }

    if (params.contrast >= 0) {
        CHECK_STATUS(CameraSetContrast(m_hCamera, params.contrast));
        std::cout << "Contrast: " << params.contrast << std::endl;
    }

    if (params.saturation >= 0) {
        CHECK_STATUS(CameraSetSaturation(m_hCamera, params.saturation));
        std::cout << "Saturation: " << params.saturation << std::endl;
    }

    if (params.triggerMode >= 0) {
        CHECK_STATUS(CameraSetTriggerMode(m_hCamera, params.triggerMode));
        std::cout << "Trigger Mode: " << params.triggerMode << std::endl;
    }

    std::cout << "Parameters set successfully!" << std::endl;
    return true;
}

bool MindVisionCamera::checkCameraStatus() {
    if (!m_hCamera)
        return false;

    tSdkFrameStatistic statistic;
    if (CameraGetFrameStatistic(m_hCamera, &statistic) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Captured frames: " << statistic.iCapture << std::endl;
        std::cout << "Lost frames: " << statistic.iLost << std::endl;
        return true;
    }
    return false;
}

void MindVisionCamera::printAllParams() {
    if (!m_hCamera) {
        std::cout << "Camera not initialized!" << std::endl;
        return;
    }

    std::cout << "\n========== Current Parameters ==========" << std::endl;
    int bayer2rgbAlgorithm;
    if (CameraGetBayerDecAlgorithm(m_hCamera, 0, &bayer2rgbAlgorithm) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Bayer to RGB Algorithm: " << bayer2rgbAlgorithm << std::endl;
    }
    double exposure;
    if (CameraGetExposureTime(m_hCamera, &exposure) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Exposure:         " << exposure << " μs" << std::endl;
    }

    int gainR, gainG, gainB;
    if (CameraGetGain(m_hCamera, &gainR, &gainG, &gainB) == CAMERA_STATUS_SUCCESS) {
        std::cout << "RGB Gain:         R=" << gainR << " G=" << gainG << " B=" << gainB
                  << std::endl;
    }

    int analogGain;
    if (CameraGetAnalogGain(m_hCamera, &analogGain) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Analog Gain:      " << analogGain << std::endl;
    }

    int gamma;
    if (CameraGetGamma(m_hCamera, &gamma) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Gamma:            " << gamma << std::endl;
    }

    int contrast;
    if (CameraGetContrast(m_hCamera, &contrast) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Contrast:         " << contrast << std::endl;
    }

    int saturation;
    if (CameraGetSaturation(m_hCamera, &saturation) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Saturation:       " << saturation << std::endl;
    }

    int triggerMode;
    if (CameraGetTriggerMode(m_hCamera, &triggerMode) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Trigger Mode:     " << triggerMode << " (";
        switch (triggerMode) {
            case 0:
                std::cout << "Continuous";
                break;
            case 1:
                std::cout << "Software";
                break;
            case 2:
                std::cout << "Hardware";
                break;
            default:
                std::cout << "Unknown";
                break;
        }
        std::cout << ")" << std::endl;
    }

    tSdkImageResolution resolution;
    if (CameraGetImageResolution(m_hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
        std::cout << "Resolution:       " << resolution.iWidth << "x" << resolution.iHeight
                  << std::endl;
    }

    std::cout << "==========================================" << std::endl;
}

bool MindVisionCamera::initCamera(int index) {
    char versionString[64];
    CameraSdkGetVersionString(versionString);
    std::cout << "SDK Version: " << versionString << std::endl;

    CHECK_STATUS(CameraGetCapability(m_hCamera, &m_capability));

    tSdkImageResolution res;
    CHECK_STATUS(CameraGetImageResolution(m_hCamera, &res));
    m_imageSize = cv::Size(res.iWidth, res.iHeight);

    REPORT_ERROR(CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_BGR8));
    REPORT_ERROR(CameraSetNoiseFilter(m_hCamera, FALSE));
    REPORT_ERROR(CameraSetAeState(m_hCamera, FALSE));

    return true;
}

void MindVisionCamera::printCameraInfo() {
    if (!m_hCamera)
        return;

    std::cout << "\n====== Camera Info ======" << std::endl;
    std::cout << "Resolution: " << m_imageSize.width << "x" << m_imageSize.height << std::endl;
    std::cout << "Type: Color Camera" << std::endl;

    std::cout << "\n------ Parameter Ranges ------" << std::endl;
    std::cout << "Exposure: " << m_capability.sExposeDesc.uiExposeTimeMin << " - "
              << m_capability.sExposeDesc.uiExposeTimeMax << " μs" << std::endl;
    std::cout << "Analog Gain: " << m_capability.sExposeDesc.uiAnalogGainMin << " - "
              << m_capability.sExposeDesc.uiAnalogGainMax << std::endl;

    std::cout << "RGB Gain: R[" << m_capability.sRgbGainRange.iRGainMin << "-"
              << m_capability.sRgbGainRange.iRGainMax << "] G["
              << m_capability.sRgbGainRange.iGGainMin << "-" << m_capability.sRgbGainRange.iGGainMax
              << "] B[" << m_capability.sRgbGainRange.iBGainMin << "-"
              << m_capability.sRgbGainRange.iBGainMax << "]" << std::endl;

    std::cout << "=========================\n" << std::endl;
}

void MindVisionCamera::reportError(const std::string& funcName, CameraSdkStatus status) {
    std::string errorMsg;
    if (status != CAMERA_STATUS_SUCCESS) {
        errorMsg = funcName + ": " + CameraGetErrorString(status);
    } else {
        errorMsg = funcName;
    }

    if (m_errorCallback) {
        m_errorCallback(errorMsg, status);
    } else {
        if (status == CAMERA_STATUS_SUCCESS) {
            std::cout << "[INFO] " << errorMsg << std::endl;
        } else {
            std::cerr << "[ERROR] " << errorMsg << " (Code: " << status << ")" << std::endl;
        }
    }
}

void MindVisionCamera::frameCallback_static(
    CameraHandle hCamera,
    BYTE* pBuffer,
    tSdkFrameHead* pHead,
    PVOID pContext
) {
    auto* camera = static_cast<MindVisionCamera*>(pContext);
    if (!camera || !camera->m_frameCallback || !pBuffer || !pHead)
        return;

    // 获取当前系统时间（UTC）
    auto now = std::chrono::system_clock::now();
    
    // 获取相机设备时间戳（单位：微秒）用于计算曝光代码
    UINT high, low;
    CameraGetDevTimeStamp(hCamera, &low, &high);
    uint64_t dev_us = ((uint64_t)high << 32) | low;
    
    // 相机内部时间戳转换为微秒（0.1ms → µs）
    uint64_t frame_internal_us = static_cast<uint64_t>(pHead->uiTimeStamp) * 100;
    
    // 计算相机内部时间戳与设备时间戳的差异（用于曝光时间计算）
    int64_t exposure_offset_us = static_cast<int64_t>(dev_us) - static_cast<int64_t>(frame_internal_us);
    
    // 计算曝光刹那的系统时间：当前系统时间 - 设备时间偏差
    auto exposure_time = now - std::chrono::microseconds(exposure_offset_us + pHead->uiExpTime);
    
    // 从曝光刹那的时间计算GPRMC格式的时间戳
    auto time_t_exposure = std::chrono::system_clock::to_time_t(exposure_time);
    std::tm tm_utc;
    gmtime_r(&time_t_exposure, &tm_utc);

    // 计算当日从00:00:00开始的毫秒数
    // 公式：(时*3600 + 分*60 + 秒) * 1000 + 毫秒
    uint32_t seconds_of_day = tm_utc.tm_hour * 3600 + tm_utc.tm_min * 60 + tm_utc.tm_sec;
    uint32_t milliseconds_of_day = seconds_of_day * 1000;
    
    // 获取曝光刹那时间的毫秒部分
    auto ms_part = std::chrono::duration_cast<std::chrono::milliseconds>(
        exposure_time.time_since_epoch()
    ).count() % 1000;
    
    // 最终时间戳：从当日UTC 00:00:00开始的毫秒计数 (0~86,399,999 ms)
    // 这是曝光刹那的GPRMC时钟
    uint32_t frame_timestamp_ms = milliseconds_of_day + ms_part;
    
    // 验证时间戳范围 (86,399,999 ms = 23:59:59.999)
    if (frame_timestamp_ms >= 86400000) {
        frame_timestamp_ms = frame_timestamp_ms % 86400000;  // 防止越界
    }

    // 检查帧数据有效性
    if (pHead->uBytes == 0 || pHead->iWidth <= 0 || pHead->iHeight <= 0) {
        camera->reportError(
            "frameCallback_static - Invalid frame data",
            CAMERA_STATUS_PARAMETER_INVALID
        );
        return;
    }
    //
    // UINT frameID = -100;
    // CameraGetFrameID(hCamera, &frameID);
    // cv::Mat frame(pHead->iHeight, pHead->iWidth, CV_8UC1, pBuffer);
    //
    // // 分配处理后的图像缓冲区
    // int bufferSize = pHead->iWidth * pHead->iHeight * 3;
    // BYTE* pProcessedBuffer = new BYTE[bufferSize];
    // // 图像处理
    // CameraSdkStatus status =
    //     CameraImageProcessEx(hCamera, pBuffer, pProcessedBuffer, pHead, CAMERA_MEDIA_TYPE_BGR8, 0);
    // // std::cout << "Image processing time: " << ms << " ms" << std::endl;
    // if (status != CAMERA_STATUS_SUCCESS) {
    //     camera->reportError("frameCallback_static - CameraImageProcessEx", status);
    //     delete[] pProcessedBuffer;
    //     return;
    // }
    // std::cout << "Received frame ID: " << frameID << std::endl;
    // // 0拷贝：Mat直接包装数据指针，不进行内存拷贝
    // // Mat不拥有数据所有权，数据在回调返回后释放
    //
    // if (frame.empty()) {
    //     camera->reportError("frameCallback_static - Empty processed frame", CAMERA_STATUS_FAILED);
    //     delete[] pProcessedBuffer;
    //     return;
    // }
    //
    // 调用用户回调，传递0拷贝的Mat
    // 如需在回调外保存frame，必须使用 frame.clone()

    camera->m_frameCallback(pHead, pBuffer, frame_timestamp_ms);

    // 回调返回后立即释放内存
    // delete[] pProcessedBuffer;
}

void MindVisionCamera::connectionStatusCallback_static(
    CameraHandle hCamera,
    UINT uMsg,
    UINT uParam,
    PVOID pContext
) {
    auto* camera = static_cast<MindVisionCamera*>(pContext);
    if (!camera)
        return;

    // 0 表示断线，1 表示连接
    switch (uMsg) {
        case 0: // 断线
            camera->m_isConnected = false;
            camera->reportError(
                "connectionStatusCallback - Camera disconnected",
                CAMERA_STATUS_DEVICE_LOST
            );
            break;

        case 1: // 连接
            camera->m_isConnected = true;
            camera->reportError(
                "connectionStatusCallback - Camera reconnected",
                CAMERA_STATUS_SUCCESS
            );
            break;

        default:
            camera->reportError(
                "Unknown connection status: " + std::to_string(uMsg),
                CAMERA_STATUS_SUCCESS
            );
            break;
    }
}
