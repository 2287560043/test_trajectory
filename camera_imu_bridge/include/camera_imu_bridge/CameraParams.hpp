#pragma once
#include <string>
struct cameraParams {
    struct RgbGain {
        int r, g, b;
    };
    std::string camera_name;
    std::string config_file_path;
    std::string camera_info_url;
    bool print_fps;
    int exposure_time;
    int analog_gain;
    RgbGain rgb_gain;
    int saturation;
    int gamma;
    int rotate;
};
