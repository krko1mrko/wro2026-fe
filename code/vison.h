#pragma once
#include "stl27_driver.h"
#include "bno055_driver.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "signal.h"

class Vision {

    Stl27_lidar* lidar;
    cv::VideoCapture cap;
    Bno055* imu;
    public:

    Vision();
    void AddImu(Bno055* imu);
    void AddLidar(Stl27_lidar* lidar);
    std::vector<Signal> DisplayFrame();
};