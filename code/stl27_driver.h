#pragma once
#include "Point.h"
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "bno055_driver.h"


class Stl27_lidar {

    public:
    Stl27_lidar(const char* port);
    ~Stl27_lidar();

    private:
    int serial_port;
    std::atomic_bool reading = true;

    std::mutex writing_mutex;
    std::thread process;
    std::vector<LidarPoint*> resoult;
    std::vector<Point> points;
    std::vector<LidarPoint*> last_res;
    std::chrono::_V2::high_resolution_clock::time_point last_time;
    int last_angle = 0;

    bool Read();
    void Parse(uint8_t* packet);
    public:
    
    volatile bool ready = false;
    volatile bool ready_polar = false;
    void DisplayPoints();
    void DisplayPoints(cv::Mat base, int zoom, cv::Scalar color);
    bool FirstReading(int timeout);
    bool GetReading(int len, std::vector<Point>& res);
    bool GetReadingPolar(std::vector<LidarPoint>& res, bool wait_for_new);
    void GetBilateralFiltered(std::vector<Point> *dst, Bno055& imu, bool wait_for_new);
    void GetBilateralFiltered(std::vector<LidarPoint> *dst, Bno055& imu, bool wait_for_new);
};