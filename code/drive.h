#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include <stdlib.h>
#include "pathfinding.h"
#include <chrono>
#include "bno055_driver.h"

class DriveBase {
    public:

    DriveBase(const char* port);
    ~DriveBase();

    private:
    
    struct Response {
        uint8_t header = 0;
        uint8_t ack = 0;
        int data;
    };

    int serial_port;
    std::thread process;
    std::thread comms;

    void Write(uint8_t cmd, uint8_t* data, int len);
    Response Read(int timeout);
    void PathLoop();

    Bno055* imu;
    volatile float heading_offset = 0;

    std::mutex writing_mutex;
    float current_distance = 0;
    Node::NodeReedSheps current_step;
    volatile int max_drive_time = 100;
    std::vector<Node::NodeReedSheps> current_path;
    std::chrono::_V2::system_clock::time_point last_command;
    std::atomic_bool new_command = false;



    std::mutex comms_mutex;
    volatile int encoder = 0;
    volatile float turn = 0;
    volatile float drive = 0;
    float accel = 0.25;
    volatile uint8_t led = 25;
    volatile uint8_t button = 0;
    volatile float current_steer = 0;
    std::atomic_bool has_reading = false;

    void CommandLoop();
    float dx = 0;
    float dy = 0;
    float da = 0;

    float min_radius = 0.35f;
    float x;
    float y;
    float a;

    float path_x;
    float path_y;
    float path_a;

    public:

    float last_steer = 0;
    float last_speed = 0;
    bool SendCommand(float drive_set, float turn_set);
    void DrivePath(std::vector<Node::NodeReedSheps>& path, int max_drive_time, float x, float y, float a);
    Node::NodeReedSheps GetCurrentNode();
    void BindImu(Bno055* imu);
    float LastSpeed();
    double EncoderAngle();
    Point GetDelta(float angle);
    void CalculateClosestPoint(std::vector<Node::NodeReedSheps> &path, float& d_min, float &a_min, float& point_x, float& point_y, float k, float &till_end, float delta_x, float delta_y, float delta_a);
    void UpdatePosition(float x, float y, float a);
    void SetHeadingOffset(float offset);
    float GetCurrentSteer();
    bool IsButtonPressed();
    void SetLedPWM(uint8_t pwm);
};