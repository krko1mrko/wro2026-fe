#pragma once
#include <cstdint>
#include <stdlib.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>

#define BNO_WRITE_SUCCESS 0x01
#define BNO_READ_FAIL 0x02
#define BNO_WRITE_FAIL 0x03
#define BNO_REGMAP_INVALID_ADDRESS 0x04
#define BNO_REGMAP_WRITE_DISABLED 0x05
#define BNO_WRONG_START_BYTE 0x06
#define BNO_BUS_OVERRUN_ERROR 0x07
#define BNO_MAX_LENGTH_ERROR 0x08
#define BNO_MIN_LENGTH_ERROR 0x09
#define BNO_RECEIVE_CHARACTER_TIMEOUT 0x0A

class Bno055 {

    public:
    Bno055(const char* port);
    ~Bno055();
    float GetHeading();
    float GetSpeed();
    float GetSpeedAtTime(std::chrono::_V2::high_resolution_clock::time_point t);
    
    struct AngularVelocityTimestamped {
        float angular_velocity;
        std::chrono::_V2::high_resolution_clock::time_point stamp;
    };

    private:
    
    struct Response {
        uint8_t header = 0;
        uint8_t len = 0;
        uint8_t* data = nullptr;
        uint8_t status = 0;

        ~Response() {
            if (data != nullptr) {
                free(data);
            }
        }
    };

    std::atomic_bool reading = true;
    int serial_port;
    std::mutex reading_mutex;
    std::vector<AngularVelocityTimestamped> reading_history;
    std::atomic<float> atomic_heading;
    std::thread process;

    void ConfigDevice();
    void Parse(uint8_t* packet);
    void Write(uint8_t cmd, uint8_t adr, uint8_t len, uint8_t* data);
    Response Read(int timeout);
    void Update();
};
