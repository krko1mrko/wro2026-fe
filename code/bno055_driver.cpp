#include "bno055_driver.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <math.h>
#include "bno055_registers.h"
#include "Config.h"
#include "consts.h"


Bno055::Bno055(const char* port) {

    std::cout << "\n===============================================\n";
    std::cout << "Initilizing BNO055 imu on port: " << port << "\n";

    serial_port = open(port, O_RDWR);
    if (serial_port == -1) {
        std::cout << "\nERROR: Unable to open port: " << port << " with errno: " << errno << " \n";
        return;
    }

    std::cout << "Configuring serial port: \n";
    
    termios config;
    tcgetattr(serial_port, &config);
    cfmakeraw(&config);
    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);
    std::cout << "\tBaudrate: 115200\n";

    config.c_cc[VMIN]  = 0;
    config.c_cc[VTIME] = 0;
    std::cout << "\tTimeout: 0\n";

    config.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8);
    config.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB);
    config.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                   ISIG | IEXTEN);
    config.c_oflag &= (tcflag_t) ~(OPOST);
    config.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);
    std::cout << "\tc_cflag configured\n";
    std::cout << "\tc_lflag configured\n";
    std::cout << "\tc_iflag configured\n";
    std::cout << "\tc_oflag configured\n";
    
    tcsetattr(serial_port, TCSANOW, &config);
    std::cout << "Apllying config\n";

    tcflush(serial_port, TCIFLUSH);
    std::cout << "Flusing old data\n";

    ConfigDevice();
    process = std::thread(&Bno055::Update, this);

    std::cout << "Starting read on thread: " << process.get_id() << "\n";

    std::cout << "===============================================\n\n";
}

Bno055::~Bno055() {

    reading = false;
    std::cout << "Stoping imu data reading\n";
    if (process.joinable()) {
        process.join();
    }
    std::cout << "Closing imu serial port\n";
    close(serial_port);
}

void Bno055::ConfigDevice() {

    std::cout << "Configuring BNO055: \n";

    std::cout << "\tSetting operating mode to CONFIGMODE: ";
    uint8_t payload =  0b00000000;
    Write(0x00, OPR_MODE_PG0, 1, &payload);
    Bno055::Response ret = Read(100);
    if (ret.status != BNO_WRITE_SUCCESS) {
        if (ret.header == 0) {
            std::cout << "Bno055 Error: TIMEOUT \n";   
        }
        else {
            std::cerr << "\tBno055 Error " << (int)ret.status << " writing to register:" << OPR_MODE_PG0 << "\n";
        }
    }
    else {
        std::cout << "succes\n";
    }
    
    std::cout << "\tSetting power mode to NORMAL: ";
    payload = 0b00000000;
    Write(0x00, PWR_MODE_PG0, 1, &payload);
    ret = Read(100);
    if (ret.status != BNO_WRITE_SUCCESS) {
        if (ret.header == 0) {
            std::cout << "Bno055 Error: TIMEOUT \n";   
        }
        else {
            std::cerr << "\tBno055 Error " << (int)ret.status << " writing to register:" << PWR_MODE_PG0 << "\n";
        }
    }
    else {
        std::cout << "succes\n";
    }
    
    std::cout << "\tSetting unit to DEGREES: ";
    payload =  0b00000000;
    Write(0x00, UNIT_SEL_PG0, 1, &payload);
    ret = Read(100);
    if (ret.status != BNO_WRITE_SUCCESS) {
        if (ret.header == 0) {
            std::cout << "Bno055 Error: TIMEOUT \n";   
        }
        else {
            std::cerr << "\tBno055 Error " << (int)ret.status << " writing to register:" << UNIT_SEL_PG0 << "\n";
        }
    }
    else {
        std::cout << "succes\n";
    }
    

    std::cout << "\tSetting operating mode to IMU: ";
    payload =  0b00001000;
    Write(0x00, OPR_MODE_PG0, 1, &payload);
    ret = Read(100);
    if (ret.status != BNO_WRITE_SUCCESS) {
        if (ret.header == 0) {
            std::cout << "Bno055 Error: TIMEOUT \n";   
        }
        else {
            std::cerr << "\tBno055 Error " << (int)ret.status << " writing to register:" << OPR_MODE_PG0 << "\n";
        }
    }
    else {
        std::cout << "succes\n";
    }
    
    std::cout << std::endl;
}

void Bno055::Write(uint8_t cmd, uint8_t adr, uint8_t len, uint8_t* data) {
    
    uint8_t* buf = (uint8_t*)malloc((4 + (cmd == 0x00 ? len : 0)) * sizeof(uint8_t));
    buf[0] = 0xAA;
    buf[1] = cmd;
    buf[2] = adr;
    buf[3] = len;
    if (len != 0 && cmd == 0x00){
        memcpy(buf + 4, data, len);
    }
    int err = write(serial_port, buf, 4 + (cmd == 0x00 ? len : 0));
    if (err < 0) {
        std::cout << "\tBno055 Error writing to BNO055 port\n";
    }
    free(buf);
}

Bno055::Response Bno055::Read(int timeout) {
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t header[2];
    int i = 0;
    while (i < 2){
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() > timeout) {
            header[0] = 0;
            break;
        }
        int ret = read(serial_port, header + i, 1);
        if (ret <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        i += ret;
        
    }

    Bno055::Response response = {};

    if (header[0] == 0xEE) {
        response.header = header[0];
        response.status = header[1];
        response.len = 0;
        response.data = nullptr;
    }
    else if (header[0] == 0xBB) {
        response.status = BNO_WRITE_SUCCESS;
        response.header = header[0];
        response.len = header[1];
        response.data = (uint8_t*)malloc(response.len * sizeof(uint8_t));
        int j = 0;
        while (j < response.len) {
            int ret = read(serial_port, response.data + j, response.len - j);
            if (ret <= 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            j += ret;
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_AFTER_READ_BNO));
    return response;
}

void Bno055::Update() {
    while (reading) {
        Write(0x01, EUL_HEADING_LSB_PG0, 2, nullptr);
        Bno055::Response ret = Read(1000);
        
        Write(0x01, GYRO_DATA_Z_LSB_PG0, 2, nullptr);
        Bno055::Response ret2 = Read(1000);
        
        reading_mutex.lock();
        if (ret.status != BNO_WRITE_SUCCESS) {
            std::cerr << "Bno055 Error " << (int)ret.status << "writing to register:" << EUL_HEADING_LSB_PG0 << "\n";
        }
        else {
            uint16_t angle;
            memcpy(&angle, ret.data, ret.len);
            atomic_heading = (float)angle / 16.f;    
        }
        if (ret2.status != BNO_WRITE_SUCCESS) {
            std::cerr << "Bno055 Error " << (int)ret2.status << "writing to register:" << GYRO_DATA_Z_LSB_PG0 << "\n";
        }
        else {
            int16_t speed;
            memcpy(&speed, ret2.data, ret2.len);
            
            float speedf = (float)speed / 16.f * PI / 180.f;
            
            AngularVelocityTimestamped at = {speedf, std::chrono::high_resolution_clock::now()};
            
            if (reading_history.size() > 20) {
                reading_history.erase(reading_history.begin());
            }
            reading_history.push_back(at);
        }

        reading_mutex.unlock();
    }
}

float Bno055::GetHeading() {
    return atomic_heading * PI / 180;
}

float Bno055::GetSpeed() {
    reading_mutex.lock();
    float av = reading_history[reading_history.size() - 1].angular_velocity;
    reading_mutex.unlock();
    return av;
}

float Bno055::GetSpeedAtTime(std::chrono::_V2::high_resolution_clock::time_point t) {
    reading_mutex.lock();
    float avb;
    int tb;
    bool hasb = false;
    for (int i = 0; i < reading_history.size(); i++) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(reading_history[i].stamp - t).count() < 0) {
            avb = reading_history[i].angular_velocity;
            tb = std::chrono::duration_cast<std::chrono::microseconds>(reading_history[i].stamp - t).count();
            hasb = true;
        } 
    }
    float avt;
    int tt;
    bool hast = false;
    for (int i = reading_history.size() - 1; i >= 0; i--) {
        if (std::chrono::duration_cast<std::chrono::microseconds>(reading_history[i].stamp - t).count() > 0) {
            avt = reading_history[i].angular_velocity;
            tt = std::chrono::duration_cast<std::chrono::microseconds>(reading_history[i].stamp - t).count();
            hast = true;
        } 
    }
    reading_mutex.unlock();

    if (hasb && !hast) {
        return avb;
    }
    if (!hasb && hast) {
        return avt;
    }
    if (!hasb && !hast) {
        return 0;
    }

    double d = tt - tb;

    float av = -avt * (double)tb / d + avb * (double)tt / d;
    return av;
}