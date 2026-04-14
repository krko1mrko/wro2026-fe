#include "stl27_driver.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <math.h>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "consts.h"

Stl27_lidar::Stl27_lidar(const char* port){
    std::cout << "\n===============================================\n";
    std::cout << "Initilizing STL27L lidar on port: " << port << "\n";
    serial_port = open(port, O_RDWR);
    if (serial_port == -1) {
        std::cout << "\nERROR: Unable to open port: " << port << " with errno: " << errno << " \n";
        return;
    }

    std::cout << "Configuring serial port: \n";
    termios config;
    tcgetattr(serial_port, &config);
    cfmakeraw(&config);
    cfsetispeed(&config, B921600);
    cfsetospeed(&config, B921600);
    std::cout << "\tBaudrate: 921600\n";

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

    process = std::thread(&Stl27_lidar::Read, this);
    std::cout << "Starting read on thread: " << process.get_id() << "\n";

    std::cout << "===============================================\n\n";
}
Stl27_lidar::~Stl27_lidar(){
    reading = false;
    std::cout << "Stoping lidar data reading\n";
    if (process.joinable()) {
        process.join();
    }
    std::cout << "Closing lidar serial port\n";
    close(serial_port);
    
}
static const uint8_t CrcTable[256] ={
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};
uint8_t CRC(uint8_t* packet, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++){
        crc = CrcTable[(crc ^ packet[i]) & 0xff];
    }
    return crc;
}

struct __attribute__((packed)) LidarPointData{
    uint16_t distance;
    uint8_t intensity;
};
struct LidarPacket {
    uint8_t header;
    uint8_t var_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointData data[12];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc;
};
void Stl27_lidar::DisplayPoints() {
    writing_mutex.lock();
    cv::Mat img = cv::Mat(512, 512, CV_8U, cv::Scalar(0,0,0,0));
    
    for (LidarPoint* p: last_res) {
        if(p->x > 2 || p->y > 2 || p->x < -2 || p->y < -2) {
            continue;
        }
        img.at<uchar>(p->x * 128 + 256, p->y * 128 + 256) = p->confidance;
    }
    cv::imshow("lidardisplay" ,img);
    writing_mutex.unlock();
}
void Stl27_lidar::DisplayPoints(cv::Mat base, int zoom, cv::Scalar color) {
    writing_mutex.lock();

    for (LidarPoint* p: last_res) {
        if(p->x > 2 || p->y > 2 || p->x < -2 || p->y < -2) {
            continue;
        }
        base(cv::Rect(std::clamp((int)(p->x * zoom + 256), 0, 511), std::clamp((int)(p->y * zoom + 256), 0, 511), 1, 1)) = color;
    }
    cv::imshow("lidardisplay" ,base);
    writing_mutex.unlock();
}
void Stl27_lidar::Parse(uint8_t* packet) {
    LidarPacket data;
    memcpy(&data, packet, sizeof(LidarPacket));

    if (data.start_angle - last_angle < -18000) {
        auto start_time = std::chrono::high_resolution_clock::now();
        writing_mutex.lock();
        points.clear();
        for( LidarPoint* p : last_res) {
            delete p;
        }
        last_res.clear();
        for( LidarPoint* p : resoult) {
            p->te = start_time;
            p->ts = last_time;
            last_res.push_back(p);
            points.push_back(Point(p->x, p->y));
        }
        resoult.clear();    
        ready = true;
        ready_polar = true;
        writing_mutex.unlock();
        last_time = start_time;
    }
    for (int i = 0; i < 12; i++) {
        
        float angle = (float)data.end_angle * i / 11.f + (float)data.start_angle * (1 - i / 11.f);
        angle = angle * 0.01 / 360 * 2 * PI;
        if (data.data[i].intensity > 40 && data.data[i].distance < 25000 && data.data[i].distance > 50){
            LidarPoint* p = new LidarPoint((float)data.data[i].distance / 1000.f ,angle, data.data[i].intensity);
            resoult.push_back(p);
        }
    }
    last_angle = data.start_angle;
}
bool Stl27_lidar::Read() {

    auto start = std::chrono::high_resolution_clock::now();
    int packet_count = 0;
    int packet_index = 0;
    uint8_t byte[128];
    const int buffer_size = 512;
    uint8_t buffer[buffer_size];
    uint8_t packet[47];
    int available = 0;
    int buffer_load_index = 0;
    int buffer_read_index = 0;
    size_t recived = 0;
    auto last_time = std::chrono::high_resolution_clock::now();

    while(this->reading) {
        int ret = read(this->serial_port, byte, 128);
        recived += ret;
        if (ret <= 0) {
            continue;
        }
        int cpy_size = std::min(ret, buffer_size - buffer_load_index);
        std::memcpy(buffer + buffer_load_index, byte, cpy_size); 
        available += cpy_size;
        buffer_load_index += cpy_size;
        buffer_load_index = buffer_load_index % buffer_size;
        ret -= cpy_size;
        if (ret != 0) {
            std::memcpy(buffer + buffer_load_index, byte + cpy_size, ret); 
            available += ret;
            buffer_load_index += ret;
            buffer_load_index = buffer_load_index % buffer_size;
            ret -= ret;        
        }

        int read_available = available;
        for (;;) {
            if (available < 47) {
                break;
            }
            if (packet_index == 0) {
                if (buffer[(buffer_read_index + packet_index) % buffer_size] == 0x54) {
                    packet[packet_index] = buffer[(buffer_read_index + packet_index) % buffer_size];
                    packet_index++;
                }
                else {
                    buffer_read_index++;
                    available--;
                    packet_index = 0;
                }
            }
            else if (packet_index == 1 && false) {
                if (buffer[(buffer_read_index + packet_index) % buffer_size] == 0x2C) {
                    packet[packet_index] = buffer[(buffer_read_index + packet_index) % buffer_size];
                    packet_index++;
                }
                else {
                    buffer_read_index++;
                    available--;
                    packet_index = 0;
                }
            }
            else {
                    packet[packet_index] = buffer[(buffer_read_index + packet_index) % buffer_size];
                    packet_index++;
            }
        
        if(packet_index >= 47) {
            packet_index = 0;
            if(CRC(packet, 46) != packet[46]) {
                buffer_read_index++;
                available--;
                std::cout << "\nfailed CRC " << (int)CRC(packet, 46) << "  " << (int)packet[46] << "\n";
                continue;
            }
            buffer_read_index += 47;
            available -= 47;
            packet_count++;
            this->Parse(packet);

        }
        buffer_read_index = buffer_read_index % buffer_size;
        /*
        auto end = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(end-last_time).count() > 1000) {
            std::cout << "data throughput " << recived / (float)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << "\n";
            last_time = end;
            std::cout << "packet recieved " << (float)std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() / packet_count << "\n";
            
        }
        */
    }

    }
    return true;
}

bool Stl27_lidar::GetReading(int len, std::vector<Point>& res) {
    
    
    writing_mutex.lock();
    if (!ready) {
        writing_mutex.unlock();
        return false;
    }
    if ( len == -1) {
        if (points.size() == 0) {
            writing_mutex.unlock();
            return false;
        }
        
        res = points;
        ready = false;
        writing_mutex.unlock();
        return true;
    }
    if (points.size() > len) {
        points.erase(points.begin() + len, points.end());
    }
    if (points.size() < len) {
        if (points.size() == 0) {
            writing_mutex.unlock();
            return false;
        }
        int i = len - points.size();
        int s = points.size();
        for (int j = 0; j < i; j++) {
            points.push_back(points[int((float)s/(float)i * j)]);
        }
        Point p = points[points.size() - 1];
        //TODO This is a fix that sould be done in the opencl kernel by discarding points after end

        while (points.size() < len) {
            points.push_back(p);
        }
    }
    res = points;
    ready = false;
    writing_mutex.unlock();
    return true;
}

bool Stl27_lidar::GetReadingPolar(std::vector<LidarPoint>& res, bool wait_for_new) {
    
    
    writing_mutex.lock();
    if (wait_for_new) {
        if (!ready_polar) {
            writing_mutex.unlock();
            return false;
        }
    }
    if (last_res.size() == 0) {
        writing_mutex.unlock();
        return false;
    }
    for (LidarPoint* p : last_res) {
        LidarPoint q = LidarPoint(*p);
        res.push_back(q);
    }
    if (wait_for_new) {
        ready_polar = false;
    }
    writing_mutex.unlock();
    return true;
}

bool Stl27_lidar::FirstReading(int timeout) {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "Waiting for first lidar reading\n";
            
    while (!ready) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() > timeout) {
            std::cout << "Timout of " << timeout << "excided\n"; 
            return false;
        }
    }
    std::cout << "First reading recieveed in: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() << "ms\n";
    return true;
}

void Stl27_lidar::GetBilateralFiltered(std::vector<Point> *dst, Bno055& imu, bool wait_for_new = true) {
        std::vector<LidarPoint> reading;
        float start = PI;
        float end = 2 * PI;
        while(!GetReadingPolar(reading, wait_for_new)){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::sort(reading.begin(), reading.end(), [](const LidarPoint& a, const LidarPoint& b) -> bool {return a.angle < b.angle; });
        
  
        int si = -1;
        int ei = -1;
        for (int i = 0; i < reading.size(); i++) {
            if (reading[i].angle > start && si == -1) {
                si = i;
            }
            if (reading[i].angle < end) {
                ei = i;
            }
        }

        reading.erase(reading.begin() + si, reading.begin() + ei);

        double duration = std::chrono::duration_cast<std::chrono::microseconds>(reading[0].te - reading[0].ts).count();
        double correction = 0;
        double last_time = duration;
        for (int i = reading.size() - 1; i >= 0; i--) {
            double td = duration * reading[i].angle / 2 / PI;
            auto t = reading[i].ts + std::chrono::microseconds((long)td);
            float av = imu.GetSpeedAtTime(t);
            correction += av * (last_time - td) / 1000000.0;
            last_time = td;
            reading[i].angle += correction;
            reading[i].x = cos(reading[i].angle) * reading[i].distance;
            reading[i].y = sin(reading[i].angle) * reading[i].distance;
        }

        std::sort(reading.begin(), reading.end(), [](const LidarPoint& a, const LidarPoint& b) -> bool {return a.angle < b.angle; });

        for (int i = 1; i < reading.size() - 1; i++) {
            Point tp = Point(reading[i].x, reading[i].y);
            if (sqrt(tp.DistanceSq(Point(reading[i+1].x , reading[i+1].y))) > 0.02 &&
                sqrt(tp.DistanceSq(Point(reading[i-1].x , reading[i-1].y))) > 0.02) {
                    reading.erase(reading.begin() + i);
                    i--;
            }
        }
      
        if (reading.size() <= 0) {
            return;
        }


        cv::Mat data = cv::Mat(cv::Size(reading.size(), 1), CV_32F);
        cv::Mat tempdata = cv::Mat(cv::Size(reading.size(), 1), CV_32F);
        
        std::vector<float> temp;
        for (LidarPoint& p : reading) {
            temp.push_back(p.distance);
        }
        memcpy(data.data, temp.data(), reading.size() * sizeof(float)); 
        cv::bilateralFilter(data, tempdata, 25, 0.2, 10);
        memcpy(temp.data(), tempdata.data, reading.size() * sizeof(float)); 
        
        for (int i = 0; i < reading.size(); i++) {
            Point p = Point(
            cos(reading[i].angle) * temp[i],
            sin(reading[i].angle) * temp[i]);
            dst->push_back(p);
        }
    }

void Stl27_lidar::GetBilateralFiltered(std::vector<LidarPoint> *dst, Bno055& imu, bool wait_for_new) {
        std::vector<LidarPoint> reading;
        float start = PI;
        float end = 2 * PI;
        while(!GetReadingPolar(reading, wait_for_new)){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::sort(reading.begin(), reading.end(), [](const LidarPoint& a, const LidarPoint& b) -> bool {return a.angle < b.angle; });
        
  
        int si = -1;
        int ei = -1;
        for (int i = 0; i < reading.size(); i++) {
            if (reading[i].angle > start && si == -1) {
                si = i;
            }
            if (reading[i].angle < end) {
                ei = i;
            }
        }

        reading.erase(reading.begin() + si, reading.begin() + ei);

        double duration = std::chrono::duration_cast<std::chrono::microseconds>(reading[0].te - reading[0].ts).count();
        double correction = 0;
        double last_time = duration;
        for (int i = reading.size() - 1; i >= 0; i--) {
            double td = duration * reading[i].angle / 2 / PI;
            auto t = reading[i].ts + std::chrono::microseconds((long)td);
            float av = imu.GetSpeedAtTime(t);
            correction += av * (last_time - td) / 1000000.0;
            last_time = td;
            reading[i].angle += correction;
            reading[i].x = cos(reading[i].angle) * reading[i].distance;
            reading[i].y = sin(reading[i].angle) * reading[i].distance;
        }

        std::sort(reading.begin(), reading.end(), [](const LidarPoint& a, const LidarPoint& b) -> bool {return a.angle < b.angle; });

        for (int i = 1; i < reading.size() - 1; i++) {
            Point tp = Point(reading[i].x, reading[i].y);
            if (sqrt(tp.DistanceSq(Point(reading[i+1].x , reading[i+1].y))) > 0.02 &&
                sqrt(tp.DistanceSq(Point(reading[i-1].x , reading[i-1].y))) > 0.02) {
                    reading.erase(reading.begin() + i);
                    i--;
            }
        }
      
        if (reading.size() <= 0) {
            return;
        }


        cv::Mat data = cv::Mat(cv::Size(reading.size(), 1), CV_32F);
        cv::Mat tempdata = cv::Mat(cv::Size(reading.size(), 1), CV_32F);
        
        std::vector<float> temp;
        for (LidarPoint& p : reading) {
            temp.push_back(p.distance);
        }
        memcpy(data.data, temp.data(), reading.size() * sizeof(float)); 
        cv::bilateralFilter(data, tempdata, 25, 0.2, 2);
        memcpy(temp.data(), tempdata.data, reading.size() * sizeof(float)); 
        
        for (int i = 0; i < reading.size(); i++) {
            reading[i].x = cos(reading[i].angle) * temp[i];
            reading[i].y = sin(reading[i].angle) * temp[i];
            dst->push_back(reading[i]);
        }
    }