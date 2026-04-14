#include "drive.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include "Config.h"
#include <algorithm>
#include "lookup.h"
#include "pid.h"
#include "consts.h"

DriveBase::DriveBase(const char* port) {
    
    std::cout << "\n===============================================\n";
    std::cout << "Initilizing DriveBase communication on port: " << port << "\n";

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

    comms = std::thread(&DriveBase::CommandLoop, this);
    process = std::thread(&DriveBase::PathLoop, this);
    std::cout << "Starting path loop on thread: " << process.get_id() << "\n";

    std::cout << "===============================================\n\n";
}

DriveBase::~DriveBase(){
    std::cout << "Stopping car\n";
    SendCommand(0,0);
    std::cout << "Closing RP2040 serial port\n";
    close(serial_port);
}

void DriveBase::Write(uint8_t cmd, uint8_t* data, int len) {
    uint8_t* buf = (uint8_t*)malloc((2 + len) * sizeof(uint8_t));
    buf[0] = 0x35;
    buf[1] = cmd;
    memcpy(buf + 2, data, len);
    int err = write(serial_port, buf, 2 + len);
    if (err < 0) {
        std::cout << "\tError writing to RP2040 port\n";
    }
    free(buf);
}

DriveBase::Response DriveBase::Read(int timeout) {
    auto start = std::chrono::high_resolution_clock::now();
    uint8_t data[7] = {};
    int i = 0;
    while (i < 7){
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() > timeout) {
            //data[0] = 0;
            break;
        }

        int ret = read(serial_port, data + i, 1);
        if (ret <= 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        if (i == 0 && data[0] != 0x35) {
            continue;
        }
        i += ret;
    }

    DriveBase::Response response = {};
    if (i < 7) {
        response;
    }

    response.header = data[0];
    response.ack = data[1];
    memcpy(&(response.data), data + 2, 4);        

    this->button = data[6];

    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_AFTER_READ_RP2040));
    return response;
}

bool DriveBase::SendCommand(float drive_set, float turn_set) {
    drive_set /= 2;
    if (drive_set > 0) {
        drive_set /= 0.97;
        drive_set += 0.22;
    }
    else {
        drive_set /= 1.22;
    }

    drive_set = std::clamp(drive_set, -0.4f, 0.62f);
    //drive_set += 0.1f * (drive_set >= 0 ? 1 : -1);
    turn_set = std::clamp(turn_set, -1.f, 1.f);
    comms_mutex.lock();

    this->turn = turn_set;
    this->drive = drive_set;

    comms_mutex.unlock();

    return true;
}

void DriveBase::DrivePath(std::vector<Node::NodeReedSheps>& path, int max_drive_time, float x, float y , float a) {
    

    this->writing_mutex.lock();
    this->new_command = true;
    this->x = x;
    this->path_x = x;
    this->y = y;
    this->path_y = y;
    this->a = a;
    this->path_a = a;
    this->current_path = path;
    this->max_drive_time = max_drive_time;

    this->writing_mutex.unlock();
}

Node::NodeReedSheps DriveBase::GetCurrentNode() {
    writing_mutex.lock();
    Node::NodeReedSheps current_node = current_step;
    current_node.distance -= current_distance;
    writing_mutex.unlock();
    return current_node;
}

void DriveBase::CalculateClosestPoint(std::vector<Node::NodeReedSheps> &path, float& d_min, float& a_min, float& point_x, float &point_y, float k, float &till_end, float delta_x, float delta_y, float delta_a) {
    
    float min_distance = 10000000;
    
    int j = 0;
    float xp = this->path_x;
    float yp = this->path_y;
    float ap = this->path_a;
    float look_ahead = 100000;
    for (Node::NodeReedSheps& n : path) {
        float t = 0;
        while (true) {
            float min_d = std::min(n.distance - t, 0.02f);

            if (n.steering != 0) {
                xp += (sin(a) - sin(a - min_d/this->min_radius * n.steering * n.drive)) * this->min_radius * n.steering;
                yp -= (cos(a) - cos(a - min_d/this->min_radius * n.steering * n.drive)) * this->min_radius * n.steering;
                ap -= min_d/this->min_radius * n.steering * n.drive;
            }
            else {
                xp += cos(a) * min_d * n.drive;
                yp += sin(a) * min_d * n.drive;
            }
            t += min_d;
            float distance = pow(xp - this->x - delta_x - delta_y, 2) + pow(yp - this->y, 2);
            if (distance < min_distance) {
                min_distance = distance;
                a_min = ap;
                point_x = xp;
                point_y = yp;
                d_min = sqrt(distance);
                look_ahead = 0;
            }

            if (look_ahead < k) {
                a_min = ap;
                point_x = xp;
                point_y = yp;
                till_end = 0;
            }
            till_end += min_d;
            look_ahead += min_d;

            if (abs(t - n.distance) < 0.00001) {
                break;
            }
        }
        j++;
    }

}

void DriveBase::PathLoop() {

    float wheel = 0.01672;

    while (!has_reading) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    auto steer_timeout = std::chrono::high_resolution_clock::now();
    double last_encoder = 0;    
    double last_encoder2 = EncoderAngle();
    auto time_start = std::chrono::high_resolution_clock::now();
    double running_averag_vel = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        writing_mutex.lock();
        double step = EncoderAngle() - last_encoder2;
        last_encoder2 = step + last_encoder2;
        double delta = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - time_start).count() / 1000000.0;
        time_start = std::chrono::high_resolution_clock::now();
        running_averag_vel = step * wheel / delta * 0.1 + running_averag_vel * 0.9;


        if (abs(step) > 100) {
            step = 0;
        }
        
        /*
        if (abs(last_steer) < 0.1) {
            dx += cos(da) * step * wheel;
            x += cos(da) * step * wheel;
            dy += sin(da) * step * wheel;
            y += sin(da) * step * wheel;
        }
        else {
            dx += (sin(da) - sin(da - step * wheel / LookUp::LookUpSteer(last_steer))) * LookUp::LookUpSteer(last_steer);
            x += (sin(da) - sin(da - step * wheel / LookUp::LookUpSteer(last_steer))) * LookUp::LookUpSteer(last_steer);
            dy -= (cos(da) - cos(da - step * wheel / LookUp::LookUpSteer(last_steer))) * LookUp::LookUpSteer(last_steer);
            y -= (cos(da) - cos(da - step * wheel / LookUp::LookUpSteer(last_steer))) * LookUp::LookUpSteer(last_steer);
            da -= step * wheel / LookUp::LookUpSteer(last_steer);
            a -= step * wheel / LookUp::LookUpSteer(last_steer); 
        }
            */
        float heading_imu = imu->GetHeading() - this->heading_offset + PI/2;
        
        dx += cos(da) * step * wheel;
        dy += sin(da) * step * wheel;
        da += heading_imu - a; 
        
        x += cos(a) * step * wheel;
        y += sin(a) * step * wheel;
        a = heading_imu;

        
        if(current_path.size() == 0) {
            writing_mutex.unlock();
            SendCommand(0, this->last_steer);
            continue;
        }
        if (this->new_command) {
            this->last_command = std::chrono::high_resolution_clock::now();
            last_encoder = EncoderAngle();
            this->new_command = false;
        }
        

        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - this->last_command).count() > this->max_drive_time) {
            writing_mutex.unlock();
            SendCommand(0, this->last_steer);
            continue;
        }

        float point_x;
        float point_y;
        float point_angle;
        float path_d;
        float till_end;
        
        float radius = LookUp::LookUpSteer(GetCurrentSteer());
        float look_ahead_time = 0.146;
        float delta_y = -(cos(a) - cos(a - look_ahead_time * running_averag_vel / radius)) * radius;
        float delta_x = (sin(a) - sin(a - look_ahead_time * running_averag_vel / radius)) * radius;
        float delta_a = -look_ahead_time * running_averag_vel / radius;
        if (abs(radius) > 10) {
            delta_y = sin(a) * look_ahead_time * running_averag_vel;
            delta_x = cos(a) * look_ahead_time * running_averag_vel;
            delta_a = 0;
        }

        CalculateClosestPoint(this->current_path, path_d, point_angle, point_x, point_y, 0.15, till_end, delta_x, delta_y, delta_a);

        float direction = (point_x - x - delta_x) * cos(point_angle) + (point_y - y - delta_y) * sin(point_angle);
        float direction_side = (point_x - x - delta_x) * sin(point_angle) - (point_y - y - delta_y) * cos(point_angle);
        
        float speed = 0.30;
        int drive = 1;
        if (direction < 0) {
            drive = -1;
        }
        if (till_end < 0.1) {
            speed = 0.2;
        }

        float heading = (a + delta_a - point_angle) + atan(0.5 * path_d / (abs(running_averag_vel) + 1.0)) * (direction_side > 0 ? 1 : -1);
        heading *= drive;
        float steer = 0.32861 / 0.065 * tan(heading) + 0.0138;
        //std::cout << "(" << a << " - " << point_angle << ") + atan(1.0 * " << path_d << " / " << (abs(running_averag_vel) + 0.2) <<   ")\n";
        
        writing_mutex.unlock();

        
        //std::cout << "ts: " << drive << " cs:" << running_averag_vel << " r:" << LookUp::LookUpSteer(this->last_steer) << " steer:" << steer  << " heading:" << heading << "\n";
        
        SendCommand(speed * drive, steer);
        
    }
}

void DriveBase::CommandLoop() {

    auto last_time = std::chrono::high_resolution_clock::now();
    while(true) {

        auto time = std::chrono::high_resolution_clock::now();
        float time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(time - last_time).count();
        time_delta = std::min(time_delta, 50.f);
        last_time = time;
        
        double max_power_delta = this->accel / LookUp::LookUpSpeed(0.4) * 0.4 * time_delta / 1000.f;
        comms_mutex.lock();
        double max_steer_delta = 6.8266 * time_delta / 1000.f;
        current_steer = std::clamp((double)this->last_steer, this->current_steer - max_steer_delta, this->current_steer + max_steer_delta);
        float power = std::clamp((double)this->drive, this->last_speed - max_power_delta, this->last_speed + max_power_delta);
        power = std::clamp(power, -1.f, 1.f);
        
        this->last_speed = power;
        this->last_steer = this->turn;

        uint8_t udrive = (uint8_t)(127 + power * 127);
        uint8_t uturn = (uint8_t)(127 + turn * 127);
        uint8_t data[3] = {udrive, uturn, this->led};
        comms_mutex.unlock();

        Write(0x80, data, 3);
        DriveBase::Response ret = Read(10);
        if (ret.header == 0x35) {
            if (ret.ack == 0x01) {
                comms_mutex.lock();
                this->encoder = ret.data;
                comms_mutex.unlock();
                has_reading = true;
                //std::cout << "Angle = " << std::dec << ret.data << "\n";
            }
            else {
                std::cout << "RP2040 error: " << std::hex << (int)ret.ack << std::dec << "\n";
            }
        }
        else {
            std::cout << "RP2040 Error: TIMEOUT " << (int)ret.ack << "\n";
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

float DriveBase::LastSpeed() {
    comms_mutex.lock();
    float ls = this->last_speed;
    comms_mutex.unlock();
    return ls;
}

void DriveBase::BindImu(Bno055* imu) {
    this->imu = imu;
}

double DriveBase::EncoderAngle() { 
    comms_mutex.lock();
    int val = this->encoder;
    comms_mutex.unlock();
    
    return 2 * PI * val / 4096. / 19.3861;
}

Point DriveBase::GetDelta(float angle) {
    writing_mutex.lock();
    da = 0;
    Point p(dx, dy);
    p.Transform(0,0,angle);
    dx = 0;
    dy = 0;
    writing_mutex.unlock();
    return p;
}

void DriveBase::UpdatePosition(float x, float y, float a) {
    this->writing_mutex.lock();
    this->x = x;
    this->y = y;
    this->a = a;
    this->writing_mutex.unlock();
}

void DriveBase::SetHeadingOffset(float offset) {
    this->writing_mutex.lock();
    this->heading_offset = offset;
    this->writing_mutex.unlock();
}

float DriveBase::GetCurrentSteer() {
    this->comms_mutex.lock();
    float cs = this->current_steer;
    this->comms_mutex.unlock();
    return cs;
}

bool DriveBase::IsButtonPressed() {
    this->comms_mutex.lock();
    bool res = this->button > 127;
    this->comms_mutex.unlock();
    return res;
}

void DriveBase::SetLedPWM(uint8_t pwm) {
    this->comms_mutex.lock();
    this->led = pwm;
    this->comms_mutex.unlock();
}