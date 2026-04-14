#include "lookup.h"
#include "icp.h"
#include <chrono>
#include "utils.h"

namespace LookUpData {
    float drive_lut[21][2] = {
{1, 0.728},
{0.9, 0.69},
{0.8, 0.683},
{0.7, 0.686},
{0.6, 0.594},
{0.5, 0.505},
{0.4, 0.455},
{0.3, 0.293},
{0.2, 0.102},
{0.1, 0.000488},
{0, 3.05e-05},
{-0.1, 0.000243},
{-0.2, -0.122},
{-0.3, -0.286},
{-0.4, -0.482},
{-0.5, -0.549},
{-0.6, -0.586},
{-0.7, -0.573},
{-0.8, -0.561},
{-0.9, -0.578},
{-1, -0.559},
};
    float steer_a = 0.32861;
    float steer_b = -0.0138;
}

void LookUp::GenerateLookup(Stl27_lidar &lidar, DriveBase& car, Bno055& imu, int steering_dev, int driving_div) {
    float speed = 1;
    for (int i = 0; i < steering_dev + 1; i++) {
        
        float avgr = 0;
        float steer = 1.f - 2.f / steering_dev * i;
        for (int k = 0; k < 2; k++) {
        std::cin.get();
        car.SendCommand(0, steer);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        double start_encoder = car.EncoderAngle();
        
        car.SendCommand(0.5, steer);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto start = std::chrono::high_resolution_clock::now();
        float imu_offset = imu.GetHeading();
        
        while(true) {

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            if (duration.count() > 1000) {
                float r = (car.EncoderAngle() - start_encoder) * 0.0177 / AngleMod(imu.GetHeading() - imu_offset);
                car.SendCommand(0, steer);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                car.SendCommand(-0.5, steer);
                std::this_thread::sleep_for(std::chrono::milliseconds(duration.count() + 500));
                car.SendCommand(0, steer);
                
                //r = r / abs(r) * sqrt(r * r - 0.065 * 0.065);
                avgr += r;
                break;
            }
        }
        }
        avgr = avgr / 2;
        std::vector<float> reading = {steer, avgr};
        lookup_steer.push_back(reading);
        std::cout << std::setprecision(3) << "Steering: " << steer << " R:" << avgr  << " \n";
                
    }
    for (int i = 0; i < driving_div + 1; i++) {
        float speed = 1.f - 2.f / driving_div * i;
        float avgv = 0;
        float steera[5] = { 0.5, 0.8, 0.4, -0.5, -0.8};
        for (int i = 0; i < 1; i++) {
        
        std::cin.get();
        car.SendCommand(speed,steera[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        auto start = std::chrono::high_resolution_clock::now();
        int j = 1;
        double es = car.EncoderAngle();
        float v;
        while(true) {

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (duration.count() > 800) {
                double ee = car.EncoderAngle();
                v = (ee - es) * 0.0175 / duration.count() * 1000;

                car.SendCommand(0,steera[i]);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                car.SendCommand(-speed,steera[i]);
                std::this_thread::sleep_for(std::chrono::milliseconds(duration.count() + 600));
                
                car.SendCommand(0,steera[i]);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                
                break;
            }
        }
        avgv += v;
        }
        std::vector<float> reading = {speed, avgv / 1};
        lookup_drive.push_back(reading);
        std::cout << std::setprecision(3) << "Speed: " << speed << " R:" << avgv / 1  << " \n";
        
    }
}

float LookUp::LookUpSteer(float control) {
    return LookUpData::steer_a / (control + LookUpData::steer_b);
}

float LookUp::InvLookUpSteer(float r) {
    if (abs(r) < 0.02) {
        return -LookUpData::steer_b;
    }
    return LookUpData::steer_a / r - LookUpData::steer_b;
}

float LookUp::LookUpSpeed(float control) {

    control /= 2;

    return 1.4429 * control;
}

float LookUp::CorrectSteer(float control) {
    return control - LookUpData::steer_b;
}
