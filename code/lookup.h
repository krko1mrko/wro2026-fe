#pragma once
#include <vector>
#include "stl27_driver.h"
#include "drive.h"
#include "bno055_driver.h"

class LookUp {  
    public:
    std::vector<std::vector<float>> lookup_steer;
    std::vector<std::vector<float>> lookup_drive;

    void GenerateLookup(Stl27_lidar &lidar, DriveBase &car, Bno055& imu, int steering_dev, int driving_div);

    static float LookUpSpeed(float control);
    static float LookUpSteer(float control);
    static float CorrectSteer(float control);
    static float InvLookUpSteer(float r);
};

