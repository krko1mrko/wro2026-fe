<br><br><h1 align=center> Team Ouroboros – WRO 2026 Future Engineers<br><br>

## Robot Overview

This repository contains the complete software system developed by Team Ouroboros for the WRO 2026 Future Engineers competition. The robot is a fully autonomous vehicle designed to complete both the Open Challenge and the Obstacle Challenge using a combination of advanced mechanical design and optimized software architecture.

The platform is based on a four-wheel drive and four-wheel steering configuration, allowing for improved maneuverability and traction. The software is designed to operate in real time, using multiple sensors and optimized algorithms for localization, perception, and decision-making.

---

## System Architecture

The system is divided into multiple modules, each responsible for a specific function. These modules interact to form a complete autonomous control pipeline.

### High-Level Data Flow

Sensor Data → Sensor Processing → Localization → Path Planning → Control → Actuation

---

## Software Modules

### 1. Sensor Interface Module

This module is responsible for acquiring and preprocessing data from all sensors used in the system.

Sensors include:
- LiDAR (STL-27L 360° DTOF) – environment mapping and obstacle detection  
- IMU (BNO055) – orientation and heading estimation  
- Camera (LattePanda 5MP UVC) – signal detection  
- Encoder (AS5600) – motor rotation and distance estimation  

The module ensures synchronized and structured data delivery to higher-level components.

---

### 2. Localization Module

The localization system determines the position and orientation of the robot within the map.

The main algorithm used is ICP (Iterative Closest Point), which aligns LiDAR scans with a pre-generated reference map. The IMU provides an initial estimate of the transformation to improve convergence speed.

To ensure real-time performance:
- A quad tree data structure is used for efficient spatial indexing  
- Approximate nearest neighbor search is implemented  
- OpenCL is used for GPU acceleration  

The output of this module is the robot’s pose (x, y, heading).

---

### 3. Path Planning Module

This module determines the optimal trajectory for the robot.

Two planning strategies are implemented:
- Open Challenge pathfinding  
- Obstacle avoidance pathfinding  

The planner uses the robot’s current position and environmental data to generate a path that balances efficiency and safety.

---

### 4. Obstacle Detection and Avoidance Module

This module processes LiDAR data to detect dynamic obstacles.

Steps include:
- Detection of obstacles from point cloud data  
- Classification of detected objects  
- Modification of trajectory to avoid collisions  

This module ensures safe navigation during the obstacle challenge.

---

### 5. Control Module

The control module converts planned trajectories into actuator commands.

Responsibilities include:
- Motor speed control  
- Steering angle control  
- Real-time corrections based on feedback  

This module communicates directly with the microcontroller for execution.

---

## Hardware and Software Integration

The software is tightly coupled with the robot’s electromechanical system.

### Main Components

- Main processor: Intel N150 (runs high-level algorithms)  
- Microcontroller: STM32 (handles real-time control)  
- Drive motor: BLDC motor with gearbox  
- Steering actuator: servo motor  

### Interaction

- The main processor processes sensor data and computes control commands  
- Commands are sent to the STM32 microcontroller  
- The STM32 controls motors and actuators  
- Feedback from sensors is continuously processed  

This separation ensures stable real-time performance.

---

## Build Instructions

### Dependencies

- Ubuntu operating system: ([LinkText]https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- C++ compiler (g++) ([LinkText]https://gist.github.com/hugoledoux/2e91ed3efbfa8ca5da1ea27e522d2b34)
- OpenCL ICD:
    1. OpenCL: [LinkText]https://github.com/KhronosGroup/OpenCL-Guide/blob/main/chapters/getting_started_linux.md
    2. Driver: [LinkText]https://support.zivid.com/en/latest/camera/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
- OpenCV Library: ([LinkText]https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html)
- Arduino IDE  ([LinkText]https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)
- STM32duino ([LinkText]https://github.com/stm32duino#welcome-to-the-stm32duino-open-source-community)
- SCServo Library ([LinkText]https://github.com/adityakamath/SCServo_Linux?tab=readme-ov-file#scservo_linux)
- Modified SimonK firmware for the ESC (Depending on your ESC choice, the firmware may vary) ([LinkText]https://github.com/sim-/tgy)

### Steps

1. Clone the repository and enter the directory:

```bash
- git clone https://github.com/krko1mrko/wro2026-fe  
- cd wro2026-fe/code
```

2. Build the project:

```bash
- g++ *.cpp -fpermissive 'pkg-config --cflags --libs opencv4' -lOpenCL -O3 -o main
```

3. Ensure all dependencies are properly installed and linked.

---

## Uploading and Running the Code

### Main Processor

The compiled program is executed directly on the onboard computer:

```bash
./main
```

### Microcontroller (STM32)

1. Connect the STM32 board via USB  
2. Open Arduino IDE or compatible environment  
3. Select the correct board and port  
4. Upload the firmware  

---

## Execution Loop

The system operates in a continuous loop:

1. Sensors acquire environmental data  
2. Data is processed and filtered  
3. Localization estimates robot pose  
4. Path planning determines next movement  
5. Control module generates commands  
6. STM32 executes motor and steering actions  

---

## Configuration

System parameters such as speed limits, control constants, and sensor calibration values
are defined within the source code and can be adjusted prior to compilation.

## Additional Notes

- The system minimizes external dependencies to improve portability  
- Performance is optimized through algorithm design and GPU acceleration  
- The modular structure allows easy debugging and future expansion  

---

## Repository Structure

/t-photos
/v-photos
/video
/CAD Files.zip
/wro2026-fe-ouroboros-docs
/README.md

---

## Conclusion

The presented system combines mechanical design and optimized software architecture to achieve reliable autonomous navigation. The modular design, efficient algorithms, and tight integration with hardware allow the robot to operate effectively within the constraints of the WRO 2026 Future Engineers competition.