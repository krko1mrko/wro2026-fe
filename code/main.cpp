#include "Config.h"
#include <iostream>
#include "stl27_driver.h"
#include "bno055_driver.h"
#include "icp.h"
#include "vison.h"
#include "drive.h"
#include "thread"
#include "pid.h"
#include "lookup.h"
#include "pathfinding.h"
#include "consts.h"
#include "signal.h"
#include "utils.h"


cv::Mat img = cv::Mat(512, 512, CV_8UC3, cv::Scalar(0,0,0,0));
void Display(std::vector<Point> reference){
    img -= cv::Scalar(150, 50, 1);
    int zoom = 64;
    for (Point& p: reference) {
        
        img.at<uchar>(std::clamp((int)(p.y * zoom + 256), 0, 511), std::clamp((int)(p.x * zoom + 256), 0, 511), 0) = 255;
        //cv::circle(img, cv::Point((int)(p.x * 256 + 256), (int)(p.y * 256 + 256)), 1, cv::Scalar(255,255,255));
        //img(cv::Rect(std::clamp((int)(p.x * zoom + 256), 0, 511), std::clamp((int)(p.y * zoom + 256), 0, 511), 1, 1)) = cv::Scalar(255, 0, 0);
        //img.at<uchar>(std::clamp((int)(p.x * zoom + 256), 0, 511), std::clamp((int)(p.y * zoom + 256), 0, 511)) = 255;
    }
    cv::imshow("cmb",img);
    return;
}

    std::vector<Point> reference;
    std::vector<std::vector<Point>> old_scans;
    double position_start[3] = {0, 0, 0};
    float target[3] = {};


    float last_click[2];
    float click_index = 0;
    bool has_target = false;
    void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
    {
        if (event == cv::EVENT_LBUTTONDOWN ) {
            float x_cord = ((float) x - 256) / 64;
            float y_cord = ((float) y - 256) / 64;
            
            //HybridAStar pathfinder = HybridAStar(0.3, 0.2, Node(x_cord, y_cord, position_start[2] + PI / 2), Node(position_start[0], position_start[1], position_start[2] + PI / 2 ));
            //pathfinder.SetEnviorment(&reference);
            //std::vector<Node> pth;
            //int num = pathfinder.CalculatePath(&pth);
            
            if (click_index++ == 1) {
                target[0] = last_click[0];
                target[1] = last_click[1];
                target[2] = -atan2(last_click[1] - y_cord, -last_click[0] + x_cord);
                click_index = 0;
                cv::arrowedLine(img, cv::Point(last_click[0] * 64 + 256, last_click[1] * 64 + 256), cv::Point(last_click[0] * 64 + 256 + cos(target[2]) * 100, last_click[1] * 64 + 256 + sin(target[2]) * 100) , cv::Scalar(0, 0, 255));
                has_target = true;
            }
            last_click[0] = x_cord;
            last_click[1] = y_cord;

        }
    }

    void AddLine(std::vector<Point>* in, float x0, float y0, float x1, float y1, float density) {
        float len = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
        len *= density;

        for (int i = 0; i < len; i++) {
            float k = (float)i/len;
            in->push_back(Point(x0 * k + x1 * ( 1 - k), y0 * k + y1 * ( 1 - k)));
        }
    }

    


    int main() {
        std::cout << "Mode: Obsticle challenge\n";
        std::cout << "Initilizing robot sensors\n";

        Stl27_lidar lidar("/dev/ttyUSB0");
        lidar.FirstReading(1000); //Wait for first reading
        
        Bno055 imu("/dev/ttyS0");
        
        PointPair::quad_tree_opencl->CompileKernel();

        DriveBase car("/dev/ttyACM0");
        car.BindImu(&imu);


        while (true) {
            bool condition = true;
            for (int i = 0; i < 2; i++) {
                condition &= car.IsButtonPressed();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (condition) {
                break;
            }
        }

        car.SetLedPWM(5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        

        #ifdef OBSTACLE_CHALLENGE
        Vision vis;
        vis.AddLidar(&lidar);
        vis.AddImu(&imu);
        #endif

        

        cv::Mat allowed_point_mask = cv::imread("obsticle_challenge_mask.bmp", cv::ImreadModes::IMREAD_GRAYSCALE);

        #ifdef DEBUG_DISPLAY
        cv::namedWindow("cmb");
        cv::setMouseCallback("cmb", mouse_callback);
        cv::namedWindow("cmb");
        cv::moveWindow("cmb", 600, 0);
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        
        AddLine(&reference, -1.5, -1.5, 1.5, -1.5, 100);
        AddLine(&reference, 1.5, -1.5, 1.5, 1.5, 100);
        AddLine(&reference, 1.5, 1.5, -1.5, 1.5, 100);
        AddLine(&reference, -1.5, 1.5, -1.5, -1.5, 100);
        #ifdef OBSTACLE_CHALLENGE
        AddLine(&reference, -0.5, -1.5, -0.5, -1.3, 100);
        AddLine(&reference, -0.25, -1.5, -0.25, -1.3, 100);
        #endif

        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        std::vector<Point> compare_start;
        lidar.GetBilateralFiltered(&compare_start, imu, true);
        #ifdef OBSTACLE_CHALLENGE
        std::vector<std::vector<float>> possible_positions = {
            {-0.25, -1.0, PI/2},
            {-0.25, -1.0, -PI/2},
        };
        #endif
        #ifdef OPEN_CHALLAGE 
        std::vector<std::vector<float>> possible_positions = {
            {-0.25, -0.7, -PI/2},
            {-0.25, -1.0, -PI/2},
            {-0.25, -1.3, -PI/2},
            {+0.25, -0.7, -PI/2},
            {+0.25, -1.0, -PI/2},
            {+0.25, -1.3, -PI/2},
            {-0.25, -0.7, +PI/2},
            {-0.25, -1.0, +PI/2},
            {-0.25, -1.3, +PI/2},
            {+0.25, -0.7, +PI/2},
            {+0.25, -1.0, +PI/2},
            {+0.25, -1.3, +PI/2},
        };
        #endif
        
        std::vector<float> best_position = BestPosition(possible_positions, reference, compare_start);

        #ifdef DO_PARK_END
            Waypoint::final_waypoint = Waypoint(-0.3725, -1.4, best_position[2] + PI/2, true);
        #else
            Waypoint::final_waypoint = Waypoint(best_position[0], best_position[1], best_position[2] + PI/2, true);
        #endif
        
        Signal::Direction direction = Signal::Direction::CW;
        
        position_start[0] = best_position[0];
        position_start[1] = best_position[1];
        position_start[2] = best_position[2];

        if (best_position[2] > 0) {    
            direction = Signal::Direction::CCW;
        }
        
        #ifdef OBSTACLE_CHALLENGE
        Waypoint(1, -1, PI / 2 + (direction == Signal::Direction::CW ? 0 : PI/2), 500);
        Waypoint(1, 1, PI + (direction == Signal::Direction::CW ? 0 : PI/2), 500);
        Waypoint(-1, -1, (direction == Signal::Direction::CW ? 0 : PI/2), 500);
        Waypoint(-1, 1, 3 * PI / 2 + (direction == Signal::Direction::CW ? 0 : PI/2), 500);
        #endif
        #ifdef OPEN_CHALLENGE
        Waypoint(1.2, -1.2, PI / 2 + (direction == Signal::Direction::CW ? -PI/4 : 3*PI/4));
        Waypoint(1.2, 1.2, PI + (direction == Signal::Direction::CW ? -PI/4 : 3*PI/4));
        Waypoint(-1.2, -1.2, (direction == Signal::Direction::CW ? -PI/4 : 3*PI/4));
        Waypoint(-1.2, 1.2, 3 * PI / 2 + (direction == Signal::Direction::CW ? -PI/4 : 3*PI/4));
        #endif

        
        float imu_offset = imu.GetHeading() - position_start[2];
        car.SetHeadingOffset(imu_offset);
        #ifdef DEBUG_INPUT
        bool auto_skip = false;
        #else 
        bool auto_skip = true;
        #endif
        
        auto last_pathfind = std::chrono::high_resolution_clock::now();
        auto waypoint_time = std::chrono::high_resolution_clock::now();
        auto imu_recalibration_timeout = std::chrono::high_resolution_clock::now();
        int waypoint_timeout = 0; 

        while(true) {

            std::vector<Point> compare;
            lidar.GetBilateralFiltered(&compare, imu, true);
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            position_start[2] = AngleMod(position_start[2]);
            Point delta_drive = car.GetDelta(-position_start[2] - PI/2);
            position_start[0] += delta_drive.x;
            position_start[1] -= delta_drive.y;
            position_start[2] = imu.GetHeading() - imu_offset;
            for (Point &p :compare) {
                p.Transform(position_start[0], position_start[1], position_start[2]);
            }


            ICP icp;
            float dx = 0;
            float dy = 0;
            float da = 0;
            double cost;
            int iterations = icp.Compute(compare, reference, dx, dy, da, cost, false);
            
            #ifdef DEBUG_DISPLAY
            Display(compare);
            #endif

            if (!isnanf(dx) && !isnanf(dy) && !isnanf(da) && abs(dx) < 0.25 && abs(dy) < 0.25 && abs(da) < 0.32 && cost < 0.5) {
                Point total = Point(0, 0);
                total.Transform(position_start[0], position_start[1], position_start[2]);
                total.Transform(dx, dy, da);
                
                position_start[0] = total.x;
                position_start[1] = total.y;
                position_start[2] = da + position_start[2];
                icp.MergeMaps(reference, old_scans, 4, allowed_point_mask);
                old_scans.push_back(compare);
            
                if (cost < 0.025 && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - imu_recalibration_timeout).count() < 1000) {
                    position_start[2] = AngleMod(position_start[2]);
                    imu_offset = imu.GetHeading() - position_start[2];
                    car.SetHeadingOffset(imu_offset);
                }
            }

            #ifdef OBSTACLE_CHALLENGE
            std::vector<Signal> signals = vis.DisplayFrame();
            for (Signal& s : signals) {
                s.Transform(position_start[0], position_start[1], position_start[2]);
                if (abs(s.x) < 1.4 && abs(s.y) < 1.4) {
                    cv::circle(img, cv::Point(s.x * 64 + 256, s.y * 64 + 256), 0.03535 * 64, s.color == Signal::Color::RED ? cv::Scalar(0, 0, 255, 255) : cv::Scalar(0, 255, 0, 255));
                }
                
            }
            Signal::MergeSignals(signals, direction);
            #endif
            
            std::vector<Point> reference_copy = reference;
            #ifdef OBSTACLE_CHALLENGE
            for (Signal& s : Signal::total_signals) {
                if (abs(s.x) < 1.4 && abs(s.y) < 1.4) {
                    cv::circle(img, cv::Point(s.x * 64 + 256, s.y * 64 + 256), 0.03535 * 64, s.color == Signal::Color::RED ? cv::Scalar(0, 0, 255, 255) : cv::Scalar(0, 255, 0, 255));
                }
                s.AddVirtualWall(&reference_copy);
                s.AddWaypoint();
            }
            #endif

            for (Waypoint& w : Waypoint::waypoints) {
                cv::arrowedLine(img, cv::Point(w.x * 64 + 256, w.y * 64 + 256), cv::Point(w.x * 64 + 256 + cos(w.a) * 20, w.y * 64 + 256 + sin(w.a) * 20) , cv::Scalar(0, 0, 255));
            }
            
            #ifdef DEBUG_DISPLAY
            Display(reference_copy);
            #endif

            Node car_pos = Node(position_start[0] + sin(position_start[2]) * 0.065, position_start[1] - cos(position_start[2]) * 0.065, position_start[2] + PI/2);
            car.UpdatePosition(car_pos.x, car_pos.y, car_pos.a);

            if (has_target && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_pathfind).count() > 100) {
                last_pathfind = std::chrono::high_resolution_clock::now();

                HybridAStar pathfinder = HybridAStar(0.35, 0.08, Node(target[0], target[1], target[2]), car_pos);
                pathfinder.SetEnviorment(&reference_copy);
                std::vector<Node::NodeReedSheps> pth;
                Node::NodeReedSheps current_drive_node = car.GetCurrentNode();
                int num = pathfinder.CalculatePath(&pth, current_drive_node);
            
                if (num < 0) {
                    has_target = false;
                }
                else {
                car_pos.pathfinder = &pathfinder;
                std::vector<Node> vis_path = Subdevide(pth, &car_pos);

                for (int i = 0; i < vis_path.size(); i++) {
                    cv::circle(img, cv::Point((int)(vis_path[i].x * 64 + 256), (int)(vis_path[i].y * 64 + 256)), 1, cv::Scalar(0,255,0));
                }

                car.DrivePath(pth, 5000, car_pos.x, car_pos.y, car_pos.a);
                }
            }
            
            if (sqrt((target[0] - car_pos.x) * (target[0] - car_pos.x) + (target[1] - car_pos.y) * (target[1] - car_pos.y)) < 0.2 && abs(AngleMod(target[2] - car_pos.a)) < 0.2  && has_target) {
                has_target = false;
                waypoint_time = std::chrono::high_resolution_clock::now();
                std::vector<Node::NodeReedSheps> pth_temp;
                car.DrivePath(pth_temp, 10, car_pos.x, car_pos.y, car_pos.a);
            }

            img(cv::Rect(std::clamp((int)(position_start[0] * 64 + 256), 0, 511), std::clamp((int)(position_start[1] * 64 + 256), 0, 511), 1, 1)) = cv::Scalar(0, 255, 255);
            
            auto end = std::chrono::high_resolution_clock::now();
            std::cout << "loop took: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time).count() << "ms \t";
            std::cout << "dr delta: " << delta_drive.x << ", " << delta_drive.y << ", " << car.EncoderAngle() << "\t";
            std::cout << std::fixed << std::setprecision(3) << position_start[0] << ", " << position_start[1] << ", " << position_start[2]  << ", imu:" << imu.GetHeading() - imu_offset << ", " << cost * 1000 << ", " << iterations << "\n";
            
            #ifdef DEBUG_INPUT
            int id = cv::waitKey(1);

            if (id == 97) {
                std::vector<Node::NodeReedSheps> pth_temp;
                car.DrivePath(pth_temp, 10, car_pos.x, car_pos.y, car_pos.a);
                car.SendCommand(0,0);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                car.SetLedPWM(50);
                return 0;
            }
            else if (id == 83) {
                auto_skip = !auto_skip;
            }
            else if (id == 115) {
                cv::imwrite("raw_map.bmp", img);
            }
            #endif

            if (auto_skip && !has_target && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - waypoint_time).count() > waypoint_timeout) {
                std::cout << "\n NEXT WAYPOINT CALLED \n\n";

                if (Waypoint::reached_final) {
                    std::vector<Node::NodeReedSheps> pth_temp;
                    car.DrivePath(pth_temp, 10, car_pos.x, car_pos.y, car_pos.a);
                    car.SendCommand(0,0);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    car.SetLedPWM(50);
                    return 0;
                }
                
                Waypoint w = Waypoint::GetNextWeypoint(position_start[0], position_start[1], (bool)direction);
                target[0] = w.x;
                target[1] = w.y;
                target[2] = w.a;
                has_target = true;
                waypoint_timeout = w.wait;          
            }

        }
    }
