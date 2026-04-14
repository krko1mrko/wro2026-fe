#include "vison.h"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "Point.h"
#include "consts.h"

Vision::Vision() {
    if (!cap.open(0)){
        std::cerr << "Cant open camera\n";
    }
}

std::vector<Signal> Vision::DisplayFrame() {
    if (!cap.isOpened()) {
        return {};
    }
    cv::Mat frame;
    
    float fov = 65;
    float offset = 0.012;
    float start_angle = PI / 2 - offset - fov * PI / 180 /2;
    float end_angle = PI / 2 - offset + fov * PI / 180 /2;
    cv::Mat frame_d;    
    cap >> frame_d;
        if (frame_d.empty()) {
            std::cerr << "frame empty\n";
            return {};
        }
        double camera_matrix[9] = {1.7417723109984879e+03, 0., 8.6906485233487876e+02, 0., 1.7222103398508298e+03, 4.9915309682977272e+02, 0., 0., 1.};
        double distortion_matrix[5] = {2.3919496976422778e-02, -6.0700842659701833e-02, -6.8538920069582824e-03, -8.2933476584160157e-03, 0.};
        cv::undistort(frame_d, frame, cv::Mat(3,3, CV_64F, camera_matrix), cv::Mat(1, 5 ,CV_64F, distortion_matrix));
        std::vector<LidarPoint> depth;
        if (lidar != nullptr && imu != nullptr) {
            lidar->GetBilateralFiltered(&depth, *(this->imu), false);
        }
        int w = frame.size().width;
        frame = frame(cv::Rect(0, 196, w, 30));
        int h = frame.size().height;
        if (depth.size() > 0) {
            std::sort(depth.begin(), depth.end(),[](const LidarPoint& a, const LidarPoint& b) -> bool {return a.angle < b.angle; });
           //frame(cv::Rect(0,0,w,10)) = cv::Scalar(0, 0, 0);
        }
        float smallest_d = 1000000;

        std::vector<float> depth_map(w);

        int j = 0;
        for (int i = 0; i < w; i++) {
            float angle = end_angle * (float)i/(float)w + start_angle * (1.f - (float)i/(float)w);
            while (depth[j].angle < angle) {
                j++;
            } 
            //float k = (angle - depth[j - 1].x) / (depth[j].x - depth[j-1].x);
            //depth_map[i] = depth[j - 1].y * (1 - k) + depth[j].y * k;   
            depth_map[i] = depth[j].distance; 
        }
        float max_d = *(std::max_element(depth_map.begin(), depth_map.end()));
        float min_d = *(std::min_element(depth_map.begin(), depth_map.end()));
        for (int i = 0; i < w; i++) {
            //img(cv::Rect(i, 0, 1, h)) = 255 - std::log(depth_map[i] - min_d + 1) * 255 / std::log(max_d - min_d + 1);
        }


        
        cv::Mat hsv_frame;
        cv::boxFilter(frame, frame, -1, cv::Size(5,5));
        cv::cvtColor(frame, hsv_frame, cv::COLOR_RGB2HSV_FULL);
        std::vector<uchar> red_bottom = {136, 87, 111};
        std::vector<uchar> red_upper = {180, 255, 255};
        cv::Mat red_mask;
        cv::inRange(hsv_frame, red_bottom, red_upper, red_mask);


        std::vector<uchar> green_bottom = {25, 52, 72};
        std::vector<uchar> green_upper = {102, 255, 255};
        cv::Mat green_mask;
        cv::inRange(hsv_frame, green_bottom, green_upper, green_mask);
        
        std::vector<std::vector<cv::Point>> contours_red;
        std::vector<cv::Vec4i> hierarchy_red;
        cv::findContours(red_mask, contours_red, hierarchy_red, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        for(int i = 0; i < contours_red.size(); i++) {
            if (cv::contourArea(contours_red[i]) < 50) {
                contours_red.erase(contours_red.begin() + i);
                i--;
            }
        }

        cv::Mat filtered_red_mask = cv::Mat(h, w, CV_8U, cv::Scalar(0,0,0,0));
        cv::drawContours(filtered_red_mask, contours_red, -1, cv::Scalar(255), cv::FILLED);
        
        std::vector<Signal> signals;
        
        double avgd = 0;
        double count = 0;
        double avga = 0;
        for (int i = 0; i < w - 3; i++) {
            double next_window = 0;
            double next_count = 0;

            for (int j = i + 1; j < i + 3; j++) {
                cv::Scalar count_slit_next = cv::sum(filtered_red_mask(cv::Rect(j, 0, 1, h)));
                next_window += depth_map[j] * count_slit_next[0] / 255;
                next_count += count_slit_next[0] / 255;    
            }
            
            cv::Scalar count_slit = cv::sum(filtered_red_mask(cv::Rect(i, 0, 1, h)));
            avgd += depth_map[i] * count_slit[0] / 255;
            avga += i * count_slit[0] / 255;
            count += count_slit[0] / 255;
            if (count > 5) {
                if (next_count > 5) {
                    if (abs(avgd / count - next_window / next_count) > 0.2) {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::RED, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                    }
                    else if (i >= w - 4) {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::RED, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                        
                    }
                }
                else {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::RED, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                }
                
            }
            
        }

        std::vector<std::vector<cv::Point>> contours_green;
        std::vector<cv::Vec4i> hierarchy_green;
        cv::findContours(green_mask, contours_green, hierarchy_green, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        for(int i = 0; i < contours_green.size(); i++) {
            if (cv::contourArea(contours_green[i]) < 50) {
                contours_green.erase(contours_green.begin() + i);
                i--;
            }
        }

        cv::Mat filtered_green_mask = cv::Mat(h, w, CV_8U, cv::Scalar(0,0,0,0));
        cv::drawContours(filtered_green_mask, contours_green, -1, cv::Scalar(255), cv::FILLED);
        
        for (int i = 0; i < w - 3; i++) {
            double next_window = 0;
            double next_count = 0;

            for (int j = i + 1; j < i + 3; j++) {
                cv::Scalar count_slit_next = cv::sum(filtered_green_mask(cv::Rect(j, 0, 1, h)));
                next_window += depth_map[j] * count_slit_next[0] / 255;
                next_count += count_slit_next[0] / 255;    
            }
            
            cv::Scalar count_slit = cv::sum(filtered_green_mask(cv::Rect(i, 0, 1, h)));
            avgd += depth_map[i] * count_slit[0] / 255;
            avga += i * count_slit[0] / 255;
            count += count_slit[0] / 255;
            if (count > 5) {
                if (next_count > 5) {
                    if (abs(avgd / count - next_window / next_count) > 0.2) {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::GREEN, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                    }
                    else if (i >= w - 4) {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::GREEN, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                        
                    }
                }
                else {
                        avga /= count;
                        avga = end_angle * (float)avga/(float)w + start_angle * (1.f - (float)avga/(float)w);
                        avgd /= count;
                        avgd += 0.025;
                        
                        signals.push_back(Signal(cos(avga) * avgd, sin(avga) * avgd, Signal::Color::GREEN, Signal::Direction::CW, count));
                        avga = 0;
                        avgd = 0;
                        count = 0;
                }
                
            }
            
        }
    return signals;

}

void Vision::AddLidar(Stl27_lidar* lidar) {
    this->lidar = lidar;
}

void Vision::AddImu(Bno055* imu) {
    this->imu = imu;
}
