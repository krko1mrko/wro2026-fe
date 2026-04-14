#include "signal.h"
#include <math.h>
#include <iostream>
#include "consts.h"
#include "utils.h"
#include "Config.h"

Signal::Signal(float x, float y, Color color, Direction direction, float size) {
    this->x = x;
    this->y = y;
    this->color = color;
    this->direction = direction;
    this->pixel_size = size;
}

float Signal::DistanceTo(Signal s) {
    return sqrt(pow(this->x - s.x, 2) + pow(this->y - s.y, 2));
}

void Signal::Transform(float dx, float dy, float da) {
    float x_temp = this->x;
	float y_temp = this->y;

	this->x = x_temp * cos(da) - y_temp * sin(da) + dx;
	this->y = x_temp * sin(da) + y_temp * cos(da) + dy;
}

std::vector<Signal> Signal::total_signals;
std::vector<Signal> Signal::old_signals;

std::vector<Signal> valid_signals = {
    Signal(-0.5,1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-0.5,0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.0,1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.0,0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.5,1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.5,0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    
    Signal(-0.5,-1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-0.5,-0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.0,-1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.0,-0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.5,-1.1,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.5,-0.9,Signal::Color::RED, Signal::Direction::CW, 0),
    
    Signal(-1.1,-0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-0.9,-0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-1.1, 0.0,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-0.9, 0.0,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-1.1, 0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(-0.9, 0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    
    Signal(1.1,-0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.9,-0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(1.1, 0.0,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.9, 0.0,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(1.1, 0.5,Signal::Color::RED, Signal::Direction::CW, 0),
    Signal(0.9, 0.5,Signal::Color::RED, Signal::Direction::CW, 0)
};

void Signal::MergeSignals(std::vector<Signal>& signals, Direction dir) {

    for (int i = 0; i < signals.size(); i++) {
        if (signals[i].pixel_size < 150) {
        //    signals.erase(signals.begin() + i);
        //    i--;
        //    continue;
        }
        bool close = false;
        for (Signal& s : valid_signals) {
            if (signals[i].DistanceTo(s) < 0.1) {
                close = true;
                break;
            }
        }
        if (!close) {
            signals.erase(signals.begin() + i);
            i--;
        }
    }

    Signal::old_signals.insert(Signal::old_signals.end(), signals.begin(), signals.end());

    for (int i = 0; i < valid_signals.size(); i++) {
        int count_red = 0;
        int count_green = 0;
        
        for (int j = 0; j < old_signals.size(); j++) {
            if (valid_signals[i].DistanceTo(old_signals[j]) < 0.10) {
                if (old_signals[j].color == Color::RED) {
                    count_red++;
                }
                else if (old_signals[j].color == Color::GREEN) {
                    count_green++;
                }
            }
        }
        if (abs(count_red - count_green) > 6) {
            Signal new_signal = Signal(valid_signals[i].x, valid_signals[i].y, Color::RED, dir, 0);
            if (count_red > count_green) {
                new_signal.color = Color::RED;
            }
            else {
                new_signal.color = Color::GREEN;
            }
            total_signals.push_back(new_signal);
            if (i % 2 == 0) {
                valid_signals.erase(valid_signals.begin() + i);
                valid_signals.erase(valid_signals.begin() + i);
                i--;
                
            }
            else {
                valid_signals.erase(valid_signals.begin() + i);
                i--;
                valid_signals.erase(valid_signals.begin() + i);
                
            }
        } 
    }
}

void AddSignalLine(std::vector<Point>* in, float x0, float y0, float x1, float y1, float density) {
        float len = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
        len *= density;

        for (int i = 0; i < len; i++) {
            float k = (float)i/len;
            in->push_back(Point(x0 * k + x1 * ( 1 - k), y0 * k + y1 * ( 1 - k)));
        }
    }

void Signal::AddVirtualWall(std::vector<Point>* map) {
    bool v = abs(this->y) > 0.6;
    if ((int)this->color == (int)this->direction) {
        AddSignalLine(map, this->x, this->y, !v ? 0.5 * (this->x > 0 ? 1 : -1) : this->x, v ? 0.5 * (this->y > 0 ? 1 : -1) : this->y, 50);
    }
    else {
        AddSignalLine(map, this->x, this->y, !v ? 1.5 * (this->x > 0 ? 1 : -1) : this->x, v ? 1.5 * (this->y > 0 ? 1 : -1) : this->y, 50);    
    }
}

void Signal::AddWaypoint() {
    if (!this->added_waypoint) {
        this->added_waypoint = true;
        bool v = abs(this->y) > 0.6;    
        if (v) {
            Waypoint(x, y + y / abs(y) * ((int)color != (int)direction ? -0.2 : 0.2), y > 0 ? ((int)direction * PI) : (PI + (int)direction * PI));
        }
        else {
            Waypoint(x + x / abs(x) * ((int)color != (int)direction ? -0.2 : 0.2), y,  x < 0 ? ( PI / 2 + (int)direction * PI) : (3 * PI / 2 + (int)direction * PI));
        }
        if (abs(abs(x) - 0.5) < 0.1 || abs(abs(y) - 0.5) < 0.1) {
            Waypoint(x < -0.6 ? -1 : (x > 0.6 ? 1 : 0), y < -0.6 ? -1 : (y > 0.6 ? 1 : 0), v ? (y > 0 ? ((int)direction * PI) : (PI + (int)direction * PI)) : (x < 0 ? ( PI / 2 + (int)direction * PI) : (3 * PI / 2 + (int)direction * PI)));
        }
    }
}

std::vector<Waypoint> Waypoint::waypoints;
Waypoint Waypoint::previous(0,0,0, true);
Waypoint Waypoint::final_waypoint(0, 0, 0, true);

bool Waypoint::reached_final = false;

Waypoint::Waypoint(float x, float y, float a, int wait, bool temp) {
    this->x = x;
    this->y = y;
    this->a = a;
    this->wait = wait;
    if (temp) {
        return;
    }

    bool exists = false;
    for (Waypoint& w: waypoints) {
        if (abs (w.x - x) < 0.01 &&
            abs (w.y - y) < 0.01 &&
            abs (w.a - a) < 0.01) {
                exists = true;
                break;
        }
    }
    if (!exists) {
        waypoints.push_back(*this);
    }
}

Waypoint Waypoint::GetNextWeypoint(float x, float y, bool direction) {
    
    float robot_angle = atan2(y, x);

    float min_value = 1000000000;
    int j = -1;
    Waypoint res = previous;
    for (int i = 0; i < waypoints.size(); i++) {
        float waypoint_angle = atan2(waypoints[i].y, waypoints[i].x);
        if ((sin(robot_angle - waypoint_angle) < 0) == direction) {
            if (pow(Waypoint::previous.x - waypoints[i].x, 2) + pow(Waypoint::previous.y - waypoints[i].y, 2) > 0.05) {
                float vel =abs(AngleMod(robot_angle - waypoint_angle));
                if (vel < min_value && waypoints[i].n != NUMBER_OF_TURNS) {
                    min_value = vel;
                    res = waypoints[i];
                    j = i;
                }
            }
        }
    }

    if (j == -1) {
        Waypoint::reached_final = true;
        return Waypoint::final_waypoint;

    }
    Waypoint::previous = res;
    waypoints[j].n++;
    return waypoints[j];
}

