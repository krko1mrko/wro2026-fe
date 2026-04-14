#pragma once

#include "Point.h"
#include <vector>

class Waypoint{
    public:
    float x;
    float y;
    float a;
    int n = 0;
    
    int wait = 0;

    Waypoint(float x, float y, float a, int wait = 0, bool temp = false);
    static std::vector<Waypoint> waypoints;
    static bool reached_final;
    static Waypoint final_waypoint;
    static Waypoint previous;
    static Waypoint GetNextWeypoint(float x, float y, bool direction);
};

class Signal {
    public:
    enum Color {
        RED = 0,
        GREEN = 1
    };

    enum Direction {
        CW = 1,
        CCW = 0
    };

    Signal(float x, float y, Color color, Direction direction, float size);
    void AddVirtualWall(std::vector<Point>* map);
    float DistanceTo (Signal s);
    void Transform(float dx, float dy, float da);
    void AddWaypoint();
    bool added_waypoint = false;
    float pixel_size;
    Color color;
    float x;
    float y;
    Direction direction;

    static std::vector<Signal> total_signals;
    static std::vector<Signal> old_signals;
    
    static void MergeSignals(std::vector<Signal>& signals, Direction dir);
};

