#pragma once
#include <vector>
#include "Point.h"


class DriveBase;
class HybridAStar;

class Node {
    
    public:

    float x;
    float y;
    float a;
    float cost;
    int owner = -1;
    HybridAStar* pathfinder;

    class NodeReedSheps{
        public:
        int steering = 0;
        int drive = 1;
        float distance = 0;

        NodeReedSheps() {
        }

        void ReverseSteering() {
            steering *= -1;
        }
        void ReverseDriveing() {
            drive *= -1;
        }
        NodeReedSheps(float distance, int steer, int drive) {
            if (distance > 0) {
                this->distance = distance;
                this->drive = drive;
            }
            else {
                this->distance = -distance;
                this->drive = -drive;
            }
            this->steering = steer;
        }

    };

    NodeReedSheps reedShepps;
    Node(float x, float y, float a);
    float CalculateCost(Node &target);
    float ReedShepsDistance(const std::vector<Node::NodeReedSheps>& pth);
    std::vector<std::vector<Node::NodeReedSheps>> CalculateReedSheps(Node &target);
    std::vector<Node::NodeReedSheps> TimeFlip(std::vector<Node::NodeReedSheps> in);
    std::vector<Node::NodeReedSheps> Reflect(std::vector<Node::NodeReedSheps> in);
    bool IsDirectPath(Node &target);
};

class HybridAStar {

    float step;
    Node target;
    Node current;
    std::vector<Node> frontier;
    std::vector<Node> explored;
    
    public:

    float min_radius;
    std::vector<Point>* enviorment;

    HybridAStar(float min_radius, float step, Node target, Node current);
    int CalculatePath(std::vector<Node::NodeReedSheps> *path_ordered, Node::NodeReedSheps current);
    void SetEnviorment(std::vector<Point> *env);
};

std::vector<Node> Subdevide(std::vector<Node::NodeReedSheps>& path, Node *root);