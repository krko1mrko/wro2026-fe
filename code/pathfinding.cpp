#include "pathfinding.h"
#include <algorithm>
#include <math.h>
#include "consts.h"
#include <iostream>
#include "utils.h"

Node::Node(float x = 0, float y = 0, float a = 0) {
    this->x = x;
    this->y = y;
    this->a = fmod(a, 2 * PI);
    this->cost = -1;
}

float Node::CalculateCost(Node &target) {
    float min_d = 100000; //Large number
    for (int i = 0; i < pathfinder->enviorment->size(); i++) {
        float d = pathfinder->enviorment->at(i).DistanceSq(Point(x,y));
        if (d < min_d) {
            min_d = d;
        }
    }

    std::vector<std::vector<Node::NodeReedSheps>> paths = CalculateReedSheps(target);
    float distance = ReedShepsDistance(paths[0]);
    
    if (sqrt(min_d) < 0.1) { 
        cost = 10000;
    }
    else {
        cost = distance;
    }
    return cost;
}

float Node::ReedShepsDistance(const std::vector<Node::NodeReedSheps>& pth) {
    float sum = 0;
    for (const Node::NodeReedSheps& n : pth) {
        sum += n.distance;
    }
    return sum * this->pathfinder->min_radius;
}
std::vector<float> ToPolar(float x, float y) {
    float r = sqrt (x * x + y * y);
    float a = atan2(y, x);
    return {r,a};
}



std::vector<Node::NodeReedSheps> path1(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float u = temp[0];
    float t = temp[1];
    float v = AngleMod(da - t);
    
    res.push_back(Node::NodeReedSheps(t, -1, 1));
    res.push_back(Node::NodeReedSheps(u, 0, 1));
    res.push_back(Node::NodeReedSheps(v, -1, 1));

    return res;
}
std::vector<Node::NodeReedSheps> path2(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;

    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho * rho >= 4) {
        float u = sqrt(rho * rho - 4);
        float t = AngleMod(t1 + atan2(2, u));
        float v = AngleMod(t - da);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 0, 1));
        res.push_back(Node::NodeReedSheps(v, 1, 1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path3(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);

    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho <= 4) {
        float a = acos(rho / 4);
        float t = AngleMod(t1 + PI / 2 + a);
        float u = AngleMod(PI - 2 * a);
        float v = AngleMod(da - t - u);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 1, -1));
        res.push_back(Node::NodeReedSheps(v, -1, 1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path4(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho <= 4) {
        float a = acos(rho / 4);
        float t = AngleMod(t1 + PI / 2 + a);
        float u = AngleMod(PI - 2 * a);
        float v = AngleMod(t + u - da);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 1, -1));
        res.push_back(Node::NodeReedSheps(v, -1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path5(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float rho = temp[0];
    float t1 = temp[1];


    if (rho <= 4) {
        float u = acos(1 - rho * rho / 8);
        float a = asin(2 * sin(u) / rho);
        float t = AngleMod(t1 + PI / 2 - a);
        float v = AngleMod(t - u - da);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 1, 1));
        res.push_back(Node::NodeReedSheps(v, -1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path6(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho <= 4) {
        if (rho <= 2) {
            float a = acos((rho + 2) / 4);
            float t = AngleMod(t1 + PI / 2 + a);
            float u = AngleMod(a);
            float v = AngleMod(da - t + 2 * u);
            
            res.push_back(Node::NodeReedSheps(t, -1, 1));
            res.push_back(Node::NodeReedSheps(u, 1, 1));
            res.push_back(Node::NodeReedSheps(u, -1, -1));
            res.push_back(Node::NodeReedSheps(v, 1, -1));
        }
        else {
            float a = acos((rho - 2) / 4);
            float t = AngleMod(t1 + PI / 2 - a);
            float u = AngleMod(PI - a);
            float v = AngleMod(da - t + 2 * u);
            
            res.push_back(Node::NodeReedSheps(t, -1, 1));
            res.push_back(Node::NodeReedSheps(u, 1, 1));
            res.push_back(Node::NodeReedSheps(u, -1, -1));
            res.push_back(Node::NodeReedSheps(v, 1, -1));
        }
    }
    return res;
}
std::vector<Node::NodeReedSheps> path7(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];
    float u1 = (20 - rho * rho) / 16;

    if (rho <= 6 && 0 <= u1 && u1 <= 1) {
        float u = acos(u1);
        float a = asin(2 * sin(u) / rho);
        float t = AngleMod(t1 + PI / 2 + a);
        float v = AngleMod(t - da);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 1, -1));
        res.push_back(Node::NodeReedSheps(u, -1, -1));
        res.push_back(Node::NodeReedSheps(v, 1, 1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path8(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho >= 2) {
        float u = sqrt(rho * rho - 4) - 2;
        float a = atan2(2, u + 2);
        float t = AngleMod(t1 + PI / 2 + a);
        float v = AngleMod(t - da + PI / 2);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(PI / 2, 1, -1));
        res.push_back(Node::NodeReedSheps(u, 0, -1));
        res.push_back(Node::NodeReedSheps(v, -1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path9(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx - sin(da), dy - 1 + cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho >= 2) {
        float u = sqrt(rho * rho - 4) - 2;
        float a = atan2(u + 2, 2);
        float t = AngleMod(t1 + PI / 2 - a);
        float v = AngleMod(t - da - PI / 2);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 0, 1));
        res.push_back(Node::NodeReedSheps(PI / 2, 1, 1));
        res.push_back(Node::NodeReedSheps(v, -1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path10(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho >= 2) {
        float t = AngleMod(t1 + PI / 2);
        float u = rho - 2;
        float v = AngleMod(da - t - PI / 2);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(PI / 2, 1, -1));
        res.push_back(Node::NodeReedSheps(u, 0, -1));
        res.push_back(Node::NodeReedSheps(v, 1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path11(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho >= 2) {
        float t = AngleMod(t1);
        float u = rho - 2;
        float v = AngleMod(da - t - PI / 2);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(u, 0, 1));
        res.push_back(Node::NodeReedSheps(PI / 2 , -1, 1));
        res.push_back(Node::NodeReedSheps(v, 1, -1));
    }
    return res;
}
std::vector<Node::NodeReedSheps> path12(float dx, float dy, float da) {
    std::vector<Node::NodeReedSheps> res;
    da = AngleMod(da);
    auto temp = ToPolar(dx + sin(da), dy - 1 - cos(da));
    float rho = temp[0];
    float t1 = temp[1];

    if (rho >= 4) {
        float u = sqrt(rho * rho - 4) - 4;
        float a = atan2(2, u + 4);
        float t = AngleMod(t1 + PI / 2 + a);
        float v = AngleMod(t - da);
        
        res.push_back(Node::NodeReedSheps(t, -1, 1));
        res.push_back(Node::NodeReedSheps(PI / 2, 1, -1));
        res.push_back(Node::NodeReedSheps(u, 0, -1));
        res.push_back(Node::NodeReedSheps(PI/2, -1, -1));
        res.push_back(Node::NodeReedSheps(v, 1, 1));
    }
    return res;
}

bool IsColiding(std::vector<Point> *env, std::vector<Node::NodeReedSheps>& path, Node *root) {
    float x = root->x;
    float y = root->y;
    float a = root->a;

    for (Node::NodeReedSheps& n : path) {
        if (isnanf(n.distance)) {
            return true;
        }
        float t = 0;
        while (true) {
    
        float min_d = 100000; //Large number
        for (int i = 0; i < env->size(); i++) {
            float d = env->at(i).DistanceSq(Point(x,y));
            if (d < min_d) {
                min_d = d;
            }
        }
        if (sqrt(min_d) < 0.1) {
            return true;
        }
        
        min_d = std::min(n.distance * root->pathfinder->min_radius - t, min_d);

        if (n.steering != 0) {
            x += (sin(a) - sin(a - min_d/root->pathfinder->min_radius * n.steering * n.drive)) * root->pathfinder->min_radius * n.steering;
            y -= (cos(a) - cos(a - min_d/root->pathfinder->min_radius * n.steering * n.drive)) * root->pathfinder->min_radius * n.steering;
            a -= min_d/root->pathfinder->min_radius * n.steering * n.drive;
        }
        else {
            x += cos(a) * min_d * n.drive;
            y += sin(a) * min_d * n.drive;
        }
        t += min_d;
        if (abs(t - n.distance  * root->pathfinder->min_radius) < 0.00001) {
            break;
        }
    }
}
return false;
}

std::vector<Node> Subdevide(std::vector<Node::NodeReedSheps>& path, Node *root) {
    float x = root->x;
    float y = root->y;
    float a = root->a;
    std::vector<Node> res;
    for (Node::NodeReedSheps& n : path) {
        float t = 0;
        while (true) {
            float min_d = std::min(n.distance - t, 0.02f);

            if (n.steering != 0) {
                x += (sin(a) - sin(a - min_d/root->pathfinder->min_radius * n.steering * n.drive)) * root->pathfinder->min_radius * n.steering;
                y -= (cos(a) - cos(a - min_d/root->pathfinder->min_radius * n.steering * n.drive)) * root->pathfinder->min_radius * n.steering;
                a -= min_d/root->pathfinder->min_radius * n.steering * n.drive;
            }
            else {
                x += cos(a) * min_d * n.drive;
                y += sin(a) * min_d * n.drive;
            }
            t += min_d;

            res.push_back(Node(x,y,a));

            if (abs(t - n.distance) < 0.00001) {
                break;
            }
        }
    }

    return res;
}


std::vector<std::vector<Node::NodeReedSheps>> Node::CalculateReedSheps(Node &target) {

        float dx = cos(this->a) * (target.x - this->x) + sin(this->a) * (target.y - this->y);
        dx /= this->pathfinder->min_radius; 
        float dy = -sin(this->a) * (target.x - this->x) + cos(this->a) * (target.y - this->y);
        dy /= this->pathfinder->min_radius;
        float da = target.a - this->a;

        std::vector<std::vector<Node::NodeReedSheps>> paths;

        paths.push_back(path1(dx, dy, da));
        paths.push_back(path2(dx, dy, da));
        paths.push_back(path3(dx, dy, da));
        paths.push_back(path4(dx, dy, da));
        paths.push_back(path5(dx, dy, da));
        paths.push_back(path6(dx, dy, da));
        paths.push_back(path7(dx, dy, da));
        paths.push_back(path8(dx, dy, da));
        paths.push_back(path9(dx, dy, da));
        paths.push_back(path10(dx, dy, da));
        paths.push_back(path11(dx, dy, da));
        paths.push_back(path12(dx, dy, da));

        paths.push_back(TimeFlip(path1(-dx, dy, -da)));
        paths.push_back(TimeFlip(path2(-dx, dy, -da)));
        paths.push_back(TimeFlip(path3(-dx, dy, -da)));
        paths.push_back(TimeFlip(path4(-dx, dy, -da)));
        paths.push_back(TimeFlip(path5(-dx, dy, -da)));
        paths.push_back(TimeFlip(path6(-dx, dy, -da)));
        paths.push_back(TimeFlip(path7(-dx, dy, -da)));
        paths.push_back(TimeFlip(path8(-dx, dy, -da)));
        paths.push_back(TimeFlip(path9(-dx, dy, -da)));
        paths.push_back(TimeFlip(path10(-dx, dy, -da)));
        paths.push_back(TimeFlip(path11(-dx, dy, -da)));
        paths.push_back(TimeFlip(path12(-dx, dy, -da)));

        paths.push_back(Reflect(path1(dx, -dy, -da)));
        paths.push_back(Reflect(path2(dx, -dy, -da)));
        paths.push_back(Reflect(path3(dx, -dy, -da)));
        paths.push_back(Reflect(path4(dx, -dy, -da)));
        paths.push_back(Reflect(path5(dx, -dy, -da)));
        paths.push_back(Reflect(path6(dx, -dy, -da)));
        paths.push_back(Reflect(path7(dx, -dy, -da)));
        paths.push_back(Reflect(path8(dx, -dy, -da)));
        paths.push_back(Reflect(path9(dx, -dy, -da)));
        paths.push_back(Reflect(path10(dx, -dy, -da)));
        paths.push_back(Reflect(path11(dx, -dy, -da)));
        paths.push_back(Reflect(path12(dx, -dy, -da)));


        paths.push_back(TimeFlip(Reflect(path1(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path2(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path3(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path4(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path5(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path6(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path7(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path8(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path9(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path10(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path11(-dx, -dy, da))));
        paths.push_back(TimeFlip(Reflect(path12(-dx, -dy, da))));

        for (int i = 0; i < paths.size(); i++) {
            if (paths[i].size() == 0) {
                paths.erase(paths.begin() + i);
                i--;
            }
        }

        std::sort(paths.begin(), paths.end(), [this](const std::vector<Node::NodeReedSheps>& a, const std::vector<Node::NodeReedSheps>& b) {return this->ReedShepsDistance(a) < this->ReedShepsDistance(b);});
        return paths;
    }


std::vector<Node::NodeReedSheps> Node::TimeFlip(std::vector<Node::NodeReedSheps> in) {
    std::vector<Node::NodeReedSheps> out = in;
    for (Node::NodeReedSheps& n : out) {
        n.ReverseDriveing();
    }
    return out;
}

std::vector<Node::NodeReedSheps> Node::Reflect(std::vector<Node::NodeReedSheps> in) {
    std::vector<Node::NodeReedSheps> out = in;
    for (Node::NodeReedSheps& n : out) {
        n.ReverseSteering();
    }
    return out;
}

bool Node::IsDirectPath(Node &target) {
    //ToDo unfinished
    float t = 0;
    std::vector<Node::NodeReedSheps> pth = CalculateReedSheps(target)[0];
    while (true) {
    
    float min_d = 100000; //Large number
    for (int i = 0; i < pathfinder->enviorment->size(); i++) {
        float d = pathfinder->enviorment->at(i).DistanceSq(Point(x,y));
        if (d < min_d) {
            min_d = d;
        }
    }

    }
}


HybridAStar::HybridAStar(float min_radius, float step, Node target, Node current){
    this->min_radius = min_radius;
    this->step = step;
    this->target = target;
    this->current = current;
    this->current.pathfinder = this;
}

void HybridAStar::SetEnviorment(std::vector<Point> *env) {
    this->enviorment = env;
}

int HybridAStar::CalculatePath(std::vector<Node::NodeReedSheps> *path_ordered,  Node::NodeReedSheps current_drive) {


    current.CalculateCost(target);
    frontier.push_back(current);
    
   // std::vector<Node::NodeReedSheps> path = current.CalculateReedSheps(target)[0];
    //std::cout << "IS COLLIDING: " << IsColiding(enviorment, path, &current) << std::endl;
    //*path_ordered = Subdevide(enviorment, path, &current);
    //return 0;
    
    int num = 0;
    while (true) {
        num++;
        if (frontier.size() <= 0) {
            return -1;
        }
        auto element = std::min_element(frontier.begin(), frontier.end(), [](const Node& a, const Node& b){return a.cost < b.cost;});
        
        std::vector<std::vector<Node::NodeReedSheps>> paths_direct = (*element).CalculateReedSheps(target);
        if (current_drive.distance > 0.1) {
        //std::sort(paths_direct.begin(), paths_direct.begin() + 2, [current_drive](const std::vector<Node::NodeReedSheps>& a, const std::vector<Node::NodeReedSheps>& b){
        //    return ((current_drive.steering == a[0].steering) || (current_drive.steering != b[0].steering));
        // });
        }
        for (std::vector<Node::NodeReedSheps>& path_direct : paths_direct) {
        if (!IsColiding(enviorment, path_direct, &(*element))) {
            
            //std::reverse(path_direct.begin(), path_direct.end());

            for (Node::NodeReedSheps& n : path_direct) {
                n.distance *= min_radius;
            }

            int next = (*element).owner;
            path_ordered->push_back((*element).reedShepps);
            while (next != -1) {
                path_ordered->push_back(explored[next].reedShepps);
                next = explored[next].owner;    
            }
            std::reverse(path_ordered->begin(), path_ordered->end());

            path_ordered->insert(path_ordered->end(), path_direct.begin(), path_direct.end());
            return num;
        }
    }

        if (num > 100) {
            return -1;
        }

        std::vector<Node> branches;

        Node b1 = Node(
            (*element).x + (sin((*element).a) - sin((*element).a - step/min_radius)) * min_radius,
            (*element).y - (cos((*element).a) - cos((*element).a - step/min_radius)) * min_radius,
            (*element).a - step/min_radius);
        b1.reedShepps = Node::NodeReedSheps(step, 1, 1);
        Node b2 = Node(
            (*element).x - (sin((*element).a) - sin((*element).a + step/min_radius)) * min_radius,
            (*element).y + (cos((*element).a) - cos((*element).a + step/min_radius)) * min_radius,
            (*element).a + step/min_radius);
        b2.reedShepps = Node::NodeReedSheps(step, -1, 1);
        Node b3 = Node(
            (*element).x + (sin((*element).a) - sin((*element).a + step/min_radius)) * min_radius,
            (*element).y - (cos((*element).a) - cos((*element).a + step/min_radius)) * min_radius,
            (*element).a + step/min_radius);
        b3.reedShepps = Node::NodeReedSheps(step, 1, -1);
        Node b4 = Node(
            (*element).x + (sin((*element).a) - sin((*element).a - step/min_radius)) * min_radius,
            (*element).y - (cos((*element).a) - cos((*element).a - step/min_radius)) * min_radius,
            (*element).a - step/min_radius);
        b4.reedShepps = Node::NodeReedSheps(step, -1, -1);
        Node b5 = Node(
            (*element).x + cos((*element).a) * step,
            (*element).y + sin((*element).a) * step,
            (*element).a);
        b5.reedShepps = Node::NodeReedSheps(step, 0, 1);
        Node b6 = Node(
            (*element).x - cos((*element).a) * step,
            (*element).y - sin((*element).a) * step,
            (*element).a);
        b6.reedShepps = Node::NodeReedSheps(step, 0, -1);
        
        branches.push_back(b1);
        branches.push_back(b2);
        branches.push_back(b3);
        branches.push_back(b4);
        branches.push_back(b5);
        branches.push_back(b6);

        for (std::vector<Node>::iterator b = branches.begin(); b != branches.end(); ++b) {
            for (Node &n : explored) {
                if(
                    abs(n.x - (*b).x) < 0.06 &&
                    abs(n.y - (*b).y) < 0.06 &&
                    abs(n.a - (*b).a) < 0.3) {
                    branches.erase(b);
                    b--;
                    break;
                }
            }
        }

        explored.push_back(*element);
        for (int i = 0; i < branches.size(); i++) {
            branches[i].owner = explored.size() - 1;
            branches[i].pathfinder = (*element).pathfinder;
            branches[i].CalculateCost(target);
            if (branches[i].cost > 1000) {
                branches.erase(branches.begin() + i);
                i--;
            }
        }

        frontier.erase(element);
        frontier.insert(frontier.end(), branches.begin(), branches.end());
    }
    return num;
}