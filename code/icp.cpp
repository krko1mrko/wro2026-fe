#include "icp.h"
#include <iostream>
#include <algorithm>
#include <math.h>
#include "KernelSource.h"
#include "Config.h"
#include <cstring>
#include "consts.h"

QuadTree::QuadTree(float x, float y, float w) {
    this->x = x;
    this->y = y;
    this->w = w;
}
void QuadTree::Clear() {
    tree.clear();
}

bool QuadTree::IsQuadIntersecting(Quad& quad, Point* test) {
    int depth = (quad.id >> 28) & 0xF;
    float wq = this->w / (float)(1 << depth);
    float xq = this->x;
    float yq = this->y;
    xq += this->w * (float)(quad.id & 0x3FFF) / (float)(1 << 14);
    yq += this->w * (float)((quad.id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);
    bool res = (fmaxf(test->x, xq) == fminf(test->x, xq + wq) &&
        fmaxf(test->y, yq) == fminf(test->y, yq + wq)) && !(test->x == xq + wq || test->y == yq + wq);
    return res;
}

void QuadTree::GenerateTreeRecursive(std::vector<Point*>& points, int index_quad) {
    
    std::vector<Quad> child;
    std::vector<std::vector<Point*>> child_points;
    for (int i = 0; i < 4; i++) {
        child.push_back(Quad());
        int index = child.size() - 1;
        int depth = (tree[index_quad].id >> 28) & 0xF;
        child[index].id = (depth + 1) << 28;
        child[index].id |= (tree[index_quad].id & 0x0FFFFFFF) | (((i & 2) >> 1) << (13 - depth));
        child[index].id |= ((i & 1) << (27 - depth));
        std::vector<Point*> cpoints;
        for (Point* p : points) {
            bool test = IsQuadIntersecting(child[index], p);
            if (test) {
                cpoints.push_back(p);
            }
        }
        if (cpoints.size() != 0) {
            //child[index].children |= ((tree.size() + 1) << 4) & 0xFFFFFFF0;
            //child[index].children |= ((index_quad) << 18) & ~0x0003FFFF;
            if (depth == 13 /* || cpoints.size() == 1 */) {
                child[index].children |= 0xF;
            }
            //tree.push_back(child[index]);
            child_points.push_back(cpoints);
        }
        else {
            child.pop_back();
        }
    }
    
    tree[index_quad].children |= (uint32_t)(child.size() & 0xF);
    int index = tree.size();
    tree[index_quad].children = (tree[index_quad].children & 0x0000000F) | (uint32_t)(index << 4);
    for (int i = 0; i < child.size(); i++) {
        tree.push_back(child[i]);
    }
    for (int i = 0; i < child.size(); i++) {
        if (((tree[index + i].id >> 28) & 0xF) != 14 /* && child_points[i].size() != 1*/) {
            GenerateTreeRecursive(child_points[i], index + i);
        }
        else if((tree[index + i].children & 0xF) == 0x00){
            ////_ASSERT(0);
        }
    }
}

void QuadTree::GenerateTree(std::vector<Point*>& points, int depth) {
    
    //std::cout << "Generating quad tree for " << points.size() << " points\n";
    Quad root = Quad();
    root.id = 0x0;
    
    tree.push_back(root);
    tree[0].children |= ((tree.size() + 1) << 4) & 0x0003FFF0;
    
    GenerateTreeRecursive(points, 0);
    //std::cout << "Done generating | Tree size: " << tree.size() * sizeof(Quad) << " bytes\n";
}

Point* QuadTree::NavigateTree(Point* point) {
    int i = 0;
    uint32_t previous = 0;
    float point_min_d = 10000;
    while (true) {
        uint32_t children = tree[i].children;
        int next_start = ((children >> 4) & 0x00003FFF);
        int previous = ((children >> 18) & 0x00003FFF);
        float min_distance = 10000;
        int min_j = 0;
        int depth = (tree[next_start].id >> 28) & 0xF;
        float wq = this->w / (float)(1 << depth);
        for (unsigned int j = 0; j < (children & 0xF); j++) {
            uint32_t id = tree[next_start + j].id;
            float xq = this->x;
            float yq = this->y;
            xq += this->w * (float)(id & 0x3FFF) / (float)(1 << 14);
            yq += this->w * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);
            float dis = (point->x - xq - wq/2) * (point->x - xq - wq/2) + (point->y - yq - wq/2) * (point->y - yq - wq/2);
            if (dis < min_distance) {
                min_distance = dis;
                min_j = j;
            }
        }
        i = next_start + min_j;
        if ((tree[i].children & 0xF) == 0xF) {
            float xq = this->x;
            float yq = this->y;
            uint32_t id = tree[next_start + min_j].id;
            xq += this->w * (float)(id & 0x3FFF) / (float)(1 << 14);
            yq += this->w * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);
            point_min_d = min_distance;
            return new Point(xq + wq/2, yq + wq/2);
        }
    }
}


QuadTreeOpenCL::QuadTreeOpenCL(float x, float y, float w) {
    this->quad_tree = new QuadTree(x, y, w);
    this->results_data = nullptr;
}
QuadTreeOpenCL::~QuadTreeOpenCL() {
    delete this->quad_tree;
}
void QuadTreeOpenCL::CompileKernel() {
    
    std::cout << "\n===========================================\n";
    std::cout << "Compiling OpenCl kernel\n";


    std::cout << "Getting number of OpenCl platforms:";
    cl_uint platform_num = 0;
    int err = clGetPlatformIDs(0, NULL, &platform_num);
    if (err != CL_SUCCESS)
    {
        std::cout << "\tError: Failed to get number of platforms!\n";
    }
    else {
        std::cout << "\t" << platform_num << "\n";
    }
    

    std::cout << "Getting OpenCl platforms ids:";
    cl_platform_id *platform_id = (cl_platform_id*)malloc(platform_num * sizeof(cl_platform_id));
    err = clGetPlatformIDs(platform_num, platform_id, NULL);
    if (err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to get Platform ids!\n";
    }
    else {
        std::cout << "\t" << "Success\n";
    }
    
    std::cout << "Creating a device group:";
    cl_uint device_num = 0;
    for (int i = 0; i < platform_num; i++) {
        err = clGetDeviceIDs(platform_id[i], CL_DEVICE_TYPE_GPU, 1, &device_id, &device_num);
        if (err != CL_SUCCESS)
        {
            std::cout << "Error: Failed to create a device group for platform " << i << " \n";
        }
        else {
            std::cout << "\tSuccess\n";
            break;
        }
    }

    std::cout << "Creating a OpenCl context:";
    context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);
    if (!context)
    {
        std::cout << "Error: Failed to create a compute context!\n";
    }
    else
    {
        std::cout << "\tSuccess\n";
    }

    std::cout << "Creating a OpenCl command queue:";
    commands = clCreateCommandQueueWithProperties(context, device_id, 0, &err);
    if (!commands)
    {
        std::cout << "Error: Failed to create a command queue!\n";
    }
    else
    {
        std::cout << "\tSuccess\n";
    }

    std::cout << "Creating a OpenCl program:";
    program = clCreateProgramWithSource(context, 1, (const char**)&KernelSource, NULL, &err);
    if (!program)
    {
        std::cout << "Error: Failed to create compute program!\n";
    }
    else
    {
        std::cout << "\tSuccess\n";
    }

    std::cout << "Building the OpenCl kernel:";
    err = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        size_t len;
        char buffer[16000];

        std::cout << "Error: Failed to build program executable!\n";
        clGetProgramBuildInfo(program, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        std::cout.write(buffer, len);
    }
    else
    {
        std::cout << "\tSuccess\n";
    }


    std::cout << "Creating the OpenCl kernel:";
    kernel = clCreateKernel(program, "quad_tree", &err);
    if (!kernel || err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to create compute kernel!\n";
    }
    else
    {
        std::cout << "\tSuccess\n";
    }
}
void QuadTreeOpenCL::Initilize(std::vector<Point*>& points, int depth, int src_point_size) {
    this->quad_tree->GenerateTree(points, depth);

    int err;
    float consts_data[6] = { this->quad_tree->x, this->quad_tree->y, this->quad_tree->w, 0, 0, 0 };
    this->points_loaded = false;
    if (compiled) {
        clReleaseMemObject(tree);
        clReleaseMemObject(points_ref);
        clReleaseMemObject(consts);
        clReleaseMemObject(results);
    }
    //if (!compiled) {
    tree = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(uint32_t) * 2 * BUFFER_SIZE, NULL, NULL);
    points_ref = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(float) * 2 * 2048, NULL, NULL);
    consts = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(uint32_t) * 6, NULL, NULL);
    results = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(float) * 2 * 2048, NULL, NULL);
    //}
    if (!tree || !points_ref || !consts || !results)
    {
        std::cout << "Error: Failed to allocate device memory!\n";
    }

    err = clEnqueueWriteBuffer(commands, tree, CL_TRUE, 0, this->quad_tree->tree.size() * sizeof(QuadTree::Quad), this->quad_tree->tree.data(), 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        std::cout << "Trying to write: " << this->quad_tree->tree.size() * sizeof(QuadTree::Quad) << " in to: " << sizeof(uint32_t) * 2 * BUFFER_SIZE << "\n";
        std::cout << "Error: Failed to write to source array tree!" << err << "\n";
    }

    err = clEnqueueWriteBuffer(commands, consts, CL_TRUE, 0, 6 * sizeof(float), consts_data, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to write to source array consts!" << err << "\n";
    }
    this->compiled = true;
}
Point* QuadTreeOpenCL::NavigateTree(std::vector<Point>* points, size_t size, float dx, float dy, float da) {

    if (!this->points_loaded) {
        Point* p_buf = (Point*)malloc(2048 * sizeof(Point));
        memset(p_buf, 0, 2048 * sizeof(Point));
        memcpy(p_buf, points->data(), std::min((int)size, 2048) * sizeof(Point));
        int err = clEnqueueWriteBuffer(commands, points_ref, CL_TRUE, 0, sizeof(Point) * 2048, p_buf, 0, NULL, NULL);
        free(p_buf);
        if (err != CL_SUCCESS)
        {
            std::cout << "Error: Failed to write to source array points!" << err << "\n";
        }
        this->points_loaded = true;
        
    }
    else {
        float consts_data[6] = { this->quad_tree->x, this->quad_tree->y, this->quad_tree->w, dx, dy, da };
        int err = clEnqueueWriteBuffer(commands, consts, CL_TRUE, 0, 6 * sizeof(float), consts_data, 0, NULL, NULL);
        if (err != CL_SUCCESS)
        {
            std::cout << "Error: Failed to write to source array consts!" << err << "\n";
        }
    }
    int err = 0;
    err = clSetKernelArg(kernel, 0, sizeof(cl_mem), &tree);
    err |= clSetKernelArg(kernel, 1, sizeof(cl_mem), &consts);
    err |= clSetKernelArg(kernel, 2, sizeof(cl_mem), &points_ref);
    err |= clSetKernelArg(kernel, 3, sizeof(cl_mem), &results);
    if (err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to set kernel arguments!" << err << "\n";
    }

    err = clGetKernelWorkGroupInfo(kernel, device_id, CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), &local, NULL);
    if (err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to retrieve kernel work group info!" << err << "\n";
    }

    global = 2048;
    err = clEnqueueNDRangeKernel(commands, kernel, 1, NULL, &global, &local, 0, NULL, NULL);
    if (err)
    {
        std::cout << "Error: Failed to execute kernel!" << err << "\n";
    }


    if (results_data != nullptr) {
        free(results_data);
    }
    results_data = (Point*)malloc(sizeof(Point) * 2048);

    clFinish(commands);

    err = clEnqueueReadBuffer(commands, results, CL_TRUE, 0, sizeof(Point) * 2048, results_data, 0, NULL, NULL);
    if (err != CL_SUCCESS)
    {
        std::cout << "Error: Failed to read output array!" << err << "\n";
    }

    return results_data;
}
void QuadTreeOpenCL::Clear() {
    this->quad_tree->Clear();
    this->points_loaded = false;
    this->quad_tree_loaded = false;
}
void QuadTreeOpenCL::ReuseQuadClear() {
    this->points_loaded = false;
    this->quad_tree_loaded = true;
}

QuadTreeOpenCL* PointPair::quad_tree_opencl = new QuadTreeOpenCL(DOMAIN_SIZE);

    
PointPair::PointPair() {
    a = nullptr;
    b = nullptr;
}
PointPair::PointPair(Point* a, Point* b) {
	this->a = a;
	this->b = b;
}

void PointPair::MatchPoints(std::vector<Point>& src, std::vector<Point>& reference, std::vector<PointPair>& dst, bool reuse_quad_tree) {

    if(!reuse_quad_tree) {
        PointPair::quad_tree_opencl->Clear();
    }
    else {
        PointPair::quad_tree_opencl->ReuseQuadClear();
    }
    std::vector<Point*> pointer_reference;
    for (Point& p : reference) {
        pointer_reference.push_back(&p);
    }
    if (!PointPair::quad_tree_opencl->quad_tree_loaded) {
        PointPair::quad_tree_opencl->Initilize(pointer_reference, 14, pointer_reference.size());
    }

    Point* res = PointPair::quad_tree_opencl->NavigateTree(&src, src.size(), 0, 0, 0);
    for (int i = 0; i < std::min((int)src.size(), 2048); i++) {
        res[i].Transform(0, 0, -0.785398);
        dst.push_back(PointPair(&src[i], &res[i]));
    }
}

void PointPair::MatchPairs(std::vector<PointPair>& src, std::vector<Point>& reference, float dx, float dy, float da) {

   Point* res = PointPair::quad_tree_opencl->NavigateTree(nullptr, src.size(), dx, dy, da);
   for (int i = 0; i < std::min((int)src.size(), 2048); i++) {
       res[i].Transform(0, 0, -0.785398);
       src[i].b = &res[i];
   }

}


ICP::ICP() {

}



void ICP::Iterate(std::vector<PointPair>& point_pairs, float& dx, float& dy, float& a) {

    if (point_pairs.size() == 0) {
        return;
    }

    float x_avg = 0;
    float y_avg = 0;
    float xr_avg = 0;
    float yr_avg = 0;
    int i = 0;
    for (PointPair &p : point_pairs) {
        if (p.a->DistanceSq(p.b) > 0.1){
            continue;
        }
        i++;
        xr_avg += p.a->x;
        yr_avg += p.a->y;

        x_avg += p.b->x;
        y_avg += p.b->y;

    }
    xr_avg /= i;
    yr_avg /= i;
    x_avg /= i;
    y_avg /= i;


    float c = 0;
    float s = 0;

    for (PointPair& p : point_pairs) {
        if (p.a->DistanceSq(p.b) > 0.1){
            continue;
        }
        s += ((p.a->y - yr_avg) * (p.b->x - x_avg) - (p.a->x - xr_avg) * (p.b->y - y_avg));
        c += ((p.a->y - yr_avg) * (p.b->y - y_avg) + (p.a->x - xr_avg) * (p.b->x - x_avg));
    }
    
    dx = (x_avg - xr_avg);
    dy = (y_avg - yr_avg);
    a = -atan2f(s, c);

}

int ICP::Compute(std::vector<Point>& src, std::vector<Point>& reference, float& dx, float& dy, float& a, double& cost, bool reuse_quad_tree) {
    
    cost = 10;
    double cost_new = 0;
    float delta_x = 0;
    float delta_y = 0;
    float delta_a = 0;
    
    int iterations = 0;
    Point delta_point = Point(0, 0);

    for (Point& p : reference) {
        p.Transform(0, 0, 0.785398);
    }

    std::vector<PointPair> pairs;
    PointPair::MatchPoints(src, reference, pairs, reuse_quad_tree);

    for (Point& p : reference) {
        p.Transform(0, 0, -0.785398);
    }

        
        
    cost = cost_new;
    cost_new = 0;


    for (int i = 0; i < MAX_ITERATIONS && abs(sqrt(cost) - sqrt(cost_new / (double)src.size())) > CONVERGANCE_CONDITION || i < MIN_ITERATIONS; i++) {
        iterations++;
        Iterate(pairs, delta_x, delta_y, delta_a);
        delta_a *= 1.0;

        delta_point.Transform(delta_x, delta_y, 0);
        delta_point.Transform(0, 0, delta_a);
        a += delta_a;
        
        
        cost = cost_new;
        cost_new = 0;

        for (PointPair& p : pairs) {
            p.a->Transform(delta_x, delta_y, 0);
            p.a->Transform(0, 0, delta_a);
            cost_new += p.a->DistanceSq(*p.b);
        }

        
        PointPair::MatchPairs(pairs, reference, delta_point.x, delta_point.y, a);
        cost /= (double)src.size();
    }
    cost = cost_new / (double)src.size();
    dx = delta_point.x;
    dy = delta_point.y;
    return iterations;
}

void ICP::MergeMaps(std::vector<Point>& map, std::vector<std::vector<Point>>& read, int lookback, cv::Mat& allowed) {
    //Merge read1 into map while using read2 to check consistancy
    
    if (read.size() <= lookback + 1) {
        return;
    }

    for (Point& p : map) {
        p.Transform(0, 0, 0.785398);
    }
    std::vector<PointPair> pair_map;
    PointPair::MatchPoints(read[read.size()-1-lookback], map, pair_map, false);
    for (Point& p : map) {
        p.Transform(0, 0, -0.785398);
    }

    for (PointPair& pp : pair_map) {
        Point *a = new Point(pp.a->x, pp.a->y);
        Point *b = new Point(pp.b->x, pp.b->y);
        pp.a = a;
        pp.b = b;
    }

    std::vector<std::vector<PointPair>> pairs;

    for (int i = 0; i < lookback; i++) {
        for (Point& p : read[read.size()-1 - i]) {
            p.Transform(0, 0, 0.785398);
        }
        std::vector<PointPair> pair_read;
        PointPair::MatchPoints(read[read.size()-1 - lookback], read[read.size()-1 - i], pair_read, false);
        

        for (Point& p : read[read.size()-1 - i]) {
            p.Transform(0, 0, -0.785398);
        }
        for (PointPair& pp : pair_read) {
            Point *a = new Point(pp.a->x, pp.a->y);
            Point *b = new Point(pp.b->x, pp.b->y);
            pp.a = a;
            pp.b = b;
        }
        pairs.push_back(pair_read);
    }

    float epsilon = 0.01;
    for (int i = 0; i < pair_map.size(); i++) {
        bool condition = sqrt(pair_map[i].a->DistanceSq(pair_map[i].b)) > 0.1;
        for (std::vector<PointPair>& pp : pairs) {
            condition &= sqrt(pp[i].a->DistanceSq(pp[i].b)) < 0.01;
        }
        if (condition) {
            if (allowed.at<uchar>(std::clamp((int)(pair_map[i].a->x * 64 + 256), 0, 512), std::clamp((int)(pair_map[i].a->y * 64 + 256), 0, 512)) < 128) {
                map.push_back(Point(*pair_map[i].a));
            }
        }
    }

    for (PointPair& pp : pair_map) {
        delete pp.a;
        delete pp.b;
    }
    for (std::vector<PointPair>& vpp : pairs) {
    for (PointPair& pp : vpp) {
        delete pp.a;
        delete pp.b;
    }}
}

float ICP::PositionCost(std::vector<Point> src, std::vector<Point>& reference, float dx, float dy, float a, bool reuse_quad_tree, int icp_iterations) {

    double cost = 0;
    float delta_x = 0;
    float delta_y = 0;
    float delta_a = 0;
    
    Point delta_point = Point(0, 0);


    for (Point& p : src) {
        p.Transform(dx, dy, a);
    }
    a = 0;
    for (Point& p : reference) {
        p.Transform(0, 0, 0.785398);
    }

    std::vector<PointPair> pairs;
    PointPair::MatchPoints(src, reference, pairs, reuse_quad_tree);

    for (Point& p : reference) {
        p.Transform(0, 0, -0.785398);
    }




    for (int i = 0; i < icp_iterations; i++) {
        Iterate(pairs, delta_x, delta_y, delta_a);
        delta_a *= 0.0;

        delta_point.Transform(delta_x, delta_y, 0);
        delta_point.Transform(0, 0, delta_a);
        a += delta_a;
        
        
        cost = 0;

        for (PointPair& p : pairs) {
            p.a->Transform(delta_x, delta_y, 0);
            p.a->Transform(0, 0, delta_a);
            cost += p.a->DistanceSq(*p.b);
        }

        
        PointPair::MatchPairs(pairs, reference, delta_point.x, delta_point.y, a);
    }
    cost = sqrt(cost / (double)src.size());
    return cost;
}


std::vector<float> BestPosition(std::vector<std::vector<float>> positions, std::vector<Point> &map, std::vector<Point> &scan) {

    std::vector<float> best_position = positions[0];
    ICP initial_pos;
    float best_cost = initial_pos.PositionCost(scan, map, positions[0][0], positions[0][1], positions[0][2], false, 100);

    for (std::vector<float> pos : positions) {
        float cost = initial_pos.PositionCost(scan, map, pos[0], pos[1], pos[2], false, 100);
        std::cout << "Position = " << pos[0] << ", " << pos[1] << ", " << pos[2] << "\t cost = " << cost << "\n";
        if (cost < best_cost) {
            best_cost = cost;
            best_position = pos;
        }
    }

    return best_position;
}