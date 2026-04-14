#pragma once
#include <vector>
#include "Config.h"
#include <CL/cl.h>
#include "Point.h"
#include "line.h"
#include <opencv2/core.hpp>

class QuadTree {
public:
	struct Quad {
		uint32_t id = 0;
		uint32_t children = 0;

	};

	float x;
	float y;
	float w;
	std::vector<Quad> tree;
	QuadTree(float x, float y, float w);
	void Clear();
	bool IsQuadIntersecting(Quad& quad, Point* test);
	void GenerateTree(std::vector<Point*>& points, int depth);
	void GenerateTreeRecursive(std::vector<Point*>& points, int index);
	Point* NavigateTree(Point* point);
};

class QuadTreeOpenCL {
public:
	QuadTree* quad_tree;

	size_t global;                      // global domain size for our calculation
	size_t local;                       // local domain size for our calculation

	cl_device_id device_id;             // compute device id 
	cl_context context;                 // compute context
	cl_command_queue commands;          // compute command queue
	cl_program program;                 // compute program
	cl_kernel kernel;                   // compute kernel

	cl_mem tree;                       // device memory used for the input array
	cl_mem points_ref;                      // device memory used for the output array
	cl_mem consts;                       // device memory used for the input array
	cl_mem results;                      // device memory used for the output array
	Point* results_data;

	bool compiled = false;
	bool points_loaded = false;
	bool quad_tree_loaded = false;
	QuadTreeOpenCL(float x, float y, float w);
	~QuadTreeOpenCL();
	void Clear();
    void ReuseQuadClear();
	void CompileKernel();
	void Initilize(std::vector<Point*>& points, int depth, int src_point_size);
	Point* NavigateTree(std::vector<Point>* point, size_t size, float dx, float dy, float da);
};

class PointPair {
public:
	Point* a;
	Point* b;
	PointPair();
	PointPair(Point* a, Point* b);
	
	
	static void MatchPoints(std::vector<Point>& src, std::vector<Point>& reference, std::vector<PointPair>& dst, bool reuse_quad_tree);
	static void MatchPairs(std::vector<PointPair>& src, std::vector<Point>& reference, float dx, float dy, float da);

public:
	static QuadTreeOpenCL* quad_tree_opencl;
    static std::vector<Line> lines;
};

class ICP {
public:
	ICP();

	void Iterate(std::vector<PointPair>& point_pairs, float& dx, float& dy, float& a);
	int Compute(std::vector<Point>& src, std::vector<Point>& reference, float& dx, float& dy, float& a, double& cost, bool reuse_quad_tree);
	float PositionCost(std::vector<Point> src, std::vector<Point>& reference, float dx, float dy, float a, bool reuse_quad_tree, int icp_iterations);


	void MergeMaps(std::vector<Point>& map, std::vector<std::vector<Point>>& read, int lookback, cv::Mat& allowed);

};

std::vector<float> BestPosition(std::vector<std::vector<float>> positions, std::vector<Point> &map, std::vector<Point> &scan);