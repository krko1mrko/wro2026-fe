#pragma once

const char* KernelSource = "\n" \
"__kernel void quad_tree(                                                       \n" \
"   __global uint* tree,                                              \n" \
"   __global float* consts,                                             \n" \
"   __global float* points,                                             \n" \
"   __global float* output)                                             \n" \
"{                                                                      \n" \
"uint i = 0;																													\n" \
"int cord = get_global_id(0);																										\n" \
"float point_min_d = 10000;																											\n" \
"float point_x = points[cord * 2];																									\n" \
"float point_y = points[cord * 2 + 1];																								\n" \
/*
"float2 point2 = {point1.x * cos(consts[5]) - point1.y * sin(consts[5]) + consts[3], point1.x * sin(consts[5]) + point1.y * cos(consts[5]) + consts[4]}; \n" \
"float2 point = {point2.x * cos(-0.785398) - point2.y * sin(-0.785398), point2.x * sin(-0.785398) + point2.y * cos(-0.785398)}; \n" \
*/
"float2 point1 = (float2)(point_x * cos(consts[5]) - point_y * sin(consts[5]) + consts[3], point_x * sin(consts[5]) + point_y * cos(consts[5]) + consts[4]); \n" \
"float2 point = (float2)(point1.x * cos(0.785398) - point1.y * sin(0.785398), point1.x * sin(0.785398) + point1.y * cos(0.785398)); \n" \
"float2 q_const = (float2)(consts[0], consts[1]); \n" \
"while (true) {																														\n" \
"    uint children = tree[i * 2 + 1];																							\n" \
"    int next_start = ((children >> 4) & 0x0FFFFFFF);																				\n" \
"    float min_distance = 10000;																									\n" \
"    int min_j = 0;																													\n" \
"    int depth = (tree[next_start * 2] >> 28) & 0xF;																				\n" \
"    float wq = consts[2] / (float)(1 << depth);																					\n" \
"	 uint count = (children & 0xF);                                                                                                 \n" \
"    for (int j = 0; j < count; j++) {																					\n" \
"        uint id = tree[(next_start + j) * 2];																					\n" \
"        float2 q = (float2)(consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14), consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14)); \n" \
/*
"        float xq = consts[0];																										\n" \
"        float yq = consts[1];																										\n" \
*/
/*
"        xq += consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14);																				\n" \
"        yq += consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);																\n" \

"        q.x += (float)(id & 0x3FFF) / (float)(1 << 12); \n" \
"        q.y += (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 12); \n" \
*/
"        q += q_const + wq / 2; \n" \


"        float dis = fast_distance(q, point); \n" \
/*
"        float dis = (point.x - q.x - wq / 2) * (point.x - q.x - wq / 2) + (point.y - q.y - wq / 2) * (point.y - q.y - wq / 2);			\n" \
*/
"        if (dis < min_distance) {																									\n" \
"            min_distance = dis;																									\n" \
"            min_j = j;																												\n" \
"        }																															\n" \
"    }																																\n" \
"    i = next_start + min_j;																										\n" \
"    if ((tree[i * 2 + 1] & 0xF) == 0xF) {																							\n" \
"        float xq = consts[0];																										\n" \
"        float yq = consts[1];																										\n" \
"        uint id = tree[(next_start + min_j) * 2];																				\n" \
"        xq += consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14);																				\n" \
"        yq += consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);																\n" \
"        point_min_d = min_distance;																								\n" \
"        output[cord * 2] = xq + wq / 2;																							\n" \
"        output[cord * 2 + 1] = yq + wq / 2;																						\n" \
"        break;																														\n" \
"    }																																\n" \
"}																																	\n" \

"}                                                                      \n" \
"\n";



const char* KernelSource2 = "\n" \
"__kernel void quad_tree(                                                       \n" \
"   __global uint* tree,                                              \n" \
"   __global float* consts,                                             \n" \
"   __global float* points,                                             \n" \
"   __global float* output)                                             \n" \
"{                                                                      \n" \
"uint i = 0;																													\n" \
"int cord = get_global_id(0);																										\n" \
"float point_min_d = 10000;																											\n" \
"float point_x = points[cord * 2];																									\n" \
"float point_y = points[cord * 2 + 1];																								\n" \
"float2 point = {point_x * cos(consts[5]) - point_y * sin(consts[5]) + consts[3], point_x * sin(consts[5]) + point_y * cos(consts[5]) + consts[4]}; \n" \
"float2 q_const = {consts[0], consts[1]}; \n" \
"while (true) {																														\n" \
"    uint children = tree[i * 2 + 1];																							\n" \
"    int next_start = ((children >> 4) & 0x00003FFF);																				\n" \
"    float min_distance = 10000;																									\n" \
"    int min_j = 0;																													\n" \
"    int depth = (tree[next_start * 2] >> 28) & 0xF;																				\n" \
"    float wq = consts[2] / (float)(1 << depth);																					\n" \
"	 uint count = (children & 0xF);                                                                                                 \n" \
"    for (int j = 0; j < count; j++) {																					\n" \
"        uint id = tree[(next_start + j) * 2];																					\n" \

"        float2 q = {consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14), consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14)}; \n" \
/*
"        float xq = consts[0];																										\n" \
"        float yq = consts[1];																										\n" \
*/
/*
"        xq += consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14);																				\n" \
"        yq += consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);																\n" \

"        q.x += (float)(id & 0x3FFF) / (float)(1 << 12); \n" \
"        q.y += (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 12); \n" \
*/
"        q += q_const + wq / 2; \n" \


"        float dis = fast_distance(q, point); \n" \
/*
"        float dis = (point.x - q.x - wq / 2) * (point.x - q.x - wq / 2) + (point.y - q.y - wq / 2) * (point.y - q.y - wq / 2);			\n" \
*/
"        if (dis < min_distance) {																									\n" \
"            min_distance = dis;																									\n" \
"            min_j = j;																												\n" \
"        }																															\n" \
"    }																																\n" \
"    i = next_start + min_j;																										\n" \
"    if ((tree[i * 2 + 1] & 0xF) == 0xF) {																							\n" \
"        float xq = consts[0];																										\n" \
"        float yq = consts[1];																										\n" \
"        uint id = tree[(next_start + min_j) * 2];																				\n" \
"        xq += consts[2] * (float)(id & 0x3FFF) / (float)(1 << 14);																				\n" \
"        yq += consts[2] * (float)((id & (0x3FFF << 14)) >> 14) / (float)(1 << 14);																\n" \
"        point_min_d = min_distance;																								\n" \
"        output[cord * 2] = xq + wq / 2;																							\n" \
"        output[cord * 2 + 1] = yq + wq / 2;																						\n" \
"        break;																														\n" \
"    }																																\n" \
"}																																	\n" \

"}                                                                      \n" \
"\n";
