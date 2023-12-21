#ifndef HELPER_H
#define HELPER_H
#include "CGL/CGL.h"
#include "CGL/color.h"
#include "CGL/vector3D.h"
#include <vector>
#include "svg.h"

float threeMin(float x, float y, float z);
float threeMax(float x, float y, float z);



CGL::Color lerp(float x, CGL::Color v0, CGL::Color v1);

void baryCompute(vector<float>& pt, vector<float>& tri, vector<float>& pt_bary);

void baryCompute_cgl(CGL::Vector2D& pt, CGL::Vector2D& a, CGL::Vector2D& b, CGL::Vector2D& c, CGL::Vector3D& pt_bary);





#endif