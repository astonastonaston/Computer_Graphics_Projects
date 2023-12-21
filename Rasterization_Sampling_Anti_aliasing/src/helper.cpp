#include "helper.h"
#include<vector>


float threeMin(float x, float y, float z)
{
    if (x < y) {
        if (x < z) return x;
        else return z;
    }
    else {
        if (y < z) return y;
        else return z;
    }
}

float threeMax(float x, float y, float z)
{
    if (x > y) {
        if (x > z) return x;
        else return z;
    }
    else {
        if (y > z) return y;
        else return z;
    }
}

// tri: {xa, ya, xb, yb, xc, yc}
// pt: {xp, yp}
// pt_bary: {alphaA, betaB, lamdaC}
void baryCompute(vector<float>& pt, vector<float>& tri, vector<float>& pt_bary)
{

  float xp=pt[0], yp=pt[1], xa=tri[0], ya=tri[1], xb=tri[2], yb=tri[3], xc=tri[4], yc=tri[5];
  pt_bary[0] = (-(xp-xb)*(yc-yb) + (yp-yb)*(xc-xb))/(-(xa-xb)*(yc-yb) + (ya-yb)*(xc-xb));
  pt_bary[1] = (-(xp-xc)*(ya-yc) + (yp-yc)*(xa-xc))/(-(xb-xc)*(ya-yc) + (yb-yc)*(xa-xc));
  pt_bary[2] = 1 - pt_bary[0] - pt_bary[1];
}

// tri: {xa, ya, xb, yb, xc, yc}
// pt: {xp, yp}
// pt_bary: {alphaA, betaB, lamdaC}
void baryCompute_cgl(CGL::Vector2D& pt, CGL::Vector2D& a, CGL::Vector2D& b, CGL::Vector2D& c, CGL::Vector3D& pt_bary)
{
  double xp=pt[0], yp=pt[1], xa=a.x, ya=a.y, xb=b.x, yb=b.y, xc=c.x, yc=c.y;
  pt_bary[0] = (-(xp-xb)*(yc-yb) + (yp-yb)*(xc-xb))/(-(xa-xb)*(yc-yb) + (ya-yb)*(xc-xb));
  pt_bary[1] = (-(xp-xc)*(ya-yc) + (yp-yc)*(xa-xc))/(-(xb-xc)*(ya-yc) + (yb-yc)*(xa-xc));
  pt_bary[2] = 1 - pt_bary[0] - pt_bary[1];
}

CGL::Color lerp(float x, CGL::Color v0, CGL::Color v1) {
    CGL::Color rescol;
    // no subtraction for CGL::Color =_=
    rescol[0] = v0[0]+x*(v1[0]-v0[0]); 
    rescol[1] = v0[1]+x*(v1[1]-v0[1]); 
    rescol[2] = v0[2]+x*(v1[2]-v0[2]); 
    return (rescol);    
}

