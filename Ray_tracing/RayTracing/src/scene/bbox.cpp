#include "bbox.h"

#include "GL/glew.h"
#include "CGL/vector2D.h"

#include <algorithm>
#include <iostream>
#include <vector>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  // get 6 axis-aligned intersaction points
  Vector2D params((1.0/r.d.x), -(r.o.x/r.d.x));
  double txmin=dot(params, Vector2D(min.x, 1)), txmax=dot(params, Vector2D(max.x, 1)); // intersaction with x planes
  if (txmin > txmax) std::swap(txmin,txmax);

  params.x = (1.0/r.d.y);
  params.y = -(r.o.y/r.d.y);
  double tymin=dot(params, Vector2D(min.y, 1)), tymax=dot(params, Vector2D(max.y, 1)); // intersaction with y planes
  if (tymin > tymax) std::swap(tymin,tymax);
  
  params.x = (1.0/r.d.z);
  params.y = -(r.o.z/r.d.z);
  double tzmin=dot(params, Vector2D(min.z, 1)), tzmax=dot(params, Vector2D(max.z, 1)); // intersaction with z planes
  if (tzmin > tzmax) std::swap(tzmin,tzmax);

  txmin = std::max(std::max(txmin, tymin), tzmin);
  txmax = std::min(std::min(txmax, tymax), tzmax);
  t0 = txmin;
  t1 = txmax;
  if((txmin <= txmax) && (txmax >= 0))
    return true;
  else
    return false;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
