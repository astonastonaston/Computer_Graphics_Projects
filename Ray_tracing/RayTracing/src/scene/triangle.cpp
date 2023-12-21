#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  // Moller Trumbore Algorithm implementation for intersaction testing
  Vector3D e2=p2-p1, e3=p3-p1, s=r.o-p1;
  Vector3D s1=cross(r.d, e3), s2=cross(s, e2);
  Vector3D res = Vector3D(dot(s2,e3), dot(s1,s), dot(s2,r.d)) / dot(s1,e2);

  // Barycentric computation for judgement
  if ((res[1]>=0) && (res[2]>=0) && ((1-res[1]-res[2])>=0) && (res[0] >= r.min_t) && (res[0] <= r.max_t)) return true;
  else return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  // Moller Trumbore Algorithm implementation for intersaction testing
  Vector3D e2=p2-p1, e3=p3-p1, s=r.o-p1;
  Vector3D s1=cross(r.d, e3), s2=cross(s, e2);
  Vector3D res = Vector3D(dot(s2,e3), dot(s1,s), dot(s2,r.d)) / dot(s1,e2);
  // cout << "resulting bary: " << res << endl;
  // cout << "range: " << r.min_t << " and " << r.max_t << endl;
  // cout << ((res[1]>=0) && (res[2]>=0) && ((1-res[1]-res[2])>=0) && (res[0] >= r.min_t) && (res[0] <= r.max_t)) << endl;

  // Barycentric computation for judgement
  if ((res[1]>=0) && (res[1]<=1) && (res[2]>=0) && (res[2]<=1) && ((1-res[1]-res[2])>=0) && (res[0] >= r.min_t) && (res[0] <= r.max_t)) {
    // if intersection within the range, update data and return true
    if (res[0] < r.max_t) r.max_t = res[0];
    isect->t = res[0];
    isect->n = (1-res[1]-res[2])*n1 + res[1]*n2 + res[2]*n3;
    // cout << "normal vector: " << isect->n << endl; 
    // cout << "Compute: " << n1 << " " << n2 << " " << n3 << endl; 
    // cout << "Compute: " << isect->n << endl; 
    isect->primitive = this; // surface material type (object)
    isect->bsdf = get_bsdf(); // surface material properties
    return true;
  } 
  // else, return false
  else return false;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
