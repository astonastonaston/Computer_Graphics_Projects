#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double med;
  if (t1>t2) {
    med = t2;
    t2 = t1;
    t1 = med;
  }
  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  Vector3D quadParam(dot(r.d,r.d), 2*dot((r.d),r.o-o), dot(r.o-o, r.o-o)-r2); // corresponding to a, b, c
  double t1, t2, delta=(quadParam[1]*quadParam[1] - 4*quadParam[0]*quadParam[2]);
  if (delta >= 0) {
    return true;
  }
  else return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {
  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  Vector3D quadParam(dot(r.d,r.d), 2*dot((r.d),r.o-o), dot(r.o-o, r.o-o)-r2); // corresponding to a, b, c
  double t1, t2, tfinal, delta=(quadParam[1]*quadParam[1] - 4*quadParam[0]*quadParam[2]);
  if (delta >= 0) {
    // std::cout << "cir! \n";
    t1=(-quadParam[1] + sqrt(quadParam[1]*quadParam[1] - 4*quadParam[0]*quadParam[2])) / (2*quadParam[0]);
    t2=(-quadParam[1] - sqrt(quadParam[1]*quadParam[1] - 4*quadParam[0]*quadParam[2])) / (2*quadParam[0]);
    test(r, t1, t2); // getting the smaller intersaction

    // in case only two ts lies outside the range
    if ((t2 < r.min_t) or (t1 > r.max_t) or ( (t1<r.min_t)&&(t2>r.max_t) )) return false;

    // in case only one t lies in the range
    if (t1 < r.min_t) tfinal = t2;
    else tfinal = t1;

    // update params
    r.max_t = tfinal;
    i->t = tfinal;
    i->n = normal(r.o+tfinal*r.d);
    i->primitive = this; // surface material type (object)
    i->bsdf = get_bsdf(); // surface material properties
    return true;
  }
  else return false;

}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
