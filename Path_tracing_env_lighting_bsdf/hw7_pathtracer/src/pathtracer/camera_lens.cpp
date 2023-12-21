#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {

  // TODO Assignment 7: Part 4
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  

  // TODO (Part 1.1):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  // transform image coordinates into camera space (obtain ray sample)
  Vector3D llCamera = Vector3D(-tan(0.5 * (hFov/180.0)), -tan(0.5 * (vFov/180.0)), -1);
  Vector3D urCamera = Vector3D(tan(0.5 * (hFov/180.0)), tan(0.5 * (vFov/180.0)), -1);
  
  Vector3D ptOnSensor = Vector3D((urCamera.x-llCamera.x)*x-tan(0.5 * (hFov/180.0)), (urCamera.y-llCamera.y)*y-tan(0.5 * (vFov/180.0)), -1);
  // Vector3D pFilm = -ptOnSensor;

  // generate the red ray in the camera space
  Ray cameraRay = Ray(Vector3D(0,0,0), ptOnSensor);

  // // transform the ray to the world space
  // Ray worldRay = Ray(pos + c2w * cameraRay.o, (c2w * cameraRay.d).unit());

  // // init the ray with clipping planes
  // worldRay.min_t = near_clip();
  // worldRay.max_t = far_clip();

  Vector3D pLens = Vector3D(lensRadius*sqrt(rndR)*cos(rndTheta), lensRadius*sqrt(rndR)*sin(rndTheta), 0);
  
  // compute red-ray and plane-focus intersaction
  double t = -focalDistance/cameraRay.d.z;
  Vector3D pFocus = cameraRay.o + t * cameraRay.d;

  // compute the blue ray
  Ray blueRay = Ray(pLens, (pFocus - pLens).unit());

  // convert the blue ray to the world space
  Ray blueRayWorld = Ray(pos + c2w * blueRay.o, (c2w * blueRay.d).unit());

  // init the ray with clipping planes
  blueRayWorld.min_t = near_clip();
  blueRayWorld.max_t = far_clip();

  return blueRayWorld;
  // return Ray(pos, Vector3D(0, 0, -1));
}


} // namespace CGL
