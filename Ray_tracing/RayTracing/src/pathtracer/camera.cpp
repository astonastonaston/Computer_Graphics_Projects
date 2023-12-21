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

/**
 * Sets the field of view to match screen screenW/H.
 * NOTE: data and screenW/H will almost certainly disagree about the aspect
 *       ratio. screenW/H are treated as the source of truth, and the field
 *       of view is expanded along whichever dimension is too narrow.
 * NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
 */
void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;

  // clipping planes for the camera
  nClip = info.nClip;
  fClip = info.fClip;
  
  // horizontal and vertical FoVs
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);

  // aspect ratio
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }

  // d_camera->screen
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
}

/**
 * This function places the camera at the target position and sets the arguments.
 * Phi and theta are in RADIANS.
 */
void Camera::place(const Vector3D targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  // screen-space target position
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

/**
 * This function copies the camera placement state.
 */
void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  // min max R ???
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

/**
 * This sets the screen size & compute the new FOV.
 */
void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  // reset aspect ratio, h and v FoV
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

/**
 * This function translates the camera position
 * Moving along world-space coordinate basises (x,y)
 * What's d ????
 */
void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);

  // along the world basis axis and along the camera basis axis, the d
  // What Is TargetPos????????
  pos += displacement;
  targetPos += displacement;
}

/**
 * This function translates the camera position (in forward direction w.r.t. polar world coordinates)
 * dist: the scala for moving forward
 */
void Camera::move_forward(const double dist) {
  // edge moving consideration
  double newR = min(max(r - dist, minR), maxR);
  // re-translate from camera-coordinate camera position to world-coordinate camera position
  // pos-targetPos: dirToCamera in the world space
  // here, scaling along the dirToCamera direction
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

/**
 * This function rotates the camera position
 * rotate the camera in world coordinate by polar coordinates
 */
void Camera::rotate_by(const double dPhi, const double dTheta) {
  // compute new polar coordinates
  theta += dTheta;
  phi = clamp(phi + dPhi, 0.0, (double) PI);

  // update world's cartesian coor and transformation matrix
  compute_position();
}

/**
 * This function computes the camera position, basis vectors, and the view matrix
 */
void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));

  // reset cartesian coordinates ???
  pos = targetPos + dirToCamera;

  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  // reset transformation matrix(from camera coordinate to the world coordinate)
  // c2w_matrix * cam_coor = I * world_coor
  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

/**
 * This function stores the camera settings into a file
 */
void Camera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;
  file << focalDistance << " " << lensRadius << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

/**
 * This function loads the camera settings from a file
 */
void Camera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;
  file >> focalDistance >> lensRadius;
  cout << "[Camera] Loaded settings from " << filename << endl;
}

/**
 * This function generates a ray from camera perspective, passing through camera / sensor plane (x,y)
 */
Ray Camera::generate_ray(double x, double y) const {

  // TODO (Part 1.1):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  // transform image coordinates into camera space (obtain ray sample)
  Vector3D llCamera = Vector3D(-tan(0.5 * (hFov/180.0)), -tan(0.5 * (vFov/180.0)), -1);
  Vector3D urCamera = Vector3D(tan(0.5 * (hFov/180.0)), tan(0.5 * (vFov/180.0)), -1);
  
  Vector3D ptOnSensor = Vector3D((urCamera.x-llCamera.x)*x-tan(0.5 * (hFov/180.0)), (urCamera.y-llCamera.y)*y-tan(0.5 * (vFov/180.0)), -1);
  
  // generate rays in camera space
  Ray cameraRay = Ray(Vector3D(0,0,0), ptOnSensor);

  // transform the ray to the world space
  Ray worldRay = Ray(pos + c2w * cameraRay.o, (c2w * cameraRay.d).unit());

  // init the ray with clipping planes
  worldRay.min_t = near_clip();
  worldRay.max_t = far_clip();
  return worldRay;
}

} // namespace CGL
