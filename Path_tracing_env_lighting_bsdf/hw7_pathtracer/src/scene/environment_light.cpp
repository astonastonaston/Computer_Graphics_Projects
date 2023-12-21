#include "environment_light.h"
#include "util/lodepng.h"

namespace CGL { namespace SceneObjects {

  EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
    init();
  }

  EnvironmentLight::~EnvironmentLight() {
    delete[] pdf_envmap;
    delete[] conds_y;
    delete[] marginal_y;
  }


  void EnvironmentLight::init() {
    uint32_t w = envMap->w, h = envMap->h;
    pdf_envmap = new double[w * h];
    conds_y = new double[w * h];
    marginal_y = new double[h];

    std::cout << "[PathTracer] Initializing environment light...";

    // TODO Assignment 7 Part 3 Task 3 Steps 1,2,3
    // Store the environment map pdf to pdf_envmap
    // Store the marginal distribution for y to marginal_y
    // Store the conditional distribution for x given y to conds_y

    double sum = 0;
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        pdf_envmap[w * j + i] = envMap->data[w * j + i].illum() * sin(PI * (j + .5) / h);
        sum += pdf_envmap[w * j + i];
      }
    }
    // normalize to make the pdf sum to 1
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        pdf_envmap[w * j + i] = pdf_envmap[w * j + i] / sum;
      }
    }

    // Compute the cdf of the marginal distribution w.r.t. the rows
    for (int j = 0; j < h; j++) {
      // jth row ith column, init with 0
      marginal_y[j] = 0;
      for (int i = 0; i < w; i++) marginal_y[j] += pdf_envmap[w * j + i];
      if (j!=0) marginal_y[j] += marginal_y[j-1];
    }

    // Update conditional distribution function
    double margi_pdf;
    for (int j = 0; j < h; j++) {
      // compute the marginal pdf
      if (j==0) margi_pdf = marginal_y[j];
      else margi_pdf = (marginal_y[j] - marginal_y[j-1]);

      // update the conditional probabilities
      for (int i = 0; i < w; i++) {
        // compute conditional cdf
        conds_y[j*w+i] = 0;
        for (int k = 0; k <= i; k++) conds_y[j*w+i] += ((pdf_envmap[w * j + k]) / (margi_pdf));
      }
    }


    if (true)
      std::cout << "Saving out probability_debug image for debug." << std::endl;
    save_probability_debug();

    std::cout << "done." << std::endl;
  }

  // Helper functions

  void EnvironmentLight::save_probability_debug() {
    uint32_t w = envMap->w, h = envMap->h;
    uint8_t* img = new uint8_t[4 * w * h];

    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        img[4 * (j * w + i) + 3] = 255;
        img[4 * (j * w + i) + 0] = 255 * marginal_y[j];
        img[4 * (j * w + i) + 1] = 255 * conds_y[j * w + i];
        img[4 * (j * w + i) + 2] = 0;
      }
    }

    lodepng::encode("probability_debug.png", img, w, h);
    delete[] img;
  }

  Vector2D EnvironmentLight::theta_phi_to_xy(const Vector2D& theta_phi) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = theta_phi.y / 2. / PI * w;
    double y = theta_phi.x / PI * h;
    return Vector2D(x, y);
  }

  Vector2D EnvironmentLight::xy_to_theta_phi(const Vector2D& xy) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = xy.x;
    double y = xy.y;
    double phi = x / w * 2.0 * PI;
    double theta = y / h * PI;
    return Vector2D(theta, phi);
  }

  Vector2D EnvironmentLight::dir_to_theta_phi(const Vector3D dir) const {
    Vector3D unit_dir = dir.unit();
    double theta = acos(unit_dir.y);
    double phi = atan2(-unit_dir.z, unit_dir.x) + PI;
    return Vector2D(theta, phi);
  }

  Vector3D EnvironmentLight::theta_phi_to_dir(const Vector2D& theta_phi) const {
    double theta = theta_phi.x;
    double phi = theta_phi.y;

    double y = cos(theta);
    double x = cos(phi - PI) * sin(theta);
    double z = -sin(phi - PI) * sin(theta);

    return Vector3D(x, y, z);
  }

  // Credits to Luowen Qian from Spring 2018 for this more robust bilerp
  Vector3D EnvironmentLight::bilerp(const Vector2D& xy) const {
    long right = lround(xy.x), left, v = lround(xy.y);
    double u1 = right - xy.x + .5, v1;
    if (right == 0 || right == envMap->w) {
      left = envMap->w - 1;
      right = 0;
    }
    else left = right - 1;
    if (v == 0) v1 = v = 1; else if (v == envMap->h) {
      v = envMap->h - 1;
      v1 = 0;
    }
    else v1 = v - xy.y + .5;
    auto bottom = envMap->w * v, top = bottom - envMap->w;
    auto u0 = 1 - u1;
    return (envMap->data[top + left] * u1 + envMap->data[top + right] * u0) * v1 +
      (envMap->data[bottom + left] * u1 + envMap->data[bottom + right] * u0) * (1 - v1);
  }


  Vector3D EnvironmentLight::sample_L(const Vector3D p, Vector3D* wi,
    double* distToLight,
    double* pdf) const {
    // TODO: Assignment 7 Part 3 Tasks 2 and 3 (step 4)
    // First implement uniform sphere sampling for the environment light
    // Later implement full importance sampling

    // Importance sampling
    Vector2D unisamp = sampler_uniform2d.get_sample();
    int w = envMap->w, h = envMap->h;
    // sampling by exceeding judgement
    int samp_y = std::upper_bound(marginal_y, marginal_y+h, unisamp.y) - marginal_y;
    int samp_x = std::upper_bound(conds_y+samp_y*w, conds_y+(samp_y+1)*w, unisamp.x) - (conds_y+samp_y*w);
    Vector2D samp_xy = Vector2D(samp_x, samp_y);

    // updating wi and pdf
    Vector2D theta_phi = xy_to_theta_phi(samp_xy);
    *wi = theta_phi_to_dir(theta_phi);
    *distToLight = INF_D;
    double pdfCoeff = w * h / (2.0 * PI * PI * sin(theta_phi.x)); 
    *pdf = pdf_envmap[samp_y * w + samp_x] * pdfCoeff;
    return bilerp(samp_xy);

    // // // Uniform sampling
    // *wi = sampler_uniform_sphere.get_sample();
    // *distToLight = INF_D;
    // *pdf = 1.0 / (4.0 * PI);
    
    // // // cvt dir to theta and phi
    // Vector2D theta_phi = dir_to_theta_phi(*wi);
    // // // bilinear interpolation
    // return bilerp(theta_phi_to_xy(theta_phi));
  }

  Vector3D EnvironmentLight::sample_dir(const Ray& r) const {
    // TODO: Assignment 7 Part 3 Task 1
    // Use the helper functions to convert r.d into (x,y)
    // then bilerp the return value
    Vector2D theta_phi = dir_to_theta_phi(r.d);
    Vector2D xy = theta_phi_to_xy(theta_phi);
    Vector3D bilrp = bilerp(xy);

    // envMap->update_pixel(bilrp, (size_t)(xy.x), (size_t)(xy.y));
    // envMap->update_pixel(bilrp, 1, 2);

    return bilrp;

  }

} // namespace SceneObjects
} // namespace CGL
