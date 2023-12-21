#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // hit point: the intersaction point
  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  // sample in unit sphere, and do Mon-te Carlo integration for estimating incoming illuminance
  // use reflaction equation to estimate the outgoing light

  Intersection isect_src;
  double prob = 1.0 / (2.0 * PI);
  Vector3D wi_hemi, wi_hemi_world;

  for (int i = 0; i < num_samples; i++) {
    // sample the solid angle, and do the world-space transformation
    wi_hemi = hemisphereSampler->get_sample();

    // transform into the world space
    wi_hemi_world = o2w * wi_hemi;

    // reconstruct the incoming ray (and hoping that it's the shadow ray)
    Ray shadow = Ray(hit_p, wi_hemi_world);
    shadow.min_t = EPS_F; // to avoid on-field intersaction

    // only add the incoming lights that hit the light source
    Vector3D numerator_est(0,0,0);
    if (bvh->intersect(shadow, &isect_src)) {
      // get src emission illuminance
      Vector3D src_emit = isect_src.bsdf->get_emission();
      numerator_est = (isect.bsdf->f(w_out, wi_hemi) * cos_theta(wi_hemi.unit()) * src_emit) / prob;
    }
    else continue;
    L_out += numerator_est / num_samples;
  }

  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  Intersection inter;

  for (SceneLight *s : scene->lights) {
    int num_samples;
    Vector3D L_no_occlu;

    if (s->is_delta_light()) {
      // point light src -> take only one point sample
      num_samples = 1;
    }

    else {
      // area light src -> sample multiple points (light srcs) from the area
      num_samples = ns_area_light;
    }

    for (int i = 0; i < num_samples; i++) {
      // incoming sampled direction
      Vector3D wi;
      double distToLight;
      double prob;

      // sample the given light src's info, given the illuminated point 
      Vector3D src_emit_radiance = s->sample_L(hit_p, &wi, &distToLight, &prob);

      // ???
      Vector3D w_world = w2o * wi;
      if (w_world.z < 0) continue;
      
      // set the shadow ray for occlusion testing
      Ray shadow = Ray(hit_p, wi);
      shadow.min_t = EPS_F;
      shadow.max_t = distToLight - EPS_F;

      if (!bvh->intersect(shadow, &inter)) {
        // no occlusion: update the illuminance with Monte-Carlo integration
        L_no_occlu += isect.bsdf->f(w_out, w_world) * cos_theta(w_world) * src_emit_radiance / prob;
      }
    }
    L_out += L_no_occlu / num_samples;
  }
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
  // return Vector3D(1.0);
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  else {
    return estimate_direct_lighting_importance(r, isect);
  }

  // return estimate_direct_lighting_hemisphere(r, isect);
  // return estimate_direct_lighting_importance(r, isect);
  // return Vector3D(1.0);
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  double prob;
  float rr_p = 0.6;
  bool rr_not_stop = coin_flip(rr_p);
  Vector3D w_in;
  Vector3D reflectance = isect.bsdf->sample_f(w_out, &w_in, &prob);

  // init with current 1-bounce radiance
  L_out = one_bounce_radiance(r, isect);

  // add with more-bounce radiance in recursion (by tracing)
  if (rr_not_stop || r.depth < max_ray_depth) {
    // transfer the incoming ray to the world space
    Vector3D w_in_world = o2w * w_in;
    w_in_world.normalize();

    // generate the shadow ray for occlusion test (object-src hitting test)
    Ray shadow = Ray(hit_p, w_in_world);
    shadow.depth = r.depth + 1;
    shadow.min_t = EPS_F;

    // object-hitting case: estimate more-bounced radiances in recursion
    Intersection inter;
    if (bvh->intersect(shadow, &inter)) {
      if (r.depth < max_ray_depth) L_out += at_least_one_bounce_radiance(shadow, inter) * reflectance * cos_theta(w_in) / prob;
      else L_out += at_least_one_bounce_radiance(shadow, inter) * reflectance * cos_theta(w_in) / (prob * rr_p);
    }
  }

  // No more intersactions with object, or stopping by rr and the max depth: Just return with one-bounce rediance
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  bool inter = bvh->intersect(r, &isect);
  // std::cout << "the t is " << isect.t << std::endl;
  if (!inter)
    return envLight ? envLight->sample_dir(r) : L_out;

  // std::cout << isect.n;
  // std::cout << r.d;
  L_out = (isect.t == INF_D) ? debug_shading(r.d) : (zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect));
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : (at_least_one_bounce_radiance(r, isect) - one_bounce_radiance(r_ori, isect_ori));
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : (zero_bounce_radiance(r, isect)+one_bounce_radiance(r, isect));
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : zero_bounce_radiance(r, isect);
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.
  // a bit shallower for banana.dae?????

  // UniformGridSampler2D sampler;
  Vector2D sample_point(0,0);
  Ray sample_ray;
  Vector3D res(0,0,0), est_rad(0,0,0);
  double sum=0, sqrSum=0, miu=0, sigma=0, cvg_var=0;
  for (int i = 0; i < num_samples; i++) {
    sample_point = gridSampler->get_sample() + origin;

    // generate ray for the pixel point
    sample_ray = camera->generate_ray((sample_point.x / sampleBuffer.w), (sample_point.y / sampleBuffer.h));
    sample_ray.depth = 0;
    est_rad = est_radiance_global_illumination(sample_ray);
    res += est_rad;

    // update sum and sqr sum
    sum += est_rad.illum();
    sqrSum += est_rad.illum() * est_rad.illum();

    // detect termination 
    if (i%samplesPerBatch == 31) {
      miu = sum / (i+1);
      sigma = sqrt((1.0/(i)) * (sqrSum - (sum * sum / (i+1))));
      cvg_var = 1.96 * (sigma / sqrt(i+1));

      // convergence reached: termination
      if (cvg_var <= maxTolerance * miu) {
        num_samples = i+1;
        break;
      }
    }
  }
  res = res / num_samples;
  // cout << "rt pixel res " << res << endl;

  sampleBuffer.update_pixel(res, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
