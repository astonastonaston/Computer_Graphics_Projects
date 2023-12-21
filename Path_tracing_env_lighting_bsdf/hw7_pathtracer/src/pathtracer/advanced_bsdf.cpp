#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Implement MirrorBSDF
  return Vector3D();
}

void MirrorBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Mirror BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

// // Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D h) {
  // TODO Assignment 7: Part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  Vector3D n(0,0,1);
  double cosThetah=cos_theta(h), cosThetahSqr = cosThetah*cosThetah, sinThetahSqr=sin_theta(h)*sin_theta(h);
  // double alpha=0.005, cosThetah=cos_theta(h), cosThetahSqr = cosThetah*cosThetah, sinThetahSqr=sin_theta(h);
  
  return exp(-((sinThetahSqr/cosThetahSqr)/(alpha*alpha))) / (PI*alpha*alpha*cosThetahSqr*cosThetahSqr);
}

Vector3D MicrofacetBSDF::F(const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Vector3D.
  
  double costhei=cos_theta(wi);
  Vector3D Rs, Rp, eta_k_sqr_sum = eta * eta + k * k;
  // update with indices
  Rs = (eta_k_sqr_sum - 2 * eta * costhei + (costhei * costhei)) / (eta_k_sqr_sum + 2 * eta * costhei + costhei * costhei);
  Rp = (eta_k_sqr_sum * costhei * costhei - 2 * eta * costhei + 1) / (eta_k_sqr_sum * costhei * costhei + 2 * eta * costhei + 1);
  return (Rs+Rp)/2;
}

Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO Assignment 7: Part 2
  // Implement microfacet model here.
  Vector3D wsum = (wo+wi), n(0,0,1);
  if ((dot(wo, n)>0) && (dot(wi, n)>0)) {
    wsum.normalize();
    return F(wi)*G(wo, wi)*D(wsum) / (4.0*dot(n,wo)*dot(n,wi));
  }
  else {
    return Vector3D(0,0,0);
  }
}

Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  // // cosine hemi sampling
  // *wi = cosineHemisphereSampler.get_sample(pdf);
  // return MicrofacetBSDF::f(wo, *wi);



  // sample theta and phai
  // double alpha=0.005;
  Vector2D rand = sampler.get_sample();
  double r1=rand[0], r2=rand[1];
  double theta=atan(sqrt(- alpha * alpha * log(1.0-r1))), phai=2*PI*r2;
  // for simple calculation
  double tantheta=sqrt(- alpha * alpha * log(1-r1)), costhetaSqr=(1/(1+tantheta*tantheta));

  // reconstruct h
  Vector3D h(1*sin(theta)*cos(phai), 1*sin(theta)*sin(phai), 1*cos(theta));
  // reconstruct wi and update
  *wi = -wo + 2.0 * dot(wo, h) * h;

  // reconstruct the pdf of wi and update
  double pdftheta, pdfphai, pdfh, pdfwi;
  pdftheta = (2.0 * tantheta / (alpha*alpha * costhetaSqr)) * (exp(- (tan(theta) * tan(theta)) / (alpha * alpha)));
  pdfphai = 1.0/(2.0*PI);
  pdfh = (pdftheta*pdfphai)/(sin(theta));
  pdfwi = pdfh/(4.0*dot(*(wi), h));
  *pdf = pdfwi;
  return MicrofacetBSDF::f(wo, *wi);
}

void MicrofacetBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Micofacet BSDF"))
  {
    DragDouble3("eta", &eta[0], 0.005);
    DragDouble3("K", &k[0], 0.005);
    DragDouble("alpha", &alpha, 0.005);
    ImGui::TreePop();
  }
}

// Refraction BSDF //

Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  // TODO Assignment 7: Part 1
  // Implement RefractionBSDF
  return Vector3D();
}

void RefractionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

// Glass BSDF //

Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

  // TODO Assignment 7: Part 1
  // Compute Fresnel coefficient and either reflect or refract based on it.

  // compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
  return Vector3D();
}

void GlassBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Refraction BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    DragDouble3("Transmittance", &transmittance[0], 0.005);
    DragDouble("ior", &ior, 0.005);
    ImGui::TreePop();
  }
}

void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

  // TODO Assignment 7: Part 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.


}

bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

  // TODO Assignment 7: Part 1
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  return true;

}
} // namespace CGL





















































