#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

// take smaller centroid x coordinate for sorting primitives
bool mycmpx(Primitive * i, Primitive * j) {return ((i->get_bbox().centroid().x)<(j->get_bbox().centroid().x));}
bool mycmpy(Primitive * i, Primitive * j) {return ((i->get_bbox().centroid().y)<(j->get_bbox().centroid().y));}
bool mycmpz(Primitive * i, Primitive * j) {return ((i->get_bbox().centroid().z)<(j->get_bbox().centroid().z));}

// void BVHAccel::qsort(const std::vector<Primitive *> &_primitives) {

// }

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  std::sort(primitives.begin(), primitives.end(), mycmpx); 
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

// void BVHAccel::__construct_bvh(std::vector<Primitive *>::iterator start, std::vector<Primitive *>::iterator end, size_t max_leaf_size, BVHNode* parent) {

// }


BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  // BBox bbox;

  // // construct first-level bbox
  // for (auto p = start; p != end; p++) {
  //   BBox bb = (*p)->get_bbox();
  //   bbox.expand(bb);
  // }

  // BVHNode *node = new BVHNode(bbox);
  // node->start = start;
  // node->end = end;
  // return node;

  // to determine the dimension to split (x, y, z in sequence)
  switch (num_of_recur)
  {
  case 0: {
    std::sort(start, end, mycmpx);
    break;
  }
  case 1: {
    std::sort(start, end, mycmpy);
    break;
  }
  case 2: {
    std::sort(start, end, mycmpz);
    break;
  }
  default:
    break;
  }
  num_of_recur = (num_of_recur+1)%3;

  BBox bbox;

  // construct first-level bbox
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  if ((end - start)<=max_leaf_size) {
    // reaching leaf
    node->l = NULL; 
    node->r = NULL; 
    return node;
  }
  else {
    node->l = construct_bvh(start, start + ((end-start)/2), max_leaf_size);
    node->r = construct_bvh(start + ((end-start)/2), end, max_leaf_size);
    return node;
  }
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  // original:
  // for (auto p : primitives) {
  //   total_isects++;
  //   if (p->has_intersection(ray))
  //     return true;
  // }
  // return false;

  // for (auto p=node->start; p < node->end; p++) {
  //   if ((*p)->has_intersection(ray)) return true;
  // }
  // return false;

  double t0, t1;
  if (!(node->bb.intersect(ray, t0, t1))) {
    // greedy: no intersaction
    return false;
  }
  else if ((node->l == NULL) && (node->r == NULL)) {
    // leaf-hitting intersaction detection
    for (auto p=node->start; p < node->end; p++) {
      total_isects++;
      // hit = p->intersect(ray, i);
      if ((*p)->has_intersection(ray)) return true;
    }
    return false;
  }
  else {
    bool left = false || has_intersection(ray, node->l);
    bool right = false || has_intersection(ray, node->r);
    return  left || right ;
  }
}

// bool BVHAccel::intersect_bbox(const Ray& r, BVHNode *node) const {
// }


bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // bool hit = false;
  // for (auto p : primitives) {
  //   total_isects++;
  //   // hit = p->intersect(ray, i);
  //   hit = p->intersect(ray, i) || hit;
  // }
  // return hit;
  
  double t0, t1;
  if (!(node->bb.intersect(ray, t0, t1))) {
    // greedy: no intersaction
    return false;
  }
  else if ((t1<ray.min_t)||(t0>ray.max_t)) {
    // intersaction not in range
    return false;
  }
  else if (node->isLeaf()) {
    // leaf-hitting intersaction detection
    bool hit = false;
    for (auto p=node->start; p < node->end; p++) {
      total_isects++;
      // hit = p->intersect(ray, i);
      // update hitting information by invoking the primitive-hitting method
      bool intr = (*p)->intersect(ray, i);
      if (intr) hit = true;
    }
    return hit;
  }
  else {
    // to avoid logical short-circuit
    bool left_inter = false;
    bool right_inter = false;
    left_inter = left_inter || intersect(ray, i, node->l);
    right_inter = right_inter || intersect(ray, i, node->r);
    return left_inter || right_inter;
    // return (intersect(ray, i, node->l) || intersect(ray, i, node->r));
  }
}

} // namespace SceneObjects
} // namespace CGL
