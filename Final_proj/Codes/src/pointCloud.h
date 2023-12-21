//
//  pointCloud.h
//  meshedit
//
//  Created by Benjamin Pust on 4/30/19.
//

#ifndef pointCloud_h
#define pointCloud_h

#include "scene.h"
#include "material.h"

namespace CGL {
  struct PointCloud : Instance {
    std::string id;
    std::string name;
    std::vector<Vector3D> vertices;
    std::vector<Vector3D> normals;
  };
} // namespace CGL

#endif /* pointCloud_h */
