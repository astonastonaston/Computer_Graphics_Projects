//
//  BPFront.hpp
//  ColladaViewer
//
//  Created by Benjamin Pust on 5/10/19.
//

#ifndef front_hpp
#define front_hpp

#include <stdio.h>
#include <vector>
#include <CGL/CGL.h>
#include "mesh.h"
#include "meshEdit.h"

namespace CGL {

  class BPLoop;
  class BPEdge;
  class BPFront;

  class BPLoop {
  public:
    BPLoop(BPEdge* startEdge, BPFront *front);
    BPEdge* startEdge;
    BPFront *front;
  };







  class BPFront {
  public:
    BPFront(std::vector<CGL::Vector3D> *vertices, std::vector<CGL::Vector3D> *normals, CGL::Polymesh* pm);
    double BP(double rho, BPFront *commonFront);
    
    CGL::Polymesh *polymesh;
    std::vector<BPLoop *> loops;

    std::vector<CGL::Vector3D> vertices;
    std::vector<CGL::Vector3D> normals;
    std::vector<bool> verticesOnFront;
    std::vector<bool> verticesUsed;
    
    void join(BPEdge* edge_ij, int k);
    bool glue(int m, int n);
    bool check(int m, int n);
    void outputTriangleToMesh(int i, int j, int k);
    BPEdge* getActiveEdge();
    BPLoop* insertEdge(BPEdge *edge);
    bool findSeedTriangle(std::vector<int> *indices, double rho, BPFront *commonFront);
    bool findSeedTriangleIndices(std::vector<int>* indices, double rho, BPFront *commonFront);
    std::vector<int> findNearbyPoints(double rho, int cand_idx, BPFront *commonFront);
  };







  class BPEdge {
  public:
    BPEdge(int i, int j, int o, BPEdge *prev, BPEdge *next, BPLoop *loop);
    
    int i, j, o;
    BPEdge *prev;
    BPEdge *next;
    BPLoop *loop;
    
    bool isActive;
    
    bool pivotOperation(double rho, int *k, BPFront *commonFront);
    std::vector<int> findPoint(double rho, CGL::Vector3D m, std::vector<CGL::Vector3D> vertices);
    void markNotActive();
    
    CGL::Vector3D getSphereCenter(CGL::Vector3D i,CGL::Vector3D j, CGL::Vector3D x, double rho);
    std::vector<int> findCandidatePoints(double rho, CGL::Vector3D m, int e_i, int e_j, std::vector<CGL::Vector3D> vertices, BPFront *commonFront);
  };

}

#endif /* front */
