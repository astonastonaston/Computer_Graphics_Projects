#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"

using namespace std;

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
    void upsample_butterfly(HalfedgeMesh& mesh);

  protected:
    // calculate the new position of the edge point w.r.t. an irregular vertex (for butterfly subdiv)
    // A denotes the irregular point, irr_deg is the degree of A, newpos stands for the position in correspondance
    // By default, newpos is directly added, instead of re-initializing
    void cal_new_pos_irregular(HalfedgeIter A_he, size_t irr_deg, Vector3D& newpos);

    // calculate boundary edge corresponding point's new position (w.r.t. the boundary edge)
    void cal_new_pos_boundary(HalfedgeIter he_bound, Vector3D& newpos);
  };
}

#endif // STUDENT_CODE_H
