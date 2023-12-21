//
//  BPFront.cpp
//  ColladaViewer
//
//  Created by Benjamin Pust on 5/10/19.
//

#include "front.hpp"
#include <time.h>


namespace CGL
{
  // initializers
  BPLoop::BPLoop(BPEdge* startEdge, BPFront *front){
    this->startEdge = startEdge;
    this->front = front;
  };

  BPFront::BPFront(std::vector<Vector3D> *vertices, std::vector<Vector3D> *normals, CGL::Polymesh* pm){
    this->vertices = *vertices;
    this->normals = *normals;
    this->polymesh = pm;
    pm->vertices = *vertices;
    pm->normals = *normals;
    verticesUsed = std::vector<bool>(vertices->size(), false);
    verticesOnFront = std::vector<bool>(vertices->size(), false);
  };
  
  BPEdge::BPEdge(int i, int j, int o, BPEdge *prev, BPEdge *next, BPLoop *loop) {
    this->i = i;
    this->j = j;
    this->o = o;
    this->prev = prev;
    this->next = next;
    this->loop = loop;
    isActive = true;
  };

  // frontier part
  bool BPFront::findSeedTriangle(std::vector<int> *indices, double rou, BPFront *frontier) {
    if (!findSeedTriangleIndices(indices, rou, frontier))
      return false;
    // cout << "seed found!";
    int i = (*indices)[0];
    int j = (*indices)[1];
    int k = (*indices)[2];
    
    std::vector<int> tIndex = std::vector<int>();
    tIndex.push_back(i);
    tIndex.push_back(j);
    tIndex.push_back(k);
    
    *indices = tIndex;
    outputTriangleToMesh(i, j, k);

    // this->verticesOnFront[i] = true;
    // this->verticesUsed[i] = true;
    // this->verticesOnFront[j] = true;
    // this->verticesUsed[j] = true;
    // this->verticesOnFront[k] = true;
    // this->verticesUsed[k] = true;
    
    BPEdge *e0, *e1, *e2;
    BPLoop* loop;
    loop = new BPLoop(nullptr, this);
    e0 = new BPEdge(i,j,k, nullptr, nullptr, loop);
    e1 = new BPEdge(j,k,i, nullptr, nullptr, loop);
    e2 = new BPEdge(k,i,j, nullptr, nullptr, loop);
    loop->startEdge = e0;
    e0->prev = e2;
    e0->next = e1;
    e1->prev = e0;
    e1->next = e2;
    e2->prev = e1;
    e2->next = e0;
    this->loops.push_back(loop);
    
    return true;
  }

  double BPFront::BP(double rou, BPFront *frontier) {
    frontier = this;
    std::vector<int> seed_triangle_indices;

    // start counting time
    clock_t stt_time = clock();

    // initialize with the seed triangle
    if (!findSeedTriangle(&seed_triangle_indices, rou, frontier)) { cout << "u dead\n"; return 0; }
    printf("Seed triangle indices: %d %d %d\n", seed_triangle_indices[0], seed_triangle_indices[1], seed_triangle_indices[2]);

    // start BPA loop
    BPEdge * edge = getActiveEdge();
    double i=1, j=1;
    while (true){
      // cout << "loop: " << i++ << endl;
      // expand the current frontier 
      while (edge != nullptr){
        // cout << "edge: " << j++ << endl;
        int k;

        // pivot on the edge
        bool pivot_success = edge->pivotOperation(rou, &k, frontier);
        if (pivot_success){
          // successful: update mesh
          outputTriangleToMesh(edge->j, edge->i, k);
          this->verticesOnFront[k] = true;
          this->verticesUsed[k] = true;

          // join the new edge (and glue the repeated edges)
          join(edge, k);
        }

        // mark the edge inactive (so that no more pivot can happen on that edge)
        edge->markNotActive();

        // get the next active edge
        edge = getActiveEdge();
      }

      // no more seed triangle: end the function, BP finished
      if (!findSeedTriangle(&seed_triangle_indices ,rou, frontier)) break;
    }

    // finish time counting
    clock_t fin_time = clock();
    cout << "stt time: " << stt_time << endl;
    cout << "fin time: " << fin_time << endl;
    cout << "time complexity: " << (double)(fin_time-stt_time)/CLOCKS_PER_SEC << endl;
    return (double)(fin_time-stt_time)/CLOCKS_PER_SEC;

  }

  void BPFront::join(BPEdge* e_ij, int k) {
    // build and connect the two new edges e_ik and e_kj
    Vector3D v_k = vertices[k];
    BPEdge* e_ik;
    BPEdge* e_kj;
    e_ik = new BPEdge(e_ij->i, k, e_ij->j, e_ij->prev, nullptr, e_ij->loop);
    e_kj = new BPEdge(k, e_ij->j, e_ij->i, e_ik, e_ij->next, e_ij->loop);
    e_ik->next = e_kj;
    e_ij->prev->next = e_ik;
    e_ij->next->prev = e_kj;
    
    // update vertices' used status
    this->verticesOnFront[k] = true;
    this->verticesUsed[k] = true;
    e_ij->loop->startEdge = e_ij;
    this->verticesOnFront[e_ij->i] = false;
    this->verticesOnFront[e_ij->j] = false;    

    // glue repeated edges
    if (check(e_ij->i, k) || glue(k, e_ij->j)) this->verticesOnFront[k] = false;
    if (glue(e_ij->i, k)) e_ik->markNotActive();
    if (glue(k, e_ij->j)) e_kj->markNotActive(); 
  }

  bool BPFront::glue(int m, int n){
    for (int i = 0; i < loops.size(); i++) {
      BPEdge* edge = loops[i]->startEdge;
      // current edge is the target edge: glue (mark inactive) and return
      if ((edge->i == n && edge->j == m) || (edge->i == m && edge->j == n)) {
        edge->markNotActive();
        return true;
      }
      edge = edge->next;
      
      // otherwise: search through all edges for the repeated edge
      while (edge->i != loops[i]->startEdge->i && edge->j != loops[i] -> startEdge -> j) {
        if ((edge->i == n && edge->j == m) || (edge->i == m && edge->j == n)) {
          // found repeated edge, glue (mark inactive) and return
          edge->markNotActive();
          return true;
        }
        edge = edge->next;
      }
    }

    // no repeated edge to glue after the traverse check
    return false;
  }

  bool BPFront::check(int m, int n){
    // check if there exists edge with indices m,n in the frontier
    // if have, mark that as inactive
    for (int i = 0; i < loops.size(); i++) {
      BPEdge* edge = loops[i]->startEdge;
      if (edge->i == m && edge->j == n){
        edge->markNotActive();
        return true;
      }
      edge = edge->next;
      while (edge->i != loops[i]->startEdge->i && edge->j != loops[i]->startEdge->j) {
        if (edge->i == m && edge->j == n){
          edge->markNotActive();
          return true;
        }
        edge = edge->next;
      }
    }
    return false;
  }

  void BPFront::outputTriangleToMesh(int i, int j, int k) {
    CGL::Polygon poly;
    std::vector<size_t> triangle_indices;
    triangle_indices.push_back(i);
    triangle_indices.push_back(j);
    triangle_indices.push_back(k);
    poly.vertex_indices = triangle_indices;
    polymesh->polygons.push_back(poly);
  }

  BPEdge* BPFront::getActiveEdge() {
    // pop an active edge from the frontier
    for (int i = 0; i < loops.size(); i++) {
      BPEdge* edge = loops[i]->startEdge;
      
      // cout << edge->isActive;
      if (edge->isActive){
        return edge;
      }
      edge = edge->next;
      // cout << loops[i]->startEdge->i << " " << loops[i]->startEdge->j;
      while (edge->i != loops[i]->startEdge->i && edge->j != loops[i] -> startEdge -> j) {
        if (edge->isActive){
          return edge;
        }
        edge = edge->next;
      }
    }
    return nullptr;
  };

  BPLoop *BPFront::insertEdge(BPEdge *edge) {
    BPLoop *loop = new BPLoop(edge, this);
    edge->loop = loop;
    this->loops.push_back(loop);
    return loop;
  }


  bool BPFront::findSeedTriangleIndices(std::vector<int>* indices, double rou, BPFront *frontier) {

    Vector3D b, bone, btwo;
    for (int index = 0; index < vertices.size(); index++) {
      if (!verticesUsed[index]) {
        b = vertices[index];

        std::vector<int> nV = findNearbyPoints(2 * rou, index, frontier);

        for (int j = 1; j < nV.size(); j++) {
          for (int i = 0; i < j; i++) {
            // try all vertices (except for repeated ones)
            if (index == nV[i] || index == nV[j] || frontier->verticesUsed[nV[i]] || frontier->verticesUsed[nV[j]]) continue;
            bone = vertices[nV[i]];
            btwo = vertices[nV[j]];
            Vector3D n = cross(bone - b, btwo - b);

            // compute ball center.
            Vector3D bi = bone - b;
            Vector3D bj = btwo - b;
            Vector3D c_circle = cross(bi.norm2() * btwo - bj.norm2() * bone, n) / 2 / n.norm2() + b;
            if (rou * rou - dot(c_circle - bone, c_circle - bone) >= 0) {
              double t = sqrt((rou * rou - (c_circle - bone).norm2()) / n.norm2());
              Vector3D c_ball;
              if (dot(n, normals[index]) > 0) c_ball = c_circle + n * t;
              else c_ball = c_circle - n * t;

              // Check if the ball contains no other points.
              bool empty_ball = true;
              for (Vector3D v : vertices) {
                if ((v - b).norm() > 1e-6 && (v - bone).norm() > 1e-6 && (v - btwo).norm() > 1e-6 && (v - c_circle).norm() < rou) {empty_ball = false; break;}
              }
              if (empty_ball) {
                // Found, mark that as the seed triangle.
                indices->push_back(index);
                indices->push_back(nV[i]);
                indices->push_back(nV[j]);
                // mark the used status of the vertices
                frontier->verticesOnFront[nV[i]] = true;
                frontier->verticesOnFront[nV[j]] = true;
                frontier->verticesOnFront[index] = true;
                frontier->verticesUsed[nV[i]] = true;
                frontier->verticesUsed[nV[j]] = true;
                frontier->verticesUsed[index] = true;
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }

  std::vector<int> BPFront::findNearbyPoints(double rou, int candidate, BPFront *frontier) {
    std::vector<int> cand = std::vector<int>();
    for (std::size_t i = 0; i != vertices.size(); ++i) {
      if (((vertices[i] - vertices[candidate]).norm() < 2 * rou) && (candidate != i) && !frontier->verticesUsed[i]) cand.push_back(i);
    }
    return cand;
  }


  // edge part
  bool BPEdge::pivotOperation(double rou, int *k, BPFront *frontier) {
    // pivot on the given frontier with rou
    std::vector<Vector3D> vertices = frontier->vertices;
    
    // get the pivot edge's vertices
    Vector3D m = (vertices[i] + vertices[j])/2;
    Vector3D i = vertices[this->i];
    Vector3D j = vertices[this->j];
    // get the pivot edge's opposite vertex's coordinates (target pivot vertex)
    Vector3D o = vertices[this->o];

    Vector3D c_ijo = getSphereCenter(i,j,o,rou);
    double r = (m - c_ijo).norm();
    
    // get unused points closed to the midpoint 
    std::vector<int> candidates = findCandidatePoints(rou, m, this->i, this->j, frontier->vertices, frontier), center_indices, nouse_c_ind;
    std::vector<Vector3D> centers, nouse_c;
    Vector3D b = vertices[this->o] - m, a;

    for (int ix = 0; ix < candidates.size(); ix++) {
      int iix = candidates[ix];

      // get candidate vertex and compute ball center
      Vector3D x = vertices[iix], c = getSphereCenter(i, j, x, rou);
      
      // invalid ball center, jump the candidate
      if (c.x == 9999 && c.y == 9999 && c.z == 9999){
        continue;
      }    

      // pick valid pivot target points
      if (abs((c-m).norm() - r) < 0.0001 && (frontier->verticesOnFront[iix] || !frontier->verticesUsed[iix])) {
        if (!frontier->verticesUsed[iix]){
          nouse_c_ind.push_back(iix);
          nouse_c.push_back(c);
        }
        else{
          center_indices.push_back(iix);
          centers.push_back(c);
        }
      }
    }
    
    // no valid ball center: report invalid status
    if (centers.size() + nouse_c.size() == 0) return false;
    
    // get the maximal ball center projection (for the nearest ball center) corresponding vertex index
    double max_proj = std::numeric_limits<double>::lowest();
    int first_index;
    if (nouse_c.size() > 0) {
      for (std::size_t i = 0; i != nouse_c.size(); ++i) {
        a = nouse_c[i] - m;
        if (dot(a,b) > max_proj ){
          max_proj = dot(a,b);
          first_index = nouse_c_ind[i];
        }
      }
    }
    else {
      for (std::size_t i = 0; i != centers.size(); ++i) {
        a = centers[i] - m;
        if (dot(a,b) > max_proj ){
          max_proj = dot(a,b);
          first_index = center_indices[i];
        }
      }
    }
    if (max_proj == std::numeric_limits<double>::lowest()) {
      *k = 99999999;
      return false;
    }

    // record the closest vertex index
    *k = first_index;
    return true;
  }

  void BPEdge::markNotActive() {
    isActive = false;
  }

  Vector3D BPEdge::getSphereCenter(Vector3D i,Vector3D j, Vector3D x, double rou) {
    // compute the ball center, given intersacted vertices and the radius
    Vector3D ji = j-i;
    Vector3D xi = x-i;
    Vector3D n = cross(ji, xi);
    Vector3D p0 = cross(dot(ji, ji) * xi - dot(xi, xi) * ji, n) / (2 * dot(n, n)) + i;
    if (dot(n, n) == 0) {
      // input zero vector: return invalid
      return Vector3D(9999, 9999, 9999);
    }

    // compute the ball center
    double t1 = sqrt((rou*rou - dot(p0-i, p0-i))/ dot(n, n));
    Vector3D c1 = p0 + (n * t1);
    return c1;
  }

  std::vector<int> BPEdge::findCandidatePoints(double rou, Vector3D m, int e_i, int e_j, std::vector<Vector3D> vertices, BPFront *frontier) {
    // find available points closed to the midpoint m for pivoting
    std::vector<int> candidates;
    for (std::size_t i = 0; i != vertices.size(); ++i) {
      if ((vertices[i] - m).norm()<2*rou && i!=e_i && i!=e_j && (!frontier->verticesUsed[i] || frontier->verticesOnFront[i])) candidates.push_back(i);
    }
    return candidates;
  }
}