#include "student_code.h"
#include "mutablePriorityQueue.h"


using namespace std;

namespace CGL
{
  // double rou = 1.7;

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Task 1.
    std::vector<Vector2D> res;
    for (int i = 0; i < points.size() - 1; i++) {
      // printf("in iter %d\n", i);
      res.push_back(points[i]*(1-this->t) + points[i+1]*this->t);
    }
    return (res);
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> res;
    res.clear();
    for (int i = 0; i < points.size() - 1; i++) {
      res.push_back(points[i]*(1-t) + points[i+1]*t);
    }
    // std::cout << res[0] << " " << res[1] << " " << res[2] << std::endl;
    // std::cout << res.size() << std::endl;

    return (res);
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    // given ctr points and lerp parameter t, it returns the evaluated point on the curve
    std::vector<Vector3D> res;
    res = evaluateStep(points, t);
    while (res.size() > 1) res = evaluateStep(res, t);
    return res[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter (in row)
   * @param v         Scalar interpolation parameter (along the other axis) (in column)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Task 2.
    // n row-wise evaluation for u, 1 column-wise evaluation for v
    std::vector<Vector3D> target_approx_col;
    for (int i = 0; i < controlPoints.size(); i++) {
      // printf("ev at %d\n", i);
      target_approx_col.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(target_approx_col, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Task 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    // Traverse to get neighbour points
    std::vector<Vector3D> neis;
    HalfedgeCIter h = this->halfedge();
    do
    {
       // do something interesting with h
       h = h->twin();
       neis.push_back(h->vertex()->position);
       h = h->next();
    }
    while( h != this->halfedge() );

    // computed weighted area face normal
    Vector3D res(0,0,0);
    for( int i = 0; i < neis.size()-1; i++) {
      res = res + cross(neis[i+1] - this->position, neis[i] - this->position)/2;
    }
    res.normalize();
    return res;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Task 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if ( e0->isBoundary() ) return e0;
    
    HalfedgeIter edge_he = e0->halfedge();

    // change Edge attributes (halfedge)
    // change Vertex attributes (halfedge)
    edge_he->twin()->vertex()->halfedge() = edge_he->next(); // a
    edge_he->vertex()->halfedge() = edge_he->twin()->next(); // c
    // edge_he->next()->twin()->vertex()->halfedge() = // b
    // edge_he->twin()->next()->twin()->vertex()->halfedge() =  // d
    // change Face attributes (halfedge)
    edge_he->face()->halfedge() = edge_he; // A
    edge_he->twin()->face()->halfedge() = edge_he->twin(); // B
    // change Halfedge attributes (twin, next, face, vertex, edge) (actually just considering next, face, vertex is enough)
    edge_he->next()->next()->next() = edge_he->twin()->next(); // l_A next
    edge_he->next()->next()->face() = edge_he->twin()->face(); // l_A face

    // report can write: 误传引用导致修改后保存的迭代器变量值丢失？？？
    HalfedgeIter he_l_A = edge_he->next()->next(); // backup edge l_A's info
    edge_he->next()->next() = edge_he; // i_A next

    edge_he->twin()->next()->next()->next() = edge_he->next(); // j_B next
    edge_he->twin()->next()->next()->face() = edge_he->face(); // j_B face

    HalfedgeIter he_j_B = edge_he->twin()->next()->next(); // backup edge j_B's info
    edge_he->twin()->next()->next() = edge_he->twin(); // k_B next

    edge_he->twin()->vertex() = edge_he->twin()->next()->twin()->vertex(); // m_B vertex
    edge_he->twin()->next() = he_l_A; // m_B next

    edge_he->vertex() = edge_he->twin()->next()->vertex(); // m_A vertex
    edge_he->next() = he_j_B; // m_A next

    return edge_he->edge();
    // return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Task 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    HalfedgeIter he_k_A = e0->halfedge();
    // boundary case
    // You may refer to the report for the meanings of those letters
    if ( e0->isBoundary() ) {
      // create new mesh elements
      VertexIter m = this->newVertex();

      EdgeIter w = this->newEdge(); 
      EdgeIter v = this->newEdge();

      HalfedgeIter v_B = this->newHalfedge();
      HalfedgeIter v_B_twin = this->newHalfedge();
      HalfedgeIter w_A = this->newHalfedge();
      HalfedgeIter w_B = this->newHalfedge();

      FaceIter B = this->newFace();
      // init new mesh elements
      // vertex
      m->halfedge() = he_k_A; // halfedge
      m->position = (he_k_A->vertex()->position + he_k_A->twin()->vertex()->position)/2; // position
      m->isNew = true; 
      
      // edge
      w->halfedge() = w_A; // halfedge
      w->isNew = true; 
      v->halfedge() = v_B;
      v->newPosition = he_k_A->edge()->newPosition;
      v->isNew = false; 

      // face
      B->halfedge() = v_B; // halfedge

      // halfedge
      v_B->setNeighbors( w_B, v_B_twin, he_k_A->vertex(), v, B); // (next, twin, vertex, edge, face)
      v_B_twin->setNeighbors( he_k_A->twin()->next(), v_B, m, v, he_k_A->twin()->face());
      w_A->setNeighbors( he_k_A, w_B, he_k_A->next()->next()->vertex(), w, he_k_A->face());
      w_B->setNeighbors( he_k_A->next()->next(), w_A, m, w, B);
      // modify old mesh elements
      // vertex
      he_k_A->vertex()->halfedge() = v_B; // b

      // edge
      // face
      he_k_A->face()->halfedge() = he_k_A;
      
      // halfedge
      he_k_A->next()->next()->setNeighbors(v_B, he_k_A->next()->next()->twin(), he_k_A->next()->next()->vertex(), he_k_A->next()->next()->edge(), B); // z_A
      he_k_A->next()->setNeighbors(w_A, he_k_A->next()->twin(), he_k_A->next()->vertex(), he_k_A->next()->edge(), he_k_A->next()->face()); // x_A
      he_k_A->twin()->setNeighbors(v_B_twin, he_k_A, he_k_A->twin()->vertex(), he_k_A->twin()->edge(), he_k_A->twin()->face()); // k_A_twin
      he_k_A->setNeighbors(he_k_A->next(), he_k_A->twin(), m, he_k_A->edge(), he_k_A->face()); // k_A

      return m;
    }
    // non-boundary case
    else {
      // create new mesh elements
      VertexIter m = this->newVertex();

      EdgeIter u = this->newEdge(); 
      EdgeIter l = this->newEdge();
      EdgeIter v = this->newEdge();

      FaceIter C = this->newFace();
      FaceIter D = this->newFace();

      HalfedgeIter l_C = this->newHalfedge();
      HalfedgeIter l_D = this->newHalfedge();
      HalfedgeIter v_B = this->newHalfedge();
      HalfedgeIter v_D = this->newHalfedge();
      HalfedgeIter u_C = this->newHalfedge();
      HalfedgeIter u_A = this->newHalfedge();

      // init new mesh elements
      // vertex
      m->halfedge() = he_k_A; // halfedge
      m->isNew = true; 
      m->position = (he_k_A->vertex()->position + he_k_A->twin()->vertex()->position)/2; // position
      
      // edge
      u->halfedge() = u_C; // halfedge
      l->halfedge() = l_C;
      l->newPosition = he_k_A->edge()->newPosition;
      v->halfedge() = v_D;
      u->isNew = true; 
      l->isNew = false; 
      v->isNew = true; 

      // face
      C->halfedge() = u_C; // halfedge
      D->halfedge() = v_D;

      // halfedge
      l_C->setNeighbors( u_C, l_D, he_k_A->vertex(), l, C); // (next, twin, vertex, edge, face)
      l_D->setNeighbors( he_k_A->twin()->next(), l_C, m, l, D);
      u_C->setNeighbors( he_k_A->next()->next(), u_A, m, u, C);
      u_A->setNeighbors( he_k_A, u_C, he_k_A->next()->next()->vertex(), u, he_k_A->face());
      v_D->setNeighbors( l_D, v_B, he_k_A->twin()->next()->twin()->vertex(), v, D);
      v_B->setNeighbors( he_k_A->twin()->next()->next(), v_D, m, v, he_k_A->twin()->face());

      // update old mesh elements
      // vertex
      he_k_A->vertex()->halfedge() = he_k_A->twin()->next(); // b
      // edge    
      // face
      he_k_A->face()->halfedge() = he_k_A->next(); // A
      he_k_A->twin()->face()->halfedge() = he_k_A->twin()->next()->next(); // B
      // halfedge (next, twin, vertex, edge, face)
      he_k_A->next()->next()->setNeighbors(l_C, he_k_A->next()->next()->twin(), he_k_A->next()->next()->vertex(), he_k_A->next()->next()->edge(), C);  // z_A
      he_k_A->next()->setNeighbors(u_A, he_k_A->next()->twin(), he_k_A->next()->vertex(), he_k_A->next()->edge(), he_k_A->next()->face());  // w_A
      he_k_A->twin()->next()->next()->setNeighbors(he_k_A->twin(), he_k_A->twin()->next()->next()->twin(), he_k_A->twin()->next()->next()->vertex(), he_k_A->twin()->next()->next()->edge(), he_k_A->twin()->next()->next()->face()); // x_B
      he_k_A->twin()->next()->setNeighbors(v_D, he_k_A->twin()->next()->twin(), he_k_A->twin()->next()->vertex(), he_k_A->twin()->next()->edge(), D); // y_B
      he_k_A->twin()->setNeighbors(v_B, he_k_A, he_k_A->twin()->vertex(), he_k_A->twin()->edge(), he_k_A->twin()->face()); // k_B
      he_k_A->setNeighbors(he_k_A->next(), he_k_A->twin(), m, he_k_A->edge(), he_k_A->face()); // k_A
      return m;
    }
  }

  // A denotes the irregular point, irr_deg is the degree of A, newpos stands for the position in correspondance
  // By default, newpos is directly added, instead of re-initializing
  void MeshResampler::cal_new_pos_irregular(HalfedgeIter A_he, size_t irr_deg, Vector3D& newpos) {
    // init local vars that would be used
    HalfedgeIter heit=A_he;
    int j = 0; // vertex index
    newpos = newpos + (3.0/4) * heit->vertex()->position;
    // loop over the irregular point
    // K>=5 case
    if (irr_deg >= 5) {
      do {
        heit=heit->twin();

        // iteration reversed!
        if (j==0) newpos = newpos + ( ( (1.0/4) + cos((2*M_PI*j)/irr_deg) + (1.0/2) * cos((4*M_PI*j)/irr_deg) ) / irr_deg ) * heit->vertex()->position;
        else newpos = newpos + ( ( (1.0/4) + cos((2*M_PI*(irr_deg - j))/irr_deg) + (1.0/2) * cos((4*M_PI*(irr_deg - j))/irr_deg) ) / irr_deg ) * heit->vertex()->position;

        heit=heit->next();
        j++;
      } while (heit != A_he);
      // newpos = newpos + (3.0/4) * heit->vertex()->position;
    }
    // K=3 case
    else if (irr_deg == 3) {
      do {
        heit=heit->twin();

        if (j==0) newpos = newpos + ( 5.0/12 ) * heit->vertex()->position;
        else newpos = newpos + ( -1.0/12 ) * heit->vertex()->position;

        heit=heit->next();
        j++;
      } while (heit != A_he);
    }
    // K=4 case
    else if (irr_deg == 4) {
      do {
        heit=heit->twin();

        if (j==0) newpos = newpos + ( 3.0/8 ) * heit->vertex()->position;
        else if (j==2) newpos = newpos + ( -1.0/8 ) * heit->vertex()->position;

        heit=heit->next();
        j++;
      } while (heit != A_he);
    }

    // if ((abs(newpos.x) <= 0.08) && (abs(newpos.y) <= 0.08) && (abs(newpos.z) <= 0.08)) {
    //   std::cout << "half6 pws " << newpos << std::endl;
    //   std::cout << "irrdeg " << irr_deg << std::endl;
    //   std::cout << "vert1 " << A_he->vertex()->position << std::endl;
    //   std::cout << "vert2 " << A_he->twin()->vertex()->position << std::endl;
    // } 
  }

  void MeshResampler::cal_new_pos_boundary(HalfedgeIter he_bound, Vector3D& newpos) {
    newpos = newpos + (9.0/16) * he_bound->vertex()->position;
    newpos = newpos + (9.0/16) * he_bound->twin()->vertex()->position;

    HalfedgeIter heit=he_bound;
    // add boundary vertices on the other boundary edges
    // compute B-side boundary
    // heit = heit->twin()->next();
    do {
      heit=heit->twin();
      heit=heit->next();

      if (heit->edge()->isBoundary()) {
        newpos = newpos + (-1.0/16) * heit->twin()->vertex()->position;
        // std::cout << "nwpsBB " << heit->twin()->vertex()->position << std::endl;
      }

    } while (!heit->edge()->isBoundary());

    // compute A-side boundary
    heit=he_bound->twin();
    do {
      heit=heit->twin();
      heit=heit->next();

      if (heit->edge()->isBoundary()) {
        newpos = newpos + (-1.0/16) * heit->twin()->vertex()->position;
        // std::cout << "nwpsAA " << heit->twin()->vertex()->position << std::endl;
      }

    } while (!heit->edge()->isBoundary());
    // std::cout << "bdy inv pos " << newpos << std::endl;
  };

  void MeshResampler::upsample_butterfly(HalfedgeMesh& mesh) {
    // compute new locs for midpts and vertices
    // for (VertexIter vert=mesh.verticesBegin(); vert != mesh.verticesEnd(); vert++) {
    // }
    // printf("buff inv!\n");
    size_t vert_deg_A=0, vert_deg_B=0, irr_deg=0; // degree of irregular vertex, and the iteration index (for calculating weights)
    HalfedgeIter he;
    Vector3D newpos(0,0,0);
    int ite_times=0;

    // init vertices isNew to false (for repeated subdivision)
    for (VertexIter vert=mesh.verticesBegin(); vert!=mesh.verticesEnd(); vert++) vert->isNew = false;

    for (EdgeIter edg=mesh.edgesBegin(); edg != mesh.edgesEnd(); edg++) {
      edg->isNew = false;
      ite_times++;
      if (!edg->isBoundary()) {
        vert_deg_A = edg->halfedge()->vertex()->degree();
        // for boundary edge, degree incr by 1
        if (edg->halfedge()->vertex()->isBoundary()) vert_deg_A++;

        vert_deg_B = edg->halfedge()->twin()->vertex()->degree();
        if (edg->halfedge()->twin()->vertex()->isBoundary()) vert_deg_B++; 

        he = edg->halfedge();
        // two deg=6 vertices
        if ((vert_deg_B==6) && (vert_deg_A==6)) {
          // one-side vertex iteration
          for (int i = 0; i <= 5; i++) {
            // not cnting the opposite vertex
            he = he->twin();
            switch (i)
            {
            case (0): {
              newpos = newpos + (1.0/2) * he->vertex()->position;
              break;
            }            
            case (1,5): {
              newpos = newpos + (1.0/8) * he->vertex()->position;
              break;
            }
            case (2,4): {
              newpos = newpos + (-1.0/16) * he->vertex()->position;
              break;
            }
            default:
              break;
            }
            he = he->next();
          }

          // other-side vertex iteration
          he=he->twin();
          for (int i = 0; i<=5; i++) {
            he=he->twin();
            switch (i)
            {
            case (0): {
              newpos = newpos + (1.0/2) * he->vertex()->position;
              break;
            }
            case (2, 4): {
              newpos = newpos + (-1.0/16) * he->vertex()->position;
              break;
            }
            default:
              break;
            }
            he=he->next();
          }
        }
        // one deg=6 and one deg=K
        else if ((vert_deg_B==6)||(vert_deg_A==6)) {
          // Select the irregular point
          irr_deg = vert_deg_A;

          // Check irregular-point-correspondant halfedge
          if (vert_deg_A==6) {
            he=he->twin();
            irr_deg = vert_deg_B;
          }

          // loop over the irregular point
          this->cal_new_pos_irregular(he, irr_deg, newpos);
        }
        // two deg=K
        else {
          // take average from two irregular points
          this->cal_new_pos_irregular(he, vert_deg_A, newpos);
          this->cal_new_pos_irregular(he->twin(), vert_deg_B, newpos);
          newpos = newpos/2;
          // std::cout << "irrpos is " << newpos << std::endl;

        }
      }

      // boundary edge case
      else {
        this->cal_new_pos_boundary(edg->halfedge(), newpos);
      }

      // update new position to edge
      edg->newPosition = newpos;

      // clear newpos for next iteration use
      newpos.x=0;
      newpos.y=0;
      newpos.z=0;
    }

    // Subdivision
    EdgeIter edgbeg = mesh.edgesBegin(), edgend = mesh.edgesEnd();
    edgend--;
    int old_cnt = 0;
    // std::cout << "before split , isnew " << edgend->isNew << std::endl;
    for (EdgeIter egit = edgbeg; old_cnt != ite_times; egit++) {
      mesh.splitEdge(egit);
      old_cnt++;
    }
    
    // std::cout << "after split , isnew " << edgend->isNew << std::endl;
    edgend++;

    // Flipping
    // std::cout << "after incr, isnew " << edgend->isNew << std::endl;
    for (EdgeIter edg=edgend; edg!=mesh.edgesEnd(); edg++) {
      if (((edg->halfedge()->vertex()->isNew) xor (edg->halfedge()->twin()->vertex()->isNew))&&(edg->isNew)&&(!edg->isBoundary())) mesh.flipEdge(edg);
    }
    // printf("edg fli dn\n");
    
    // Update new locs
    for (VertexIter vert=mesh.verticesBegin(); vert!=mesh.verticesEnd(); vert++) {
      if (vert->isNew) {
        // for midpoint, its halfedge should correspond to the old edge
        HalfedgeIter he_ite = vert->halfedge();
        do {
          he_ite = he_ite->twin()->next();
        }
        while(he_ite->edge()->isNew); 
        // printf("nw pt ass %d %d %d", he_ite->edge()->newPosition[0], he_ite->edge()->newPosition[1], he_ite->edge()->newPosition[2]);
        vert->position = he_ite->edge()->newPosition; 
      }
    }
  }


  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // old edge::new_pos, old vertex::now_pos -> new vertex::position
    // compute positions -> subdivision -> update positions
    // TODO Task 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    // printf("befo begi\n");
    Vector3D neiSum(0.0,0.0,0.0);
    double u;
    size_t n;

    for (VertexIter vert=mesh.verticesBegin(); vert!=mesh.verticesEnd(); vert++) {
      if (!vert->isBoundary()) {
        // compute and update vertex new position
        n = vert->degree();
        if (n!=3) u = 3.0/(8*n);
        else u = 3.0/16;
        // std::cout << "u,n are " << u << " and " << n << std::endl;

        // vert->computeCentroid();
        // neiSum = vert->centroid * n;
        HalfedgeIter he_it=vert->halfedge();
        do
        {
          he_it=he_it->twin();
          // std::cout << he_it->vertex()->position << "is the pos\n";
          neiSum = neiSum + he_it->vertex()->position;
          // std::cout << neiSum << "summed\n";
          he_it=he_it->next();
        } while (he_it != vert->halfedge());
        // neiSum = neiSum / n;
        // std::cout << "neisum is " << neiSum << std::endl;

            
        vert->newPosition = (1.0-n*u)*vert->position + u*neiSum;
        neiSum.x=0;
        neiSum.y=0;
        neiSum.z=0;
        // std::cout << vert->newPosition << "loaded\n";
      }
      else {
        // compute and update vertex new position
        // vert->computeCentroid();
        // neiSum = vert->centroid * n;
        HalfedgeIter he_it=vert->halfedge();
        neiSum = neiSum + 3.0/4 * he_it->vertex()->position;
        do
        {
          he_it=he_it->twin();
          // std::cout << he_it->vertex()->position << "is the pos\n";
          if (he_it->edge()->isBoundary())  neiSum = neiSum + 1.0/8 * he_it->vertex()->position;
          // std::cout << neiSum << "summed\n";
          he_it=he_it->next();
        } while (he_it != vert->halfedge());
        // neiSum = neiSum / n;
        // std::cout << "neisum is " << neiSum << std::endl;
        vert->newPosition = neiSum;
        neiSum.x=0;
        neiSum.y=0;
        neiSum.z=0;
        // std::cout << vert->newPosition << "loaded\n";
      }
      
      // mark original vertex
      vert->isNew = false;
    }
    // printf("cal nwpo dn\n");


    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    Vector3D pos_A, pos_B, pos_C, pos_D;
    // printf("%p egbg %p egend\n", mesh.edgesBegin(), mesh.edgesEnd());
    EdgeIter edgbeg = mesh.edgesBegin(), edgend = mesh.edgesEnd();
    int ite_times = 0;
    for (EdgeIter edg=edgbeg; edg!=edgend; edg++) {
      if (!edg->isBoundary()) {
        // compute and update edge midpt new position
        pos_A = edg->halfedge()->vertex()->position;
        pos_B = edg->halfedge()->twin()->vertex()->position;
        pos_C = edg->halfedge()->next()->twin()->vertex()->position;
        pos_D = edg->halfedge()->twin()->next()->twin()->vertex()->position;
        edg->newPosition = (3.0/8) * (pos_A + pos_B) + (1.0/8) * (pos_C + pos_D);
      }
      else {
        pos_A = edg->halfedge()->vertex()->position;
        pos_B = edg->halfedge()->twin()->vertex()->position;
        edg->newPosition = (1.0/2) * (pos_A + pos_B);
      }
      // mark original edgex
      edg->isNew = false;
      ite_times++;
    }
    // printf("edg midpt dn %d its\n", ite_times);

    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    edgend--;

    int old_cnt = 0;
    for (EdgeIter egit = edgbeg; old_cnt != ite_times; egit++) {
      mesh.splitEdge(egit);
      // printf("%d is\n", (egit==edgend));
      old_cnt++;
      // std::cout << edgend;
    }
    // printf("edg spli dn\n");

    // std::cout << "after split , isnew " << edgend->isNew << std::endl;

    // 4. Flip any new edge that connects an old and new vertex.
    edgend++;
    // std::cout << "aft " << edgend->isNew << std::endl;
    // std::cout << "after incr , isnew " << edgend->isNew << std::endl;

    for (EdgeIter edg=edgend; edg!=mesh.edgesEnd(); edg++) {
      if (((edg->halfedge()->vertex()->isNew) xor (edg->halfedge()->twin()->vertex()->isNew))&&(edg->isNew)&&(!edg->isBoundary())) mesh.flipEdge(edg);
    }
    // printf("edg fli dn\n");


    // 5. Copy the new vertex positions into final Vertex::position.
    HalfedgeIter he_ite;
    for (VertexIter vert=mesh.verticesBegin(); vert!=mesh.verticesEnd(); vert++) {
      if (!vert->isNew) vert->position = vert->newPosition;
      else {
        // for midpoint, its halfedge should correspond to the old edge
        he_ite = vert->halfedge();
        do {
          he_ite = he_ite->twin()->next();
        }
        while(he_ite->edge()->isNew); 
        // printf("nw pt ass %d %d %d", he_ite->edge()->newPosition[0], he_ite->edge()->newPosition[1], he_ite->edge()->newPosition[2]);
        vert->position = he_ite->edge()->newPosition;
      }
    }
  }

  // In a high-quality mesh, the change in size from one face or cell to the next should be gradual (as measured by smoothness). Large differences in size between adjacent faces or cells will result in a poor computational grid because the differential equations being solved assume that the cells shrink or grow smoothly.
  double compute_smoothness(Polymesh& pm) {
    double smoothness_sum=0;
    double ptnum = min({50000, (int)(pm.polygons.size()-1)});
    for (int i = 0; i <= ptnum-1; i++) {
      // obtain current polygon and its neighbour polygon
      vector<Index> current_pl=pm.polygons[i].vertex_indices, adjacent_pl; 
      Index vert1ind=current_pl[0], vert2ind=current_pl[1];
      for (auto j: pm.polygons) {
        if ((find(j.vertex_indices.begin(), j.vertex_indices.end(), vert1ind) != j.vertex_indices.end())
          && (find(j.vertex_indices.begin(), j.vertex_indices.end(), vert2ind) != j.vertex_indices.end())) {
          adjacent_pl = j.vertex_indices;
          break;
        }
      }

      // compute smoothness and contribute to the sum
      Vector3D currpt0=pm.vertices[current_pl[0]], 
        currpt1=pm.vertices[current_pl[1]],
        currpt2=pm.vertices[current_pl[2]];
      Vector3D adjpt0=pm.vertices[adjacent_pl[0]], 
        adjpt1=pm.vertices[adjacent_pl[1]],
        adjpt2=pm.vertices[adjacent_pl[2]];
      double areaCurr=cross(currpt1-currpt0, currpt2-currpt0).norm(), areaAdj=cross(adjpt1-adjpt0, adjpt2-adjpt0).norm();
      
      // compute area ratio (>=1)
      double ratio=0;
      if (areaCurr > areaAdj) ratio = areaCurr/areaAdj;
      else ratio = areaAdj/areaCurr;
      if (areaCurr==0 || areaAdj==0) ratio=0;
      smoothness_sum += ratio;
    }

    // return ratio average as smoothness
    return smoothness_sum / (1.0f*pm.polygons.size());
  }

  // The aspect ratio of a face or cell is the ratio of the longest edge length to the shortest edge length. The aspect ratio applies to triangular, tetrahedral, quadrilateral, and hexahedral elements and is defined differently for each element type.
  // The aspect ratio can also be used to determine how close to ideal a face or cell is.
  double compute_aspectratio(Polymesh& pm) {
    double aspectratio_sum=0, ratio=0;
    double ptnum = min({50000, (int)(pm.polygons.size()-1)});
    for (int i = 0; i <= ptnum-1; i++) {
      // obtain current polygon 
      vector<Index> current_pl=pm.polygons[i].vertex_indices; 
      
      // compute aspect ratio and contribute to the sum
      Vector3D currpt0=pm.vertices[current_pl[0]], 
        currpt1=pm.vertices[current_pl[1]],
        currpt2=pm.vertices[current_pl[2]],
        sideLengths((currpt1-currpt0).norm(), (currpt2-currpt0).norm(), (currpt1-currpt2).norm());
      
      // update aspect ratio sum
      ratio = (max({sideLengths[0], sideLengths[1], sideLengths[2]}) / min({sideLengths[0], sideLengths[1], sideLengths[2]}));
      if (min({sideLengths[0], sideLengths[1], sideLengths[2]}) == 0) ratio = 0;
      aspectratio_sum += ratio;
    }

    // return average aspect ratio over all faces
    return aspectratio_sum / ptnum;
  }
  
  // construct point-cloud mesh using BPA
  void MeshEdit::pc_to_mesh() {
    // init point cloud and the ball
    cout << "Constructing Mesh... " << endl;
    PointCloud pc = pointCloudNodes.back().point_cloud;
    Polymesh pm;
    cout << "Found " << pc.vertices.size() << " vertices." << endl;
    BPFront *front = new BPFront(&pc.vertices, &pc.normals, &pm);
    cout << "Built front." << endl;
    BPFront *global_front;

    // start construction loop to build the mesh
    double bp_secs = front->BP(rou, global_front);
    cout << ".. built mesh ..." << endl;
    cout << "Mesh reconstructed with " << pm.vertices.size() << " vertices and " << pm.polygons.size() << " faces." << endl;
    cout << front->polymesh->polygons.size() << endl;
    // int temp = pm.polygons[0].vertex_indices[1];
    // pm.polygons[0].vertex_indices[1] = pm.polygons[0].vertex_indices[2];
    // pm.polygons[0].vertex_indices[2] = temp;
    // // PolyList pl = pm.polygons;
    // int num = 0;
    // for (Polygon p: pm.polygons){
    //   vector<Index> ind = p.vertex_indices;
    //   Vector3D a, b, c;
    //   double cr;
    //   a = pc.vertices[ind[0]];
    //   b = pc.vertices[ind[1]];
    //   c = pc.vertices[ind[2]];
    //   for (int i = 0; i < 3; i++){
    //     if (ind[i%3] == 12573 && ind[(i+1)%3] == 15468){
    //       std::cout << ind[0]  << ", "<< ind[1] << ", " << ind[2] << std::endl;
    //       num++;
    //     }
    //     if (ind[i%3] == 15468 && ind[(i+1)%3] == 12573 ){
    //       std::cout << ind[0]  << ", "<< ind[1] << ", " << ind[2] << std::endl;
    //       num++;
    //     }
    //   }
    // }
    // printf("num: %d\n", num);
    // vector<Vector3D> temp_vec;
    // for (int i = 0; i < pm.vertices.size()-1; i++){
    //   temp_vec.push_back(pm.vertices[i]);
    // }
    // pm.vertices.clear();
    // pm.vertices = temp_vec;

    // init the mesh
    init_polymesh(pm);
    // compute mesh smoothness
    double smoothness = compute_smoothness(pm);
    // compute mesh average aspect ratio
    double aspectratio = compute_aspectratio(pm);
    // get mesh size
    double vertnum= pm.vertices.size(), facenum= pm.polygons.size();
    cout << "smoothness: " << smoothness << " aspect ratio: " << aspectratio << endl;
    cout << "vertex number: " << vertnum << " face number: " << facenum << endl;
    cout << "BPA time: " << bp_secs << " seconds" << endl;
    // render the new mesh
    render();
  }

}
