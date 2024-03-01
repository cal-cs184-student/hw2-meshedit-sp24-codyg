#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    Vector2D point1;
    Vector2D point2;
    std::vector<Vector2D> returnVec((points.size() - 1), Vector2D());
    for (int i = 0; i < (points.size() - 1); i++) {
        point1 = points[i];
        point2 = points[i + 1];
        returnVec[i] = ((1 - t) * point1) + (t * point2);
    }

    return returnVec;
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
    // TODO Part 2.
  Vector3D point1;
  Vector3D point2;
  std::vector<Vector3D> returnVec((points.size() - 1), Vector3D());
  for (int i = 0; i < (points.size() - 1); i++) {
      point1 = points[i];
      point2 = points[i + 1];
      returnVec[i] = ((1 - t) * point1) + (t * point2);
  }

  return returnVec;
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
    // TODO Part 2.
    std::vector<Vector3D> recurseVec = vector<Vector3D>(points);
    while (recurseVec.size() > 1) {
        recurseVec = evaluateStep(recurseVec, t);
    }

    return recurseVec[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> vLine(controlPoints.size(), Vector3D());
    for (int i = 0; i < controlPoints.size(); i++) {
        vLine[i] = evaluate1D(controlPoints[i], u);
    }

    return evaluate1D(vLine, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D vertex1 = position;
    Vector3D vertex2;
    Vector3D vertex3;
    float area;
    Vector3D totWeightedNorm = Vector3D();

    HalfedgeCIter vec1 = halfedge();
    HalfedgeCIter vec2;

    do {
        vec2 = vec1->next()->next()->twin();
        vertex2 = vec1->twin()->vertex()->position;
        vertex3 = vec2->twin()->vertex()->position;
        area = 0.5 * abs(vertex1.x * (vertex2.y - vertex3.y)
                + vertex2.x * (vertex3.y - vertex1.y)
                + vertex3.x * (vertex1.y - vertex2.y));
        totWeightedNorm += area * vec1->face()->normal();

        vec1 = vec2;
    } while (vec1 != halfedge());

    totWeightedNorm.normalize();

    return totWeightedNorm;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    if (e0->isBoundary()) {                                         //test if original edge is boundary
        return e0;
    }
//    if (e0->halfedge()->next()->edge()->isBoundary()                        //test next edge
//    || e0->halfedge()->next()->next()->edge()->isBoundary()                 //test next next edge
//    || e0->halfedge()->twin()->next()->edge()->isBoundary()                 //test twin's next edge
//    || e0->halfedge()->twin()->next()->next()->edge()->isBoundary()) {      //test twin's next next edge
//        return e0;
//    }

    //observe next, twin, vertex, edge, and face
    HalfedgeIter he1 = e0->halfedge();          //next to be 6, vertex to be a
    HalfedgeIter he2 = he1->next();             //next to be 1
    HalfedgeIter he3 = he2->next();             //next to be 5, face to be 2
    HalfedgeIter he4 = e0->halfedge()->twin();  //next to be 3, vertex to be d
    HalfedgeIter he5 = he4->next();             //next to be 4
    HalfedgeIter he6 = he5->next();             //next to be 2, face to be 1

    //observe half edge
    VertexIter b = he1->vertex();       //half edge to be 5
    VertexIter c = he2->vertex();       //half edge to be 2
    VertexIter a = he3->vertex();       //half edge to be 4
    VertexIter d = he6->vertex();       //half edge to be 1

    //observe half edge
    FaceIter face1 = he1->face();
    FaceIter face2 = he4->face();


    //set all new vals
    he1->setNeighbors(he3, he1->twin(), d, he1->edge(), he1->face());
    he2->setNeighbors(he4, he2->twin(), he2->vertex(), he2->edge(), face2);
    he3->setNeighbors(he5, he3->twin(), he3->vertex(), he3->edge(), he3->face());
    he4->setNeighbors(he6, he4->twin(), a, he4->edge(), he4->face());
    he5->setNeighbors(he1, he5->twin(), he5->vertex(), he5->edge(), face1);
    he6->setNeighbors(he2, he6->twin(), he6->vertex(), he6->edge(), he6->face());
    b->halfedge() = he5;
    c->halfedge() = he2;
    a->halfedge() = he4;
    d->halfedge() = he1;

    face1->halfedge() = he1;
    face2->halfedge() = he4;
    e0->halfedge() = he4;


    // This method should flip the given edge and return an iterator to the flipped edge.
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    //create 2 new faces, 1 new vertex, 3 new edges, and 6 new half edges
    if (e0->isBoundary()) {                                         //test if original edge is boundary
        return e0->halfedge()->vertex();
    }

    //observe next, twin, vertex, edge, and face
    HalfedgeIter he1 = e0->halfedge();          //vertex to be e
    HalfedgeIter he2 = he1->next();             //next to be 7
    HalfedgeIter he3 = he2->next();             //next to be 9, face to be 4
    HalfedgeIter he4 = e0->halfedge()->twin();  //next to be 12
    HalfedgeIter he5 = he4->next();             //next to be 11, face to be 3
    HalfedgeIter he6 = he5->next();             //next to be 4
    HalfedgeIter he7 = newHalfedge();           //next to be 1, twin to be 8, vertex to be a, edge to be 1, face to be 1
    HalfedgeIter he8 = newHalfedge();           //next to be 3, twin to be 7, vertex to be e, edge to be 1, face to be 4
    HalfedgeIter he9 = newHalfedge();           //next to be 8, twin to be 10, vertex to be b, edge to be 2, face to be 4
    HalfedgeIter he10 = newHalfedge();          //next to be 5, twin to be 9, vertex to be e, edge to be 2, face to be 3
    HalfedgeIter he11 = newHalfedge();          //next to be 10, twin to be 12, vertex to be d, edge to be 3, face to be 3
    HalfedgeIter he12 = newHalfedge();          //next to be 6, twin to be 11, vertex to be e, edge to be 3, face to be 2

    //observe half edge
    VertexIter a = he3->vertex();               //half edge to be 3
    VertexIter b = he1->vertex();               //half edge to be 5
    VertexIter c = he2->vertex();               //half edge to be 3
    VertexIter d = he6->vertex();               //half edge to be 6
    VertexIter e = newVertex();                 //half edge to be 1
    Vector3D pointA = e0->halfedge()->vertex()->position;
    Vector3D pointB = e0->halfedge()->twin()->vertex()->position;
    e->position = (pointA + pointB) / 2;
    e->newPosition = e0->newPosition;
    e->isNew = true;

    //observe half edge
    FaceIter face1 = he1->face();               //half edge to be 1
    FaceIter face2 = he4->face();               //half edge to be 4
    FaceIter face3 = newFace();                 //half edge to be 5
    FaceIter face4 = newFace();                 //half edge to be 3

    //create edge 1, 2, 3
    EdgeIter e1 = newEdge();                    //half edge to be 8
    EdgeIter e2 = newEdge();                    //half edge to be 10
    EdgeIter e3 = newEdge();                    //half edge to be 12


    //set all new vals
    he1->setNeighbors(he1->next(), he1->twin(), e, he1->edge(), he1->face());
    he2->setNeighbors(he7, he2->twin(), he2->vertex(), he2->edge(), he2->face());
    he3->setNeighbors(he9, he3->twin(), he3->vertex(), he3->edge(), face4);
    he4->setNeighbors(he12, he4->twin(), he4->vertex(), he4->edge(), he4->face());
    he5->setNeighbors(he11, he5->twin(), he5->vertex(), he5->edge(), face3);
    he6->setNeighbors(he4, he6->twin(), he6->vertex(), he6->edge(), he6->face());
    he7->setNeighbors(he1, he8, a, e1, face1);
    he8->setNeighbors(he3, he7, e, e1, face4);
    he9->setNeighbors(he8, he10, b, e2, face4);
    he10->setNeighbors(he5, he9, e, e2, face3);
    he11->setNeighbors(he10, he12, d, e3, face3);
    he12->setNeighbors(he6, he11, e, e3, face2);

    a->halfedge() = he3;
    b->halfedge() = he5;
    c->halfedge() = he2;
    d->halfedge() = he6;
    e->halfedge() = he1;

    face1->halfedge() = he1;
    face2->halfedge() = he4;
    face3->halfedge() = he5;
    face4->halfedge() = he3;

    e0->halfedge() = he1;
    e0->isNew = false;
    e1->halfedge() = he8;
    e1->isNew = true;
    e2->halfedge() = he10;
    e2->isNew = false;
    e3->halfedge() = he12;
    e3->isNew = true;





      // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return e;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.
    EdgeIter e;
    EdgeIter nextEdge;
    EdgeIter ogEdges;
    int ogEdge;
    VertexIter vLoop;
    VertexIter nextVertex;
    VertexIter v1;
    VertexIter v2;
    VertexIter a;
    VertexIter b;
    VertexIter c;
    VertexIter d;
    VertexIter newVertex;
    Vector3D sum;
    HalfedgeIter sumHelp;
    HalfedgeIter startH;
    double u;
    int counter;
    int tot;

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
//    e = mesh.edgesBegin();
//    while (e != mesh.edgesEnd()) {
//        nextEdge = e;
//        nextEdge++;
//
//        sum = Vector3D(0, 0, 0);
//        v1 = e->halfedge()->vertex();
//
//        sumHelp = e->halfedge();
//        startH = sumHelp;
//        do {
//            sum += sumHelp->next()->vertex()->position;
//            sumHelp = sumHelp->next()->next()->twin();
//        } while (startH != sumHelp);
//
//
//        if (v1->degree() == 3) {
//            u = 3.0 / 16.0;
//        } else {
//            u = 3.0 / (8.0 * double(v1->degree()));
//        }
//        //cout << v1->degree() << "\n";
//        v1->newPosition = (1.0 - double(v1->degree()) * u) * v1->position + u * sum;
//        //cout << v1->position << " " << v1->newPosition << "\n";
//
//        v1->isNew = false;
//        e = nextEdge;
//    }
    vLoop = mesh.verticesBegin();
    while (vLoop != mesh.verticesEnd()) {
        nextVertex = vLoop;
        nextVertex++;

        sum = Vector3D(0, 0, 0);
        v1 = vLoop;

        sumHelp = v1->halfedge();
        startH = sumHelp;
        do {
            sum += sumHelp->next()->vertex()->position;
            sumHelp = sumHelp->next()->next()->twin();
        } while (startH != sumHelp);


        if (v1->degree() == 3) {
            u = 3.0 / 16.0;
        } else {
            u = 3.0 / (8.0 * double(v1->degree()));
        }
        //cout << v1->degree() << "\n";
        v1->newPosition = (1.0 - double(v1->degree()) * u) * v1->position + u * sum;
        //cout << v1->position << " " << v1->newPosition << "\n";

        v1->isNew = false;
        vLoop = nextVertex;
    }
    tot = mesh.nVertices();

    e = mesh.edgesBegin();
    while (e != mesh.edgesEnd()) {
        // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        nextEdge = e;
        nextEdge++;
        //if (!e->isBoundary()) {
            a = e->halfedge()->vertex();
            b = e->halfedge()->twin()->vertex();
            c = e->halfedge()->twin()->next()->next()->vertex();
            d = e->halfedge()->next()->next()->vertex();
            e->newPosition = 3.0 / 8.0 * (a->position + b->position) + 1.0 / 8.0 * (c->position + d->position);
        //}
        e->isNew = false;
        e = nextEdge;
    }

    ogEdge = mesh.nEdges();
    // ogEdges = mesh.edgesEnd();
    e = mesh.edgesBegin();
    for (int i = 0; i < ogEdge; i++) {
    //while (e != ogEdges) {
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
        nextEdge = e;
        nextEdge++;
        //if (!e->isNew) {
            mesh.splitEdge(e);
        //}

        e = nextEdge;
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    e = mesh.edgesBegin();
    while (e != mesh.edgesEnd()) {
        nextEdge = e;
        nextEdge++;

        if (e->isNew) {
            v1 = e->halfedge()->vertex();
            v2 = e->halfedge()->twin()->vertex();

            if ((!(v1->isNew) && v2->isNew) || (v1->isNew && !(v2->isNew))) {
                mesh.flipEdge(e);
            }
        }

        e = nextEdge;
    }

    counter = 0;
    vLoop = mesh.verticesBegin();
    while (vLoop != mesh.verticesEnd()) {
    // 5. Copy the new vertex positions into final Vertex::position.
        nextVertex = vLoop;
        nextVertex++;
        //cout << vLoop->position << " " << vLoop->newPosition << "\n";
        vLoop->position = vLoop->newPosition;
        //out << vLoop->degree() << "\n";
        if (vLoop->newPosition.x == 0) {
            cout << counter << "\n";
        }
        vLoop->newPosition = Vector3D(0,0,0);
        vLoop = nextVertex;
        counter++;
    }
      //cout << "total: " << tot << "\n";

  }
}
