// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "hogman_slam_interface.h"

#include "output.h"

#include <iostream>
using namespace std;

namespace AISNavigation {

static char buffer[10000]; // that should be more than enough

HogmanSlamInterface::HogmanSlamInterface() :
  _optimizer(0), _optimizer3D(0), _firstOptimization(true), _nodesAdded(0), _updateGraphEachN(10),
  _initSolverDone(false)
{
}

bool HogmanSlamInterface::addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values)
{
  // allocating the desired solver
  if (! _initSolverDone) {
    _initSolverDone = true;
    int numLevels = 2;
    int nodeDistance = 5;
    cerr << __PRETTY_FUNCTION__ << " dimension " << dimension << " " << values.size() << endl;
    if (dimension == 3) {
      _optimizer = new HCholOptimizer2D(numLevels, nodeDistance);
    } else {
      cerr << "Allocating 3D optimizer" << endl;
      _optimizer3D = new HCholOptimizer3D(numLevels, nodeDistance);
    }
  }

  // we add the node when we are asked to add the according edge
  (void) tag;
  (void) id;
  (void) values;
  return true;
}

bool HogmanSlamInterface::addEdge(const std::string& tag, int id, int dimension, int v1Id, int v2Id, const std::vector<double>& measurement, const std::vector<double>& information)
{
  (void) tag;
  (void) id;

  if (dimension == 3) {

    Transformation2 transf(Vector2(measurement[0], measurement[1]), Angle(measurement[2]));
    Matrix3 infMat;
    int idx = 0;
    for (int r = 0; r < 3; ++r)
      for (int c = r; c < 3; ++c, ++idx) {
        assert(idx < (int)information.size());
        infMat[r][c] = infMat[c][r] = information[idx];
      }

    static Transformation2 zeroTransf;
    static Matrix3 I3 = Matrix3::eye(1.);

    PoseGraph2D::Vertex* v1=_optimizer->vertex(v1Id);
    if (! v1){
      v1=_optimizer->addVertex(v1Id, zeroTransf, I3);
      assert(v1);
      _nodesAdded++;
    }

    PoseGraph2D::Vertex* v2=_optimizer->vertex(v2Id);
    if (! v2){
      v2=_optimizer->addVertex(v2Id, zeroTransf, I3);
      assert(v2);
      _nodesAdded++;
    }

    _optimizer->addEdge(v1, v2, transf, infMat);

  }
  else if (dimension == 6) {

    Vector6 p;
    for (int i = 0; i < 6; ++i)
      p[i] = measurement[i];
    // TODO verify that our conversion from Euler angles is the same as given in the example code on the webpage
    Transformation3 transf = Transformation3::fromVector(p);
    Matrix6 infMat;
    int idx = 0;
    for (int r = 0; r < 6; ++r)
      for (int c = r; c < 6; ++c, ++idx) {
        assert(idx < (int)information.size());
        infMat[r][c] = infMat[c][r] = information[idx];
      }

    static Transformation3 zeroTransf;
    static Matrix6 I6 = Matrix6::eye(1.);

    PoseGraph3D::Vertex* v1=_optimizer3D->vertex(v1Id);
    if (! v1){
      v1=_optimizer3D->addVertex(v1Id, zeroTransf, I6);
      assert(v1);
      _nodesAdded++;
    }

    PoseGraph3D::Vertex* v2=_optimizer3D->vertex(v2Id);
    if (! v2){
      v2=_optimizer3D->addVertex(v2Id, zeroTransf, I6);
      assert(v2);
      _nodesAdded++;
    }

    _optimizer3D->addEdge(v1, v2, transf, infMat);

  }
  else {
    cerr << __PRETTY_FUNCTION__ << " not implemented for this dimension" << endl;
    return false;
  }

  return true;
}

bool HogmanSlamInterface::fixNode(const std::vector<int>& nodes)
{
  // has to be implemented
  // Hog-MAN currently keeps the vertex with the smallest id fixed
  // which should be okay according to the slameval website
  (void) nodes;
  return true;
}

bool HogmanSlamInterface::queryState(const std::vector<int>& nodes)
{
  cout << "BEGIN" << endl;

  if (_optimizer) { // 2D SLAM
    if (nodes.size() == 0) {
      for (Graph::VertexIDMap::const_iterator it = _optimizer->vertices().begin(); it != _optimizer->vertices().end(); it++){
        PoseGraph2D::Vertex* v = static_cast<PoseGraph2D::Vertex*>(it->second);
        printVertex(v);
      }
    } else {
      for (size_t i = 0; i < nodes.size(); ++i) {
        PoseGraph2D::Vertex* v = static_cast<PoseGraph2D::Vertex*>(_optimizer->vertex(nodes[i]));
        if (v)
          printVertex(v);
      }
    }
  }
  else if (_optimizer3D) { // 3D SLAM
    if (nodes.size() == 0) {
      for (Graph::VertexIDMap::const_iterator it = _optimizer3D->vertices().begin(); it != _optimizer3D->vertices().end(); it++){
        PoseGraph3D::Vertex* v = static_cast<PoseGraph3D::Vertex*>(it->second);
        printVertex(v);
      }
    } else {
      for (size_t i = 0; i < nodes.size(); ++i) {
        PoseGraph3D::Vertex* v = static_cast<PoseGraph3D::Vertex*>(_optimizer3D->vertex(nodes[i]));
        if (v)
          printVertex(v);
      }
    }
  }

  cout << "END" << endl << flush;

  return true;
}

bool HogmanSlamInterface::solveState()
{
  if (_nodesAdded >= _updateGraphEachN) {
    int iterations = 1;
    if (_optimizer) {
      _optimizer->optimize(iterations, true);
    } else if (_optimizer3D) {
      _optimizer3D->optimize(iterations, true);
    }
    _firstOptimization = false;
    _nodesAdded = 0;
  }
  return true;
}

bool HogmanSlamInterface::printVertex(PoseGraph2D::Vertex* v)
{
  char* s = buffer;
  memcpy(s, "VERTEX_XYT ", 11);
  s += 11;
  s += modp_itoa10(v->id(), s);
  *s++ = ' ';
  s += modp_dtoa(v->transformation.translation().x(), s, 6);
  *s++ = ' ';
  s += modp_dtoa(v->transformation.translation().y(), s, 6);
  *s++ = ' ';
  s += modp_dtoa(v->transformation.rotation().angle(), s, 6);
  *s++ = '\n';
  cout.write(buffer, s - buffer);
  return true;
}

bool HogmanSlamInterface::printVertex(PoseGraph3D::Vertex* v)
{
  char* s = buffer;
  memcpy(s, "VERTEX_XYZRPY ", 14);
  s += 14;
  s += modp_itoa10(v->id(), s);
  *s++ = ' ';
  // TODO verify that our conversion from Euler angles is the same as given in the example code on the webpage
  Vector6 p = v->transformation.toVector();
  for (int i = 0; i < p.size(); ++i) {
    s += modp_dtoa(p[i], s, 6);
    *s++ = ' ';
  }
  *s++ = '\n';
  cout.write(buffer, s - buffer);
  return true;
}

} // end namespace
