// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
//
// This file is part of HOG-Man.
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with HOG-Man.  If not, see <http://www.gnu.org/licenses/>.

#include "pose_graph_vis3d.h"
#include "axes.h"
#include "vrml_output.h"
#include "primitives.h"

#define NUM_DRAW_LISTS (2)

namespace AISNavigation {

using namespace std;

PoseGraph3DVis::PoseGraph3DVis() :
  _graph(0), _hirarchy(0),
  _updateDrawList(false), _useDrawList(false), _drawList(0), _listAllocated(false)
{
  _colors.push_back(Color(1, 0, 0)); 
  _colors.push_back(Color(0, 1, 0)); 
  _colors.push_back(Color(0, 0, 1)); 
  _colors.push_back(Color(1, 1, 0)); 
  _colors.push_back(Color(1, 0, 1)); 
  _colors.push_back(Color(0, 1, 1)); 
  for (int i = 0; i < 100; ++i) { // that should be more than enough
    double r,g,b;
    GET_COLOR(i, 100, r, g, b);
    _colors.push_back(Color(r, g, b));
  }
  _drawOptions.resize(NUM_DRAW_LISTS, true);
}

PoseGraph3DVis::~PoseGraph3DVis()
{
  if (_listAllocated) // clean the draw list if there was one allocated
    glDeleteLists(_drawList, NUM_DRAW_LISTS);
}

void PoseGraph3DVis::writeVrml(std::ostream& os) const
{
  if (!_graph)
    return;
  writeVrmlFileHeader(os);
  os << Axes(1.0) << endl;
  writeStart(os); 
  vrmlLoadIdentity();
  if (_drawOptions[0])
    writeGraph(os);
  if (_drawOptions[1])
    writeHirarchy(os);
  writeEnd(os);
}

void PoseGraph3DVis::draw() const
{
  if (!_graph)
    return;

  // turn off the light for the edges
  GLboolean hasLight = glIsEnabled(GL_LIGHTING);
  if (hasLight)
    glDisable(GL_LIGHTING);

  if (_useDrawList) { // handle update and creation of the draw list
    if (!_listAllocated) {
      _drawList = glGenLists(NUM_DRAW_LISTS);
      _listAllocated = true;
      _updateDrawList = true; // force update
    }
    if (_updateDrawList) {
      _updateDrawList = false;
      // graph
      if (_drawOptions[0])
        glNewList(_drawList, GL_COMPILE_AND_EXECUTE);
      else
        glNewList(_drawList, GL_COMPILE);
      drawGraph(); // draw into the list
      glEndList();

      // hirarchy
      if (_drawOptions[1])
        glNewList(_drawList+1, GL_COMPILE_AND_EXECUTE);
      else
        glNewList(_drawList+1, GL_COMPILE);
      drawHirarchy();
      glEndList();
    } else {
      for (GLuint l = 0; l < NUM_DRAW_LISTS; ++l)
        if (_drawOptions[l])
          glCallList(_drawList + l);
    }
  } else {
    if (_drawOptions[0])
      drawGraph(); // normal draw of the graph
    if (_drawOptions[1])
      drawHirarchy();
  }

  if (hasLight)
    glEnable(GL_LIGHTING);
}

void PoseGraph3DVis::setGraph(PoseGraph3D* graph)
{
  _graph = graph;
  if (_useDrawList)
    _updateDrawList = true;
}

void PoseGraph3DVis::drawGraph() const
{
  // display the nodes
  for (PoseGraph3D::VertexIDMap::const_iterator it = _graph->vertices().begin();
      it != _graph->vertices().end(); ++it) {
    const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>(it->second);
    const Vector3& t = vi->transformation.translation();
    AxisAngle axis(vi->transformation.rotation());
    double angle = axis.norm();
    axis.normalize();
    glPushMatrix();
    glTranslatef(t.x(), t.y(), t.z());
    glRotatef(RAD2DEG(angle), axis.x(), axis.y(), axis.z());
    drawPoseBox();
    glPopMatrix();
  }

  // display the edges
  glColor3f(0.7f, 0.7f, 0.7f);
  glBegin(GL_LINES);
  for (PoseGraph3D::EdgeSet::const_iterator it = _graph->edges().begin();
      it != _graph->edges().end(); ++it) {
    const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>((*it)->from());
    const PoseGraph3D::Vertex* vj = static_cast<PoseGraph3D::Vertex*>((*it)->to());
    const Vector3& t1 = vi->transformation.translation();
    const Vector3& t2 = vj->transformation.translation();
    glVertex3f(t1.x(), t1.y(), t1.z());
    glVertex3f(t2.x(), t2.y(), t2.z());
  }
  glEnd();
}

void PoseGraph3DVis::setHirarchy(std::vector<PoseGraph3DVis::HEdgeVector>* hirarchy)
{
  _hirarchy = hirarchy;
}

void PoseGraph3DVis::drawHirarchy() const
{
  if (!_hirarchy || !_graph)
    return;

  int l = 0;
  glBegin(GL_LINES);
  for (std::vector<HEdgeVector>::const_iterator level = _hirarchy->begin(); level != _hirarchy->end(); ++level, ++l) {
    const HEdgeVector& hEdges = *level;
    glColor3f(_colors[l].r, _colors[l].g, _colors[l].b);
    for (HEdgeVector::const_iterator it = hEdges.begin(); it != hEdges.end(); ++it) {
      const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>(_graph->vertex(it->id1));
      const PoseGraph3D::Vertex* vj = static_cast<PoseGraph3D::Vertex*>(_graph->vertex(it->id2));
      const Vector3& t1 = vi->transformation.translation();
      const Vector3& t2 = vj->transformation.translation();
      glVertex3f(t1.x(), t1.y(), t1.z());
      glVertex3f(t2.x(), t2.y(), t2.z());
    }
  }
  glEnd();
}

void PoseGraph3DVis::writeGraph(std::ostream& os) const
{
  os << "# PoseGraph3DVis \n";
  // display the nodes
  for (PoseGraph3D::VertexIDMap::const_iterator it = _graph->vertices().begin();
      it != _graph->vertices().end(); ++it) {
    const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>(it->second);
    const Vector3& t = vi->transformation.translation();
    AxisAngle axis(vi->transformation.rotation());
    double angle = axis.norm();
    axis.normalize();
    vrmlPushMatrix();
    vrmlTranslatef(t.x(), t.y(), t.z());
    vrmlRotatef(RAD2DEG(angle), axis.x(), axis.y(), axis.z());
    writePoseBox(os);
    vrmlPopMatrix();
  }

  // display the edges
  std::list<Vector3> line_points;
  vrmlColor3f(0.5f, 0.5f, 0.5f);
  for (PoseGraph3D::EdgeSet::const_iterator it = _graph->edges().begin();
      it != _graph->edges().end(); ++it) {
    const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>((*it)->from());
    const PoseGraph3D::Vertex* vj = static_cast<PoseGraph3D::Vertex*>((*it)->to());
    const Vector3& t1 = vi->transformation.translation();
    const Vector3& t2 = vj->transformation.translation();
    line_points.push_back(t1);
    line_points.push_back(t2);
  }
  writeLines(os, line_points.begin(), line_points.end());
  os << "# PoseGraph3DVis Nodes end\n";
}

void PoseGraph3DVis::writeHirarchy(std::ostream& os) const
{
  if (!_hirarchy || !_graph)
    return;

  int l = 0;
  std::list<Vector3> line_points;
  for (std::vector<HEdgeVector>::const_iterator level = _hirarchy->begin(); level != _hirarchy->end(); ++level, ++l) {
    const HEdgeVector& hEdges = *level;
    vrmlColor3f(_colors[l].r, _colors[l].g, _colors[l].b);
    for (HEdgeVector::const_iterator it = hEdges.begin(); it != hEdges.end(); ++it) {
      const PoseGraph3D::Vertex* vi = static_cast<PoseGraph3D::Vertex*>(_graph->vertex(it->id1));
      const PoseGraph3D::Vertex* vj = static_cast<PoseGraph3D::Vertex*>(_graph->vertex(it->id2));
      const Vector3& t1 = vi->transformation.translation();
      const Vector3& t2 = vj->transformation.translation();
      line_points.push_back(t1);
      line_points.push_back(t2);
    }
  }
  writeLines(os, line_points.begin(), line_points.end());
}

void PoseGraph3DVis::setDrawGraph(bool draw)
{
  _drawOptions[0] = draw;
}

void PoseGraph3DVis::setDrawHirarchy(bool draw)
{
  _drawOptions[1] = draw;
}

} // end namespace
