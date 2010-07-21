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

#include "vrml_output.h"

#include "axes.h"
#include <aislib/stuff/macros.h>
#include <qglviewer/vec.h>
#include <qglviewer/quaternion.h>

#include <list>
#include <cmath>
#include <stack>
#include <stdexcept>
#include <limits>
using namespace std;
//using namespace vrml;

vrml_color_t g_vrml_color = { {0, 0, 0, 1} };
vrml::Matrix4x4 g_vrml_modelview = vrml::Matrix4x4::identity();

static stack<vrml::Matrix4x4> s_matrix_stack;

void vrmlColor3f(float r, float g, float b)
{
  vrmlColor4f(r, g, b, 1.f);
}

void vrmlColor4f(float r, float g, float b, float alpha)
{
  g_vrml_color.r = r;
  g_vrml_color.g = g;
  g_vrml_color.b = b;
  g_vrml_color.alpha = alpha;
}

void vrmlRotatef(float phi, float x, float y, float z)
{
  vrml::Matrix4x4 rot = vrml::Matrix4x4::rotate(DEG2RAD(phi), x, y, z);
  g_vrml_modelview = g_vrml_modelview * rot;
}

void vrmlTranslatef(float dx, float dy, float dz)
{
  vrml::Matrix4x4 trans = vrml::Matrix4x4::translate(dx, dy, dz);
  g_vrml_modelview = g_vrml_modelview * trans;
}

void vrmlMultMatrix(const vrml::Matrix4x4& m)
{
  g_vrml_modelview = g_vrml_modelview * m;
}

void vrmlScalef(float sx, float sy, float sz)
{
  std::cerr << "WARNING " << __PRETTY_FUNCTION__ << " not working right now" << std::endl;
  vrml::Matrix4x4 scale = vrml::Matrix4x4::scale(sx, sy, sz);
  g_vrml_modelview = g_vrml_modelview * scale;
}

void vrmlPushMatrix()
{
  s_matrix_stack.push(g_vrml_modelview);
}

void vrmlPopMatrix()
{
  if (s_matrix_stack.size() == 0) {
    throw runtime_error("called vrmlPopMatrix() on an empty matrix stack");
  } else {
    g_vrml_modelview = s_matrix_stack.top();
    s_matrix_stack.pop();
  }
}

void vrmlLoadIdentity()
{
  g_vrml_modelview = vrml::Matrix4x4::identity();
}

// helper functions
void writeCurrentHead(std::ostream& os, bool allColor)
{
  vrml::Vector4 origin(0, 0, 0, 1);
  vrml::Vector4 trans = g_vrml_modelview * origin;
  double dx = trans[0] / trans[3];
  double dy = trans[1] / trans[3];
  double dz = trans[2] / trans[3];

  // TODO handle scale inside the matrix?
  double rot[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      rot[i][j] = g_vrml_modelview[i][j];
  qglviewer::Quaternion quat;
  quat.setFromRotationMatrix(rot);
  qglviewer::Vec axis;
  float angle;
  quat.getAxisAngle(axis, angle);

  os
    << "Transform {\n"
    << "  rotation " << axis[0] << " " << axis[1] << " " << axis[2] << " " << angle << "\n"
    << "  translation " << dx << " " << dy << " " << dz << "\n"
    << "  children [\n"
    << "  Shape {\n"
    << "    appearance Appearance {\n"
    << "      material Material {\n"
    << "        diffuseColor " << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n"
    << "        ambientIntensity 0.2\n";
  if (allColor) {
    os
      << "        emissiveColor " << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n"
      << "        specularColor " << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n";
  } else {
    os
      << "        emissiveColor 0 0 0\n"
      << "        specularColor 0 0 0\n";
  }
  os
    << "        shininess 0.2\n"
    << "        transparency " << 1.0 - g_vrml_color.alpha << "\n"
    << "      }\n"
    << "    }\n"
    << "    geometry ";
}

void writeCurrentFoot(std::ostream& os)
{
  os
    << "  }\n"
    << "  ]\n"
    << "}\n";
}

void writeScaleHead(std::ostream& os, double sx, double sy, double sz)
{
  os
    << "Transform {\n"
    << "  scale " << sx << " " << sy << " " << sz << "\n"
    << "  children [\n"
    << "  Shape {\n"
    << "    appearance Appearance {\n"
    << "      material Material {\n"
    << "        diffuseColor " << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n"
    << "        ambientIntensity 0.2\n"
    << "        emissiveColor 0 0 0\n"
    << "        specularColor 0 0 0\n"
    << "        shininess 0.2\n"
    << "        transparency " << 1.0 - g_vrml_color.alpha << "\n"
    << "      }\n"
    << "    }\n"
    << "    geometry ";
}

void writeScaleFoot(std::ostream& os)
{
  os
    << "  }\n"
    << "  ]\n"
    << "}\n";
}

void writeBox(std::ostream& os, double l, double w, double h)
{
  writeCurrentHead(os);
  os
    << "Box {\n"
    << "        size " << l << " " << w << " " << h << "\n"
    << "     }\n";
  writeCurrentFoot(os);
}

void writeSphere(std::ostream& os, double radius)
{
  writeCurrentHead(os);
  os
    << "Sphere {\n"
    << "        radius " << radius << "\n"
    << "     }\n";
  writeCurrentFoot(os);
}

void writeEllipsoid(std::ostream& os, double r1, double r2, double r3)
{
  writeCurrentHead(os);
  writeScaleHead(os, r1, r2, r3);
  os
    << "Sphere {\n"
    << "        radius 1.0\n"
    << "     }\n";
  writeScaleFoot(os);
  writeCurrentFoot(os);
}

void writeCone(std::ostream& os, double radius, double height)
{
  writeCurrentHead(os);
  os
    << "Cone {\n"
    << "        bottomRadius " << radius << "\n"
    << "        height " << height << "\n"
    << "     }\n";
  writeCurrentFoot(os);
}

void writeCylinder(std::ostream& os, double radius, double height)
{
  writeCurrentHead(os);
  os
    << "Cylinder {\n"
    << "        radius " << radius << "\n"
    << "        height " << height << "\n"
    << "     }\n";
  writeCurrentFoot(os);
}

void writePyramid(std::ostream& os, double length, double height)
{
  (void) length; (void) height; // avoid warnings
  // TODO maybe use an indexedFaceSet ???
  os << "# writePyramid not implemented atm\n";
}

void writeVrmlFileHeader(std::ostream& os)
{
  os << "#VRML V2.0 utf8\n";
}

void writePlane(std::ostream& os, double l, double w)
{
  list<vrml_point_t> plane; // all the points
  plane.push_back(vrml_point_t(-l*0.5,  w*0.5, 0.));
  plane.push_back(vrml_point_t( l*0.5,  w*0.5, 0.));
  plane.push_back(vrml_point_t( l*0.5, -w*0.5, 0.));
  plane.push_back(vrml_point_t(-l*0.5, -w*0.5, 0.));
  writeQuads(os, plane.begin(), plane.end());
}

void writeSlice(std::ostream& os, double radius, double height, double fov, int slices_per_circle)
{
  double fov_rad = fov/180.*M_PI; // convert to rad
  int num_slices = int(slices_per_circle * (fov_rad / (2*M_PI))) + 1;
  double angle_step = fov_rad / num_slices;

  double height_half = height * 0.5f;
  double lower_z = -height_half;
  double upper_z =  height_half;

  double last_x = std::cos(-fov_rad * 0.5) * radius;
  double last_y = std::sin(-fov_rad * 0.5) * radius;


  vrmlPushMatrix();
  list<vrml_point_t> triangles; // all the points
  triangles.push_back(vrml_point_t(0.f, 0.f, upper_z));
  triangles.push_back(vrml_point_t(0.f, 0.f, lower_z));
  triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
  triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
  triangles.push_back(vrml_point_t(last_x, last_y, lower_z));
  triangles.push_back(vrml_point_t(0.f, 0.f, lower_z));

  double start_angle = -0.5*fov_rad + angle_step;
  double angle       = start_angle;
  for (int i = 0; i < num_slices; ++i) {
    double x = std::cos(angle) * radius;
    double y = std::sin(angle) * radius;

    // lower triangle
    triangles.push_back(vrml_point_t(0.f, 0.f, lower_z));
    triangles.push_back(vrml_point_t(x, y, lower_z));
    triangles.push_back(vrml_point_t(last_x, last_y, lower_z));
    // upper
    triangles.push_back(vrml_point_t(0.f, 0.f, upper_z));
    triangles.push_back(vrml_point_t(x, y, upper_z));
    triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
    //front rectangle (we use two triangles)
    triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
    triangles.push_back(vrml_point_t(last_x, last_y, lower_z));
    triangles.push_back(vrml_point_t(x, y, upper_z));
    triangles.push_back(vrml_point_t(x, y, upper_z));
    triangles.push_back(vrml_point_t(x, y, lower_z));
    triangles.push_back(vrml_point_t(last_x, last_y, lower_z));

    last_x = x;
    last_y = y;
    angle += angle_step;
  }

  triangles.push_back(vrml_point_t(0.f, 0.f, upper_z));
  triangles.push_back(vrml_point_t(0.f, 0.f, lower_z));
  triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
  triangles.push_back(vrml_point_t(last_x, last_y, upper_z));
  triangles.push_back(vrml_point_t(last_x, last_y, lower_z));
  triangles.push_back(vrml_point_t(0.f, 0.f, lower_z));

  // now we can write all the triangles
  writeTriangles(os, triangles.begin(), triangles.end());
  vrmlPopMatrix();
}

void writeAxes(std::ostream&os, double len)
{
  writeCurrentHead(os);
  os << "#coordinate axes " << __func__ << std::endl;
  os << Axes(len) << endl;
  writeCurrentFoot(os);
}

void vrmlHeadTranslation(qglviewer::Vec& p )
{
  vrml::Vector4 origin(0, 0, 0, 1);
  vrml::Vector4 trans = g_vrml_modelview * origin;
  p[0] = trans[0] / trans[3];
  p[1] = trans[1] / trans[3];
  p[2] = trans[2] / trans[3];
}

void vrmlHeadRotation(qglviewer::Quaternion& quat )
{
  double rot[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      rot[i][j] = g_vrml_modelview[i][j];

  quat.setFromRotationMatrix(rot);
}

std::ostream& operator<<(std::ostream& os, const vrml_point_t& p)
{
  os << p[0] << " " << p[1] << " " << p[2];
  return os;
}

void writePoseBox(std::ostream& os)
{
  vrmlPushMatrix();
  vrmlPushMatrix();
  vrmlTranslatef(-0.5*0.5,0.5*0.25,0);
  vrmlColor3f(1.0, 0.3, 0.3);
  writeBox(os, 1*0.5, 0.25, 0.5);
  vrmlPopMatrix();

  vrmlPushMatrix();
  vrmlTranslatef(-0.5*0.5,-0.5*0.25,0);
  vrmlColor3f(1.0, 0.1, 0.1);
  writeBox(os, 1*0.5, 0.25, 0.5);
  vrmlPopMatrix();

  vrmlPushMatrix();
  vrmlTranslatef(0.5*0.5,0.5*0.25,0);
  vrmlColor3f(0.3, 0.3, 1.0);
  writeBox(os, 1*0.5, 0.25, 0.5);
  vrmlPopMatrix();

  vrmlPushMatrix();
  vrmlTranslatef(+0.5*0.5,-0.5*0.25,0);
  vrmlColor3f(0.1, 0.1, 1.);
  writeBox(os, 1*0.5, 0.25, 0.5);
  vrmlPopMatrix();
  vrmlPopMatrix();
}

void writeStart(std::ostream& os)
{
  os << "Transform {\n"
    << "  translation 0 0 0" << std::endl
    << "  children [" << std::endl;
}

void writeEnd(std::ostream& os)
{
  os << "  ]" << std::endl
    << "}" << std::endl;
}
