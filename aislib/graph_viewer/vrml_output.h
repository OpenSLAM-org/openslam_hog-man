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

#ifndef VRML_OUTPUT_H
#define VRML_OUTPUT_H

/** @addtogroup vrmloutput libvrmloutput **/
// @{

/**
 * \file vrmlOutput.h
 * \brief output things to a VRML 2.0 file
 *
 * This code implements a matrix stack, that can be regarded as a adaption
 * of the well known matrix stack of OpenGL. Additionally the current color
 * is also stored.
 * It can be used to write the Transforms of a vrml 2.0 file, so that you can draw
 * objects in vrml in a quite similar fashion compared to OpenGL
 */

// TODO implement an vrml equivalent for all the other translate rotate functions of OpenGL
// TODO needs implementation and testing of glScalef for the vrml matrix stack

#include "matrix4x4.h"
#include <aislib/stuff/macros.h>

// forward declarations
namespace qglviewer {
  class Vec;
  class Quaternion;
}

#include <iostream>
#include <iterator>
#include <iomanip>

// vrml color
/**
 * \brief vrml color
 */
typedef union {
  struct {
    float r; ///< red value
    float g; ///< green value
    float b; ///< blue value
    float alpha; ///< the alpha value
  }; ///< rgb color, each color value should be in [0..1]
  float color[4]; ///< rgb array
  float operator[] (int i) const {return color[i];}
  float& operator[] (int i) {return color[i];}
} vrml_color_t;

/**
 * \brief a 3d point
 */
union vrml_point_t {
  struct {
    double x; ///< x coordinate
    double y; ///< y coordinate
    double z; ///< z coordinate
  };
  double coords[3]; ///< coordinates array
  double operator[] (int i) const {return coords[i];}
  double& operator[] (int i) {return coords[i];}
  vrml_point_t() : x(0.), y(0.), z(0.) {}
  vrml_point_t(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};
std::ostream& operator<<(std::ostream& os, const vrml_point_t& p);

/**
 * \brief a 3d point with color
 */
struct vrml_colored_point_t {
  vrml_point_t point;
  vrml_color_t color;
  vrml_colored_point_t() {color[3] = 1.0;}
}; 

//! the current vrml color
extern vrml_color_t g_vrml_color;
//! the current transformation matrix
extern vrml::Matrix4x4 g_vrml_modelview;

/**
 * store current translation in p
 */
void vrmlHeadTranslation(qglviewer::Vec& p);

/**
 * current rotation as Quaternion
 */
void vrmlHeadRotation(qglviewer::Quaternion& quat);

/**
 * change the current color
 */
void vrmlColor3f(float r, float g, float b);

/**
 * change the current color
 */
void vrmlColor4f(float r, float g, float b, float alpha);

/**
 * rotate with an angle around the axis (x, y, z)
 * @param phi the rotation angle specified in degree (just like in OpenGL)
 * @param x, y, z the rotation axis
 */
void vrmlRotatef(float phi, float x, float y, float z);

/**
 * translate the current coordinate frame
 */
void vrmlTranslatef(float dx, float dy, float dz);

/**
 * translate the current coordinate frame
 */
void vrmlScalef(float sx, float sy, float sz);

/**
 * multiply the current matrix with the given matrix
 */
void vrmlMultMatrix(const vrml::Matrix4x4& m);

/**
 * push the current Matrix on the stack
 */
void vrmlPushMatrix();

/**
 * pop/restore the matrix from the stack.
 * Throws a runtime_error, iff called on an empty stack.
 */
void vrmlPopMatrix();

/**
 * Load the identity Matrix and replace the current modelview matrix
 */
void vrmlLoadIdentity();

/**
 * write the required vrml header to the stream.
 * Otherwise the file is not a valid vrml file.
 */
void writeVrmlFileHeader(std::ostream& os);

/**
 * call first to init the output file
 */
void writeStart(std::ostream& os);

/**
 * call to finish the writing
 */
void writeEnd(std::ostream& os);

/**
 * writes the current transform header and starts the shape + complete
 * Appearance node, so you just need to write the geometry node afterwards
 */
void writeCurrentHead(std::ostream& os, bool allColor = false);

/**
 * close the transform node.
 * Make sure to call this after you have written the geometry node.
 */
void writeCurrentFoot(std::ostream& os);

/**
 * write a scaling transformation to the stream, only the head, need to call foot function afterwards
 */
void writeScaleHead(std::ostream& os, double sx, double sy, double sz);

/**
 * write foot of scale transform
 */
void writeScaleFoot(std::ostream& os);

/**
 * write Box to a vrml file that is centered in the current coordinate frame
 * @param os the output stream
 * @param l length of the box (x dimension)
 * @param w width of the box (y dimension)
 * @param h height of the box (z dimension)
 */
void writeBox(std::ostream& os, double l, double w, double h);

/**
 * write a plane in x-y dimension with a height of zero
 * @param os the output stream
 * @param l length in x
 * @param w width in y
 */
void writePlane(std::ostream& os, double l, double w);

/**
 * write a sphere to a vrml file whose center is in the origin of the current coordinate frame
 * @param os the output stream
 * @param radius the radius of the sphere
 */
void writeSphere(std::ostream& os, double radius);

/**
 * write a ellipsoid to a vrml file whose center is in the origin of the current coordinate frame
 * @param os the output stream
 * @param r1 radius along x axis
 * @param r2 radius along y axis
 * @param r3 radius along z axis
 */
void writeEllipsoid(std::ostream& os, double r1, double r2, double r3);

/**
 * write a cone
 */
void writeCone(std::ostream& os, double radius, double height);

/**
 * write a closed cylinder
 * @param os the output stream
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 */
void writeCylinder(std::ostream& os, double radius, double height);

/**
 * write a pyramid
 */
void writePyramid(std::ostream& os, double length, double height);

/**
 * draw a slice of a cylinder (approximated with slices_per_circle triangles for the complete circle)
 * @param os the output stream
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 * @param fov the "fov" of the slice (om degree)
 * @param slices_per_circle the number of triangle used to approximate the fulle circle
 */
void writeSlice(std::ostream& os, double radius, double height, double fov, int slices_per_circle = 32);

/**
 * write axes of the current coodinate frame
 * @param os the output stream
 * @param len the length of the axes
 */
void writeAxes(std::ostream&os, double len=1.0);

/**
 * write an IndexedLineset.
 * line are assumed in the following way 0-1, 2-3, 3-4 (just like GL_LINES)
 * Limitation: They are all assumed to have the same color
 */
template <class InputIterator>
void writeLines(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "IndexedLineSet {\n"
     << "  color Color {\n"
     << "    color [\n"
     << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n"
     << "    ]\n"
     << "  }\n"
     << "  coord Coordinate {\n" << "    point [\n";
  unsigned int cnt = 0;
  for (InputIterator it = begin; it != end; ++it) {
    os << FIXED((*it)[0] << " " << (*it)[1] << " " << (*it)[2]) << ",\n";
    cnt++;
  }
  os << "    ]\n" << "  }\n";
  os << "  colorIndex [\n";
  for (unsigned int i = 0; i < cnt; i+=2) {
    os << "0 0 -1\n";
  }
  os << "\n  ]\n";
  os << "  coordIndex [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
    os << i++ << " -1\n";
  }
  os << "  ]\n"
     << "}\n";
  writeCurrentFoot(os);
}

/**
 * write a line strip
 * line is assumed in the following way 0-1-2-3-4 (just like GL_LINE_STRIP)
 * Limitation: They are all assumed to have the same color
 */
template <class InputIterator>
void writeLineStrip(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "IndexedLineSet {\n"
     << "  color Color {\n"
     << "    color [\n"
     << g_vrml_color.r << " " << g_vrml_color.g << " " << g_vrml_color.b << "\n"
     << "    ]\n"
     << "  }\n"
     << "  coord Coordinate {\n" << "    point [\n";
  unsigned int cnt = 0;
  for (InputIterator it = begin; it != end; ++it) {
    os << FIXED((*it)[0] << " " << (*it)[1] << " " << (*it)[2]) << ",\n";
    cnt++;
  }
  os << "    ]\n" << "  }\n";
  os << "  colorIndex [\n";
  for (unsigned int i = 0; i < cnt; i++)
    os << "0 ";
  os << "-1\n";
  os << "\n  ]\n";
  os << "  coordIndex [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
  }
  os << "-1\n";
  os << "  ]\n"
     << "}\n";
  writeCurrentFoot(os);
}

/**
 * write a list of triangles (like GL_TRIANGLES)
 * Limitation: They are all assumed to have the same color
 */
template <class InputIterator>
void writeTriangles(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "IndexedFaceSet {\n"
     << "  solid FALSE\n"
     << "  coord Coordinate {\n"
     << "    point [\n";
  unsigned int cnt = 0;
  for (InputIterator it = begin; it != end; ++it) {
    os << FIXED((*it)[0] << " " <<  (*it)[1] << " " << (*it)[2]) << std::endl;
    ++cnt;
  }
  os << "    ]\n"
     << "  }\n"
     << "  coordIndex  [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
    os << i++ << " ";
    os << i++ << " -1\n";
  }
  os << "  ]\n"
     << "}\n";
  writeCurrentFoot(os);
}

/**
 * write a list of quads (like GL_QUADS)
 * Limitation: They are all assumed to have the same color
 */
template <class InputIterator>
void writeQuads(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "IndexedFaceSet {\n"
     << "  solid FALSE\n"
     << "  coord Coordinate {\n"
     << "    point [\n";
  unsigned int cnt = 0;
  for (InputIterator it = begin; it != end; ++it) {
    os << FIXED((*it)[0] << " " <<  (*it)[1] << " "  << (*it)[2]) << "\n";
    ++cnt;
  }
  os << "    ]\n"
     << "  }\n"
     << "  coordIndex  [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
    os << i++ << " ";
    os << i++ << " ";
    os << i++ << " -1\n";
  }
  os << "  ]\n"
     << "}\n";
  writeCurrentFoot(os);
}

/**
 * write a PointSet.
 * Limitation: They are all assumed to have the same color
 */
template <class InputIterator>
void writePoints(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os, true);
  os << "PointSet {\n";
  os << "  coord Coordinate {\n";
  os << "    point [\n";
  std::streamsize oldPrec = os.precision();
  os << std::fixed << std::setprecision(3);
  for (InputIterator it = begin; it != end; ++it)
    os << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << "\n";
  os << std::resetiosflags(std::ios_base::floatfield);
  os.precision(oldPrec);
  os << "    ]\n";
  os << "  }\n";
  os << "}\n";
  writeCurrentFoot(os);
}

/**
 * write a colored PointSet. value_type for the InputIterator has to be a vrml_colored_point_t
 * or something similar, that provides color and point.
 */
template <class InputIterator>
void writeColoredPoints(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "PointSet {\n";
  os << " color Color {\n";
  os << "   color [\n";
  for (InputIterator it = begin; it != end; ++it)
    os << (*it).color[0] << " " << (*it).color[1] << " " << (*it).color[2] << "\n";
  os << "   ]\n";
  os << "  }\n";
  os << "  coord Coordinate {\n";
  os << "    point [\n";
  for (InputIterator it = begin; it != end; ++it)
    os << FIXED((*it).point[0] << " " << (*it).point[1] << " " << (*it).point[2]) << "\n";
  os << "    ]\n";
  os << "  }\n";
  os << "}\n";
  writeCurrentFoot(os);
}

/**
 * write an colored line set
 * line are assumed in the following way 0-1, 2-3, 3-4 (just like GL_LINES)
 * value_type for the InputIterator has to be a vrml_colored_point_t
 * or something similar, that provides color and point.
 */
template <class InputIterator>
void writeColoredLines(std::ostream& os, const InputIterator& begin, const InputIterator& end)
{
  writeCurrentHead(os);
  os << "IndexedLineSet {\n"
    << "  color Color {\n"
    << "    color [\n";
  unsigned int cnt = 0;
  for (InputIterator it = begin; it != end; ++it, ++cnt)
    os << (*it).color[0] << " " << (*it).color[1] << " " << (*it).color[2] << "\n";
  os << "    ]\n"
    << "  }\n"
    << "  coord Coordinate {\n" << "    point [\n";
  for (InputIterator it = begin; it != end; ++it)
    os << FIXED((*it).point[0] << " " << (*it).point[1] << " " << (*it).point[2]) << ",\n";
  os << "    ]\n" << "  }\n";
  os << "  colorIndex [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
    os << i++ << " -1\n";
  }
  os << "\n  ]\n";
  os << "  coordIndex [\n";
  for (unsigned int i = 0; i < cnt; ) {
    os << i++ << " ";
    os << i++ << " -1\n";
  }
  os << "  ]\n"
    << "}\n";
  writeCurrentFoot(os);
}

/**
 * set the current transformation matrix from another matrix
 */
template <class MatrixType>
void setVrmlTransform(const MatrixType& mat)
{
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      g_vrml_modelview[i][j] = mat[i][j];
}

/**
 * writes a box used to represent a 6d pose
 */
void writePoseBox(std::ostream& os);

// @}

#endif
