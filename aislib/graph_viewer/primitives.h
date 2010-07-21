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

#ifndef PRIMITIVES_H
#define PRIMITIVES_H

/** @addtogroup viewer libviewer **/
// @{

/** \file primitives.h
 * \brief draw primitives with OpenGL
 */

#include <qgl.h>
#include <cmath>

/**
 * Quadratic used in several drawing functions
 */
extern GLUquadricObj* g_quadratic;

/**
 * Call this function before using any of the draw functions
 */
void initPrimitives();

/**
 * draw a box that is centered in the current coordinate frame
 * @param l length of the box (x dimension)
 * @param w width of the box (y dimension)
 * @param h height of the box (z dimension)
 */
inline void drawBox(GLfloat l, GLfloat w, GLfloat h)
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;
  GLfloat sz = h*0.5f;

  glBegin(GL_QUADS);
  // bottom
  glNormal3f( 0.0f, 0.0f,-1.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, -sy, -sz);
  // top
  glNormal3f( 0.0f, 0.0f,1.0f);
  glVertex3f(-sx, -sy, sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // back
  glNormal3f(-1.0f, 0.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(-sx, sy, sz);
  glVertex3f(-sx, -sy, sz);
  // front
  glNormal3f( 1.0f, 0.0f, 0.0f);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(sx, -sy, sz);
  // left
  glNormal3f( 0.0f, -1.0f, 0.0f);
  glVertex3f(-sx, -sy, -sz);
  glVertex3f(sx, -sy, -sz);
  glVertex3f(sx, -sy, sz);
  glVertex3f(-sx, -sy, sz);
  //right
  glNormal3f( 0.0f, 1.0f, 0.0f);
  glVertex3f(-sx, sy, -sz);
  glVertex3f(sx, sy, -sz);
  glVertex3f(sx, sy, sz);
  glVertex3f(-sx, sy, sz);
  glEnd();
}

/**
 * draw a plane in x-y dimension with a height of zero
 * @param l length in x
 * @param w width in y
 */
inline void drawPlane(GLfloat l, GLfloat w)
{
  GLfloat sx = l*0.5f;
  GLfloat sy = w*0.5f;

  glBegin(GL_QUADS);
  glNormal3f( 0.0f, 0.0f, 1.0f);
  glVertex3f(-sx, -sy, 0.f);
  glVertex3f(-sx, sy, 0.f);
  glVertex3f(sx, sy, 0.f);
  glVertex3f(sx, -sy, 0.f);
  glEnd();
}

/**
 * draw a sphere whose center is in the origin of the current coordinate frame
 * @param radius the radius of the sphere
 */
inline void drawSphere(GLfloat radius)
{
  gluSphere(g_quadratic, radius, 32, 32);
}

/**
 * draw a ellipsoid whose center is in the origin of the current coordinate frame
 * @param r1 radius along x axis
 * @param r2 radius along y axis
 * @param r3 radius along z axis
 */
inline void drawEllipsoid(GLfloat r1, GLfloat r2, GLfloat r3)
{
  GLboolean hasNormalization = glIsEnabled(GL_NORMALIZE);
  if (!hasNormalization)
    glEnable(GL_NORMALIZE);
  glPushMatrix();
  glScalef(r1, r2, r3);
  gluSphere(g_quadratic, 1.0, 32, 32);
  glPopMatrix();
  if (!hasNormalization)
    glDisable(GL_NORMALIZE);
}

/**
 * draw a cone
 */
inline void drawCone(GLfloat radius, GLfloat height)
{
  glPushMatrix();
  glRotatef(-90.f, 1.f, 0.f, 0.f);
  glTranslatef(0, 0, - height/2.0);
  gluCylinder(g_quadratic, radius, 0.f, height, 32, 1);
  gluDisk(g_quadratic, 0, radius, 32, 1);
  glPopMatrix();
}

/**
 * draw a (closed) cylinder
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 */
inline void drawCylinder(GLfloat radius, GLfloat height)
{
  glPushMatrix();
  glRotatef(-90, 1.f, 0.f, 0.f);
  glTranslatef(0, 0, + height/2.0);
  gluDisk(g_quadratic, 0, radius, 32, 1);
  glTranslatef(0, 0, - height);
  gluCylinder(g_quadratic, radius, radius, height, 32, 1);
  glRotatef(180, 1.f, 0.f, 0.f);
  gluDisk(g_quadratic, 0, radius, 32, 1);
  glPopMatrix();
}

/**
 * draw a pyramid
 */
inline void drawPyramid(GLfloat length, GLfloat height)
{
  glPushMatrix();
  glTranslatef(0, 0, - height/2.0);
  glRotatef(45, 0.f, 0.f, 1.f);
  gluCylinder(g_quadratic, length, 0.f, height, 32, 1);
  gluDisk(g_quadratic, 0, length, 32, 1);
  glPopMatrix();
}

/**
 * draw a range ring
 * @param range the range (radius) of the partial ring
 * @param fov Field Of View of the range sensor
 * @param range_width specify how thick the ring should be drawn
 */
inline void drawRangeRing(GLfloat range, GLfloat fov, GLfloat range_width = 0.05)
{
  glPushMatrix();
  glRotatef((fov/2.0f) - 90, 0.f, 0.f, 1.f);
  gluPartialDisk(g_quadratic, range, range + range_width, 32, 1, 0.f, fov);
  glPopMatrix();
}

/**
 * draw a slice of a cylinder (approximated with slices_per_circle triangles for the complete circle)
 * @param radius the radius of the cylinder
 * @param height the height of the cylinder
 * @param fov the "fov" of the slice (om degree)
 * @param slices_per_circle the number of triangle used to approximate the fulle circle
 */
inline void drawSlice(GLfloat radius, GLfloat height, GLfloat fov, int slices_per_circle = 32)
{
  double fov_rad = fov/180.*M_PI; // convert to rad
  int num_slices = int(slices_per_circle * (fov_rad / (2*M_PI))) + 1;
  double angle_step = fov_rad / num_slices;
  double angle_step_half = angle_step * 0.5;

  GLfloat height_half = height * 0.5f;
  GLfloat lower_z = -height_half;
  GLfloat upper_z =  height_half;

  GLfloat last_x = std::cos(-fov_rad * 0.5) * radius;
  GLfloat last_y = std::sin(-fov_rad * 0.5) * radius;

  glPushMatrix();
  glBegin(GL_TRIANGLES);
  glNormal3f(std::sin(-fov_rad * 0.5), -std::cos(-fov_rad * 0.5), -0.f);
  glVertex3f(0.f, 0.f, upper_z);
  glVertex3f(0.f, 0.f, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.f, 0.f, lower_z);

  double start_angle = -0.5*fov_rad + angle_step;
  double angle       = start_angle;
  for (int i = 0; i < num_slices; ++i) {
    GLfloat x = std::cos(angle) * radius;
    GLfloat y = std::sin(angle) * radius;
    GLfloat front_normal_x = std::cos(angle + angle_step_half);
    GLfloat front_normal_y = std::sin(angle + angle_step_half);

    // lower triangle
    glNormal3f(0.f, 0.f, -1.f);
    glVertex3f(0.f, 0.f, lower_z);
    glVertex3f(x, y, lower_z);
    glVertex3f(last_x, last_y, lower_z);
    // upper
    glNormal3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.f, upper_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(last_x, last_y, upper_z);
    //front rectangle (we use two triangles)
    glNormal3f(front_normal_x, front_normal_y, 0.f);
    glVertex3f(last_x, last_y, upper_z);
    glVertex3f(last_x, last_y, lower_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(x, y, upper_z);
    glVertex3f(x, y, lower_z);
    glVertex3f(last_x, last_y, lower_z);

    last_x = x;
    last_y = y;
    angle += angle_step;
  }

  glNormal3f(-std::sin(fov_rad * 0.5), std::cos(fov_rad * 0.5), -0.f);
  glVertex3f(0.f, 0.f, upper_z);
  glVertex3f(0.f, 0.f, lower_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, upper_z);
  glVertex3f(last_x, last_y, lower_z);
  glVertex3f(0.f, 0.f, lower_z);

  glEnd();
  glPopMatrix();
}

/**
 * draws a box used to represent a 6d pose
 */
void drawPoseBox();

/**
 * Draws a 3D arrow along the positive Z axis.
 */
void drawArrow(float length=1.0f, float radius=-1.0f, int nbSubdivisions=12);

/**
 * draw a 2D arrow along the x axis with the given len
 */
void drawArrow2D(float len, float head_width, float head_len);

/**
 * Draws an XYZ axis, with a given len (default is 1.0).
 * 
 * The axis position and orientation matches the current modelView matrix state: three arrows (red,
 * green and blue) of length \p length are drawn along the positive X, Y and Z directions.
 */
void drawAxis(float length = 1.f);

/**
 * Draws a grid in the XY plane, centered on (0,0,0) (defined in the current coordinate system).
 */
void drawGrid(float size=1.0f, int nbSubdivisions=10);

// @}




#endif
