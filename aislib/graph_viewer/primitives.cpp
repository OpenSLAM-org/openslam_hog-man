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

#include "primitives.h"
#include <qglviewer/qglviewer.h>
#include <cstdlib>

GLUquadricObj* g_quadratic = NULL;

void freePrimitives()
{
  if (g_quadratic) {
    gluDeleteQuadric(g_quadratic);
    g_quadratic = NULL;
  }
}

void initPrimitives()
{
  if (!g_quadratic) {
    g_quadratic = gluNewQuadric();              // Create A Pointer To The Quadric Object ( NEW )
    gluQuadricNormals(g_quadratic, GLU_SMOOTH); // Create Smooth Normals ( NEW )

    atexit(freePrimitives);
  }
}

void drawAxis(float length)
{
  QGLViewer::drawAxis(length);
}

void drawArrow(float length, float radius, int nbSubdivisions)
{
  QGLViewer::drawArrow(length, radius, nbSubdivisions);
}

void drawGrid(float size, int nbSubdivisions)
{
  QGLViewer::drawGrid(size, nbSubdivisions);
}

void drawArrow2D(float len, float head_width, float head_len)
{
  glBegin(GL_LINES);
  glVertex2f(0, 0);
  glVertex2f(len, 0);
  glEnd();

  glNormal3f(0,0,1);
  glBegin(GL_TRIANGLES);
  glVertex2f(len, 0);
  glVertex2f(len - head_len,  0.5*head_width);
  glVertex2f(len - head_len, -0.5*head_width);
  glEnd();
}

void drawPoseBox()
{
  glPushMatrix();
  glScalef(0.5,1,1);
  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(-0.5,0.5,0);
  glColor3f(1.0, 0.3, 0.3);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(-0.5,-0.5,0);
  glColor3f(1.0, 0.1, 0.1);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(+0.5,0.5,0);
  glColor3f(0.3, 0.3, 1.0);
  drawBox(1, 1, 1);
  glPopMatrix();

  glPushMatrix();
  glScalef(1,0.25,0.5);
  glTranslatef(+0.5,-0.5,0);
  glColor3f(0.1, 0.1, 1.);
  drawBox(1, 1, 1);
  glPopMatrix();
  glPopMatrix();
}
