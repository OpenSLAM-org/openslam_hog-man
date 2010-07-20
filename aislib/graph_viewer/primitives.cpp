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
