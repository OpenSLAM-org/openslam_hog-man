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

#include "matrix4x4.h"
#include <aislib/stuff/os_specific.h>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <cstring>
using namespace std;

namespace vrml {

Matrix4x4::Matrix4x4()
{
  memset(_data, 0, 16 * sizeof(double));
}

Matrix4x4::~Matrix4x4()
{
}

Matrix4x4::Matrix4x4(const Matrix4x4& other)
{
  copy(other);
}

void Matrix4x4::copy(const Matrix4x4& other)
{
  memcpy(_data, other._data, 16 * sizeof(double));
}

void Matrix4x4::makeDiag(double d)
{
  memset(_data, 0, 16 * sizeof(double));
  for (unsigned int i = 0; i < 4; ++i)
    (*this)[i][i] = d;
}

Matrix4x4& Matrix4x4::operator=(const Matrix4x4& other)
{
  if (this != &other){
    copy(other);
  }
  return *this;
}

Matrix4x4 Matrix4x4::operator* (const Matrix4x4& other) const
{
  Matrix4x4 prod;
  for (uint row = 0; row < 4; row++)
    for (uint col = 0; col < 4; col++){
      prod[row][col] = 0.;
      for (uint i=0; i < 4; i++)
        prod[row][col] += (*this)[row][i]*other[i][col];
    }

  return prod;
}

Matrix4x4 Matrix4x4::translate(double dx, double dy, double dz)
{
  Matrix4x4 trans;
  trans.makeDiag(1.0);
  trans[0][3] = dx;
  trans[1][3] = dy;
  trans[2][3] = dz;
  return trans;
}

Matrix4x4 Matrix4x4::rotate(double phi, double u, double v, double w)
{
  Matrix4x4 matrix;

  double rcos = cos(phi);
  double rsin = sin(phi);
  matrix[0][0] =      rcos + u*u*(1-rcos);
  matrix[1][0] =  w * rsin + v*u*(1-rcos);
  matrix[2][0] = -v * rsin + w*u*(1-rcos);
  matrix[0][1] = -w * rsin + u*v*(1-rcos);
  matrix[1][1] =      rcos + v*v*(1-rcos);
  matrix[2][1] =  u * rsin + w*v*(1-rcos);
  matrix[0][2] =  v * rsin + u*w*(1-rcos);
  matrix[1][2] = -u * rsin + v*w*(1-rcos);
  matrix[2][2] =      rcos + w*w*(1-rcos);
  matrix[3][3] = 1.0;

  return matrix;
}

std::ostream &operator<<(std::ostream &stream, const Matrix4x4& mat)
{
  stream << "Matrix4x4:" << std::endl;
  for(unsigned int i = 0; i < 4; i++){
    stream << "\t";
    for(unsigned int j = 0; j < 4; j++)
      stream << std::setw(7) << mat[i][j] << " ";
    stream << std::endl;
  }
  return stream;
}

Matrix4x4::Matrix4x4(const double* data)
{
  memcpy(_data, data, 16*sizeof(double));
}

Vector4 Matrix4x4::operator* (const Vector4& vec) const
{
  Vector4 prod;
  for (uint row = 0; row < 4; row++){
    prod[row] = 0;
    for (uint col = 0; col < 4; col++)
      prod[row] += (*this)[row][col] * vec[col];
  }
  return prod;
}

Matrix4x4 Matrix4x4::identity()
{
  Matrix4x4 ident;
  ident.makeDiag(1.0);
  return ident;
}

Matrix4x4 Matrix4x4::scale(double sx, double sy, double sz)
{
  Matrix4x4 scale;
  scale[0][0] = sx;
  scale[1][1] = sy;
  scale[2][2] = sz;
  scale[3][3] = 1;
  return scale;
}

}
