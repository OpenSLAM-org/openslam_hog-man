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

#include "vector4.h"

#include <aislib/stuff/macros.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
using namespace std;

namespace vrml {

Vector4::Vector4()
{
  memset(_data, 0, 4*sizeof(double));
}

Vector4::~Vector4()
{
}

Vector4::Vector4(const Vector4& other)
{
  copy(other);
}

void Vector4::copy(const Vector4& other)
{
  memcpy(_data, other._data, 4*sizeof(double));
}

Vector4::Vector4(const double* data)
{
  memcpy(_data, data, 4*sizeof(double));
}

Vector4& Vector4::operator=(const Vector4& other)
{
  if (this != &other) {
    copy(other);
  }
  return *this;
}

bool Vector4::operator== (const Vector4& other) const
{
  if ((fabs(_data[0] - other[0]) < PREC) &&
      (fabs(_data[1] - other[1]) < PREC) &&
      (fabs(_data[2] - other[2]) < PREC) &&
      (fabs(_data[3] - other[3]) < PREC))
    return true;
  return false;
}

bool Vector4::operator!= (const Vector4& other) const
{
  return !(*this == other);
}

Vector4 Vector4::operator+ (const Vector4& other) const
{
  Vector4 sum(other);
  for (unsigned int i = 0; i < 4; ++i)
    sum[i] += _data[i];
  return sum;
}

Vector4 Vector4::operator- (const Vector4& other) const
{
  Vector4 diff(*this);
  for (unsigned int i = 0; i < 4; ++i)
    diff[i] -= other[i];
  return diff;
}

Vector4 Vector4::operator* (double x) const
{
  Vector4 prod(*this);
  for (unsigned int i = 0; i < 4; ++i)
    prod[i] *= x;
  return prod;
}

Vector4 Vector4::operator/ (double x) const
{
  Vector4 quot(*this);
  for (unsigned int i = 0; i < 4; ++i)
    quot[i] /= x;
  return quot;
}

Vector4 Vector4::operator- () const
{
  Vector4 minus(*this);
  for (unsigned int i = 0; i < 4; ++i)
    minus[i] *= -1.0;
  return minus;
}

Vector4& Vector4::operator+= (const Vector4& other)
{
  *this = *this + other;
  return *this;
}

Vector4& Vector4::operator-= (const Vector4& other)
{
  *this = *this - other;
  return *this;
}

Vector4& Vector4::operator*= (double x)
{
  *this = *this * x;
  return *this;
}

Vector4& Vector4::operator/= (double x)
{
  *this = *this / x;
  return *this;
}

Vector4 operator* (double x, const Vector4& v)
{
  return v * x;
}

Vector4 operator/ (double x, const Vector4& v)
{
  Vector4 out;
  for (unsigned int i = 0; i < 4; ++i)
    out[i] = x / v[i];
  return out;
}

std::ostream& operator<<(std::ostream& os, const Vector4& v)
{
  os << v[0] << " " << v[1] << " " << v[2] << " " << v[3];
  return os;
}

Vector4::Vector4(double d1, double d2, double d3, double d4)
{
  _data[0] = d1;
  _data[1] = d2;
  _data[2] = d3;
  _data[3] = d4;
}

}
