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

#ifndef VRML_VECTOR4_H
#define VRML_VECTOR4_H

#include <stdexcept>
#include <iostream>

namespace vrml {

/**
 * \brief a 4d vector used together with Matrix4x4 to represent the OpenGL Matrix Stack
 */
class Vector4
{
  public:
    Vector4();
    ~Vector4();

    Vector4(const Vector4& other);
    Vector4(double d1, double d2, double d3, double d4);
    explicit Vector4(const double* data);

    double& operator[](unsigned int m) throw (std::runtime_error)
    {
      if (m > 4)
        throw std::runtime_error("index exceeds vector4 dimensions");
      return _data[m];
    }

    const double& operator[](unsigned int m) const throw (std::runtime_error)
    {
      if (m > 4)
        throw std::runtime_error("index exceeds vector4 dimensions");
      return _data[m];
    }

    Vector4& operator=(const Vector4& other);
    bool operator== (const Vector4&  other) const;
    bool operator!= (const Vector4& other) const;

    Vector4 operator+ (const Vector4& other) const;
    Vector4 operator- (const Vector4& other) const;
    Vector4 operator* (double x) const;
    Vector4 operator/ (double x) const;
    Vector4 operator- () const;
//    operator double*() {return _data;}
//    operator const double*() const {return _data;}

    Vector4& operator+= (const Vector4& other);
    Vector4& operator-= (const Vector4& other);
    Vector4& operator*= (double x);
    Vector4& operator/= (double x);
    friend Vector4 operator* (double x, const Vector4& v);
    friend Vector4 operator/ (double x, const Vector4& v);

  protected:
    double _data[4]; ///< array that represents the vector elems
    void copy(const Vector4& other);
};

std::ostream& operator<<(std::ostream& os, const Vector4& v);

}
#endif
