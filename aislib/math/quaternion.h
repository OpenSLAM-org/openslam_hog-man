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

#ifndef _QUATERNION_H_
#define _QUATERNION_H_
#include <assert.h>
#include <limits>
#include <iostream>
#include "rotation_matrix.h"

using namespace std;

/** @addtogroup math **/
//@{

/**Rotation as the rotation axis whose lenght is proportional 
   to the entity of the rotation. 
   There is also a small algebra for UNIT quaternions.
   the function names are pretty self-explanatory.
*/
template <typename Base=double>
  struct _Quaternion : public _Vector<4, Base>
{
  static const int Angles=3;
  static const int Dimension=3;
  _Quaternion();
  _Quaternion(Base x, Base y, Base z, Base w);
  _Quaternion(const _RotationMatrix3<Base>& m);
  _Quaternion(const _Vector<3, Base>& vec);
  _Quaternion(Base roll, Base pitch, Base yaw);
  _Quaternion<Base>& operator*=(const _Quaternion& q);
  _Quaternion<Base>  operator* (const _Quaternion& q) const;
  template <typename Base2>
  _Quaternion(const _Quaternion<Base2>& other);  // Enable cast from quaternions with other, but compatible base
  _Vector<3, Base> operator*(const _Vector<3, Base>& v) const;
  inline _Quaternion<Base> inverse() const;
  inline _Vector<3, Base> angles() const;

  _RotationMatrix3<Base> rotationMatrix() const;
  _Quaternion<Base> normalized() const;
  
  //this function normalizes the quaternion and ensures thar w>0.
  _Quaternion<Base>& normalize();
  Base  angle() const;
  static inline _Quaternion<Base> slerp(const _Quaternion<Base>& from, const _Quaternion<Base>& to, Base lambda);
 protected:
  _Quaternion(const _Vector<4, Base>& v);
};

typedef _Quaternion<double> Quaternion;
typedef _Quaternion<float>  Quaternionf;

//@}

#include "quaternion.hpp"

 
#endif
