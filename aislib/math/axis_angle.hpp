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

template <typename Base>
_AxisAngle<Base>::_AxisAngle():_Vector<3, Base> (Base(0.),Base(0.),Base(0.)) {}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Quaternion<Base>& q) {
  Base angle=q.angle();
  Base imNorm = sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
  if (imNorm < std::numeric_limits<Base>::min()){
    this->x()=Base(0.);
    this->y()=Base(0.);
    this->z()=Base(0.);
  } else {
    Base alpha=angle/imNorm;
    this->x()=q.x()*alpha;
    this->y()=q.y()*alpha;
    this->z()=q.z()*alpha;
  }
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _RotationMatrix3<Base>& m) {
  *this=_Quaternion<Base>(m);
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Vector<3, Base>& vec) {
  *this=_Quaternion<Base>(vec);
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(Base roll, Base pitch, Base yaw){
  *this=_AxisAngle(_Quaternion<Base>(roll, pitch, yaw));
}

template <typename Base>
_AxisAngle<Base>::_AxisAngle(const _Vector<3, Base>& axis, Base angle){
  *this = axis.normalized() * angle;
}

template <typename Base>
_AxisAngle<Base>& _AxisAngle<Base>::operator*=(const _AxisAngle& a){
  *this=(*this)*a;
  return *this;
}
  
  
template <typename Base>
_AxisAngle<Base>  _AxisAngle<Base>::operator* (const _AxisAngle& a) const{
  return _AxisAngle<Base>(quaternion()*a.quaternion());
}
  
template <typename Base>
_Vector<3, Base> _AxisAngle<Base>::operator*(const _Vector<3, Base>& v) const {
  return rotationMatrix()*v;
}

template <typename Base>
inline _AxisAngle<Base> _AxisAngle<Base>::inverse() const {
  _AxisAngle a(*this);
  a._Vector<3, Base>::operator*=(Base(-1.));
  return a;
}
  
template <typename Base>
inline _Vector<3, Base> _AxisAngle<Base>::angles() const {
  return quaternion().angles();
}

template <typename Base>
_RotationMatrix3<Base> _AxisAngle<Base>::rotationMatrix() const{
  return quaternion().rotationMatrix();
}

template <typename Base>
_Quaternion<Base> _AxisAngle<Base>::quaternion() const {
  Base n=this->norm();
  if (n<=0){
    return _Quaternion<Base>();
  }
  Base s=sin(n*Base(.5))/n;
  Base c=cos(n*Base(.5));
  return _Quaternion<Base> (this->x()*s, this->y()*s, this->z()*s, c);
}

