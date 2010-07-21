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

#ifndef STANDARD_CAMERA_H
#define STANDARD_CAMERA_H

namespace AISNavigation {

class StandardCamera : public qglviewer::Camera
{
  public:
    StandardCamera() : _standard(true) {};

    float zNear() const {
      if (_standard) 
        return 0.001; 
      else 
        return Camera::zNear(); 
    }

    float zFar() const
    {  
      if (_standard) 
        return 1000.0; 
      else 
        return Camera::zFar();
    }

    void toggleMode() {_standard = !_standard;}
    bool isStandard() const {return _standard;}

  private:
    bool _standard;
};

} // end namespace

#endif
