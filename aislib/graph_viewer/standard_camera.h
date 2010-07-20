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
