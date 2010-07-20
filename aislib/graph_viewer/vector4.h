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
