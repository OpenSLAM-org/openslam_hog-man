#ifndef VRML_MATRI4X4_H
#define VRML_MATRI4X4_H

#include <stdexcept>
#include <iostream>

#include "vector4.h"

namespace vrml {

/**
 * \brief a 4x4 matrix used to represent the OpenGL rotation/translation Matrix stack
 */
class Matrix4x4
{
  public:
    Matrix4x4();
    ~Matrix4x4();

    Matrix4x4(const Matrix4x4& other);
    explicit Matrix4x4(const double* data);

    /**
     * make a diagonal matrix
     * @param d the value for the diagonal elems
     */
    void makeDiag(double d = 1.0);

    /**
     * index operator, to access the m-th row
     */
    double* operator[](unsigned int m) throw (std::runtime_error)
    {
      if (m > 4)
        throw std::runtime_error("index exceeds matrix dimensions");
      return &_data[m*4];
    }

    /**
     * index operator, to access the m-th row
     */
    const double* operator[](unsigned int m) const throw (std::runtime_error)
    {
      if (m > 4)
        throw std::runtime_error("index exceeds matrix dimensions");
      return &_data[m*4];
    }
    Matrix4x4& operator=(const Matrix4x4& other);
    //! matrix multiplication
    Matrix4x4 operator* (const Matrix4x4& other) const;
    //! calculate matrix * vec
    Vector4 operator* (const Vector4& vec) const;

    /**
     * return translation matrix
     */
    static Matrix4x4 translate(double dx, double dy, double dz);

    /**
     * rotate with an angle around the axis (u, v, w).
     * IMPORTANT angle is in degree.
     * @param phi rotation angle in radian
     * @param u, v, w the rotation axis
     */
    static Matrix4x4 rotate(double phi, double u, double v, double w);

    /**
     * get scaling Matrix
     */
    static Matrix4x4 scale(double sx, double sy, double sz);
    /**
     * return the identity matrix
     */
    static Matrix4x4 identity();

  protected:
    //! copy method for a deep copy of the data
    void copy(const Matrix4x4& other);
    double _data[16]; ///< array representing the matrix elems
};

std::ostream &operator<<(std::ostream &stream, const Matrix4x4& mat);

}

#endif
