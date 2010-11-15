#ifndef BASE_TYPES_EIGEN_WRAPPERS_H
#define BASE_TYPES_EIGEN_WRAPPERS_H

#ifndef __orogen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#endif

namespace wrappers
{
    /**
     * Wrapper class for Eigen matrices
     */ 
    template<typename _Scalar, int _ROWS, int _COLS>
    struct Matrix
    {
        typedef _Scalar Scalar;
        _Scalar data[_ROWS * _COLS];
    };

    /**
     * Wrapper class for Eigen quaternions
     */ 
    template<typename _Scalar>
    struct Quaternion
    {
        // store as imaginary and real part, so it comes out clear in the pocosim logs
        _Scalar im[3];
        _Scalar re;
    };

    /** This is needed to that GCCXML sees the particular template instances,
     * and so, so that we can typedef them below
     */
    struct __gccxml_workaround_eigen_wrappers_instanciator {
        Matrix<double, 3, 3> matrix3;
        Matrix<double, 4, 4> matrix4;
        Matrix<double, 3, 4> matrix3x4;
        Matrix<double, 4, 4> matrix4x4;
        Matrix<double, 4, 3> matrix4x3;
        Matrix<double, 3, 1> vector3;
        Matrix<double, 4, 1> vector4;
        Quaternion<double>   quaternion;
    };

    typedef Matrix<double, 3, 3> Matrix3d;
    typedef Matrix<double, 4, 4> Matrix4d;
    typedef Matrix<double, 3, 4> Matrix3x4d;
    typedef Matrix<double, 4, 4> Matrix4x4d;
    typedef Matrix<double, 4, 3> Matrix4x3d;
    typedef Matrix<double, 3, 1> Vector3d;
    typedef Matrix<double, 3, 1> Vector4d;
    typedef Quaternion<double>   Quaterniond;
}

#endif
