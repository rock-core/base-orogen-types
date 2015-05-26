#ifndef BASE_TYPES_EIGEN_WRAPPERS_H
#define BASE_TYPES_EIGEN_WRAPPERS_H

#include <vector>

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
    
    /** Wrapper for Eigen::MatrixXd. */
    template<typename _Scalar>
    struct MatrixX
    {
        typedef _Scalar Scalar;
        int rows;
        int cols;
        std::vector<_Scalar> data;
    };
    
    /** Wrapper for Eigen::VectorXd. */
    template<typename _Scalar>
    struct VectorX
    {
        typedef _Scalar Scalar;
        std::vector<_Scalar> data; 
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

    /**
     * Wrapper class for Eigen AngleAxis
     */ 
    template<typename _Scalar>
    struct AngleAxis
    {
        // store as angle and axis part, so it comes out clear in the pocosim logs
        _Scalar angle;
        _Scalar axis[3];
    };


    /** This is needed to that GCCXML sees the particular template instances,
     * and so, so that we can typedef them below
     */
    struct __gccxml_workaround_eigen_wrappers_instanciator {
        Matrix<double, 2, 2> matrix2;
        Matrix<double, 3, 3> matrix3;
        Matrix<double, 4, 4> matrix4;
        Matrix<double, 6, 6> matrix6;
        Matrix<double, 4, 4> matrix4x4;
        Matrix<double, 2, 1> vector2;
        Matrix<double, 3, 1> vector3;
        Matrix<double, 4, 1> vector4;
        Matrix<double, 6, 1> vector6;
        Quaternion<double>   quaternion;
        AngleAxis<double>   angleaxis;
        VectorX<double> vectorx;
        MatrixX<double> matrixx;
    };

    typedef Matrix<double, 2, 2> Matrix2d;
    typedef Matrix<double, 3, 3> Matrix3d;
    typedef Matrix<double, 4, 4> Matrix4d;
    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 4, 4> Matrix4x4d;
    typedef Matrix<double, 2, 1> Vector2d;
    typedef Matrix<double, 3, 1> Vector3d;
    typedef Matrix<double, 4, 1> Vector4d;
    typedef Matrix<double, 6, 1> Vector6d;
    typedef Quaternion<double>   Quaterniond;
    typedef AngleAxis<double>   AngleAxisd;
    typedef VectorX<double> VectorXd;
    typedef MatrixX<double> MatrixXd;
}

#endif
