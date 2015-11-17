/* Generated from orogen/lib/orogen/templates/typekit/Opaques.hpp */

#ifndef __OROGEN_GENERATED_base_USER_MARSHALLING_HH
#define __OROGEN_GENERATED_base_USER_MARSHALLING_HH

namespace orogen_typekits
{
    template<typename T, int ROWS, int COLS, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void toIntermediate(wrappers::Matrix<T, ROWS, COLS>& intermediate, Eigen::Matrix<T, ROWS, COLS, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS> const& real)
    {
        for(int i=0; i < ROWS*COLS; ++i)
            intermediate.data[i] = real(i / COLS, i % COLS);
    }

    template<typename T, int ROWS, int COLS, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void fromIntermediate(Eigen::Matrix<T, ROWS, COLS, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>& real, wrappers::Matrix<T, ROWS, COLS> const& intermediate)
    {
        for(int i=0; i < ROWS*COLS; ++i)
            real(i / COLS, i % COLS) = intermediate.data[i];
    }
    

    template<typename T, int ROWS, int COLS, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void toIntermediate(::wrappers::MatrixX<T>& intermediate, Eigen::Matrix<T, ROWS, COLS, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS> const& real)
    {
        intermediate.data.resize(real.size(), 0.0);
        intermediate.rows = real.rows();
        intermediate.cols = real.cols();

        typedef Eigen::Matrix<T,ROWS,COLS,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenMatrix;
        
        Eigen::Map<EigenMatrix> m(&(intermediate.data[0]),real.rows(), real.cols());
        m = real;
    }

    template<typename T, int ROWS, int COLS, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void fromIntermediate(Eigen::Matrix<T, ROWS, COLS, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>& real, ::wrappers::MatrixX<T> const& intermediate)
    {
        typedef const Eigen::Matrix<T,ROWS,COLS,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenMatrix;
        
        if (static_cast<int>(intermediate.data.size()) != intermediate.rows * intermediate.cols)
            throw std::logic_error("vector size does not match rows and cols in /wrappers/MatrixX");

        Eigen::Map<EigenMatrix> m(&(intermediate.data[0]),intermediate.rows, intermediate.cols);
        real = m;
    }


    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void toIntermediate(wrappers::MatrixX<T>& intermediate, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS> const& real)
    {
        toIntermediate<T,Eigen::Dynamic,Eigen::Dynamic,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS>(intermediate, real);
    }

    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void fromIntermediate(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>& real, wrappers::MatrixX<T> const& intermediate)
    {
        fromIntermediate<T, Eigen::Dynamic, Eigen::Dynamic, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>(real, intermediate); 
    }

    
    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void toIntermediate(wrappers::VectorX<T>& intermediate, Eigen::Matrix<T, Eigen::Dynamic, 1, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS> const& real)
    {
        typedef Eigen::Matrix<T,Eigen::Dynamic,1,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenVector;
        
        intermediate.data.resize(real.size(),0.0);
        
        Eigen::Map<EigenVector> m(&(intermediate.data[0]),real.size());
        m = real;
    }

    template<typename T, int EIGEN_OPTIONS, int EIGEN_MAX_ROWS, int EIGEN_MAX_COLS>
    void fromIntermediate(Eigen::Matrix<T, Eigen::Dynamic, 1, EIGEN_OPTIONS, EIGEN_MAX_ROWS, EIGEN_MAX_COLS>& real, wrappers::VectorX<T> const& intermediate)
    {
        typedef const Eigen::Matrix<T,Eigen::Dynamic,1,EIGEN_OPTIONS,EIGEN_MAX_ROWS,EIGEN_MAX_COLS> EigenVector;
        
        Eigen::Map<EigenVector> m(&(intermediate.data[0]),intermediate.data.size());
        real = m;
    }


    template<typename T, int EIGEN_OPTIONS>
    void toIntermediate(::wrappers::Quaternion<T>& intermediate, ::Eigen::Quaternion<T, EIGEN_OPTIONS> const& real)
    {
        intermediate.re = real.w();
        intermediate.im[0] = real.x();
        intermediate.im[1] = real.y();
        intermediate.im[2] = real.z();
    }

    template<typename T, int EIGEN_OPTIONS>
    void fromIntermediate(::Eigen::Quaternion<T, EIGEN_OPTIONS>& real, ::wrappers::Quaternion<T> const& intermediate)
    {
        real.w() = intermediate.re;
        real.x() = intermediate.im[0];
        real.y() = intermediate.im[1];
        real.z() = intermediate.im[2];
    }

    template<typename T>
    void toIntermediate(::wrappers::AngleAxis<T>& intermediate, ::Eigen::AngleAxis<T> const& real)
    {
        intermediate.angle = real.angle();
        intermediate.axis[0] = real.axis()[0];
        intermediate.axis[1] = real.axis()[1];
        intermediate.axis[2] = real.axis()[2];
    }

    template<typename T>
    void fromIntermediate(::Eigen::AngleAxis<T>& real, ::wrappers::AngleAxis<T> const& intermediate)
    {
        real.angle() = intermediate.angle;
        real.axis()[0] = intermediate.axis[0];
        real.axis()[1] = intermediate.axis[1];
        real.axis()[2] = intermediate.axis[2];
    }

    void toIntermediate(::wrappers::geometry::Spline& intermediate, ::base::geometry::SplineBase const& real_type);

    void fromIntermediate(::base::geometry::SplineBase& real_type, ::wrappers::geometry::Spline const& intermediate);


    void toIntermediate(::wrappers::Matrix<double, 4, 4>& intermediate, ::base::Affine3d const& real);

    void fromIntermediate(::base::Affine3d& real, wrappers::Matrix<double, 4, 4> const& intermediate);
}

#endif

