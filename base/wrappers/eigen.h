#ifndef BASE_TYPES_EIGEN_WRAPPERS_H
#define BASE_TYPES_EIGEN_WRAPPERS_H

#ifndef __orogen
#include <Eigen/Core>
#include <Eigen/Geometry> 
#endif

namespace wrappers
{
    /**
     * Wrapper class for Eigen 3x3 matrices.
     */ 
    struct Matrix3 
    {
      double data[9];

#ifndef __orogen
      Matrix3()
      {
          for(int i=0;i<9;data[i++]=0);
      }

      Matrix3(const Eigen::Matrix3d& mat)
      {
          for(int i=0;i<9;i++)
              data[i] = mat(i/3,i%3);
      }

      Matrix3 &operator=(const Eigen::Matrix3d &mat)
      {
          for(int i=0;i<9;i++)
              data[i] = mat(i/3,i%3);
          return *this;
      }

      operator Eigen::Matrix3d() const 
      {
          Eigen::Matrix3d m;
          for(int i=0;i<9;i++)
              m(i/3,i%3) = data[i];
          return m;
      }

      double& operator() (int m,int n) 
      {
          return data[m*3+n];
      }
#endif
    };

    /**
     * Wrapper class for Eigen 3-vectors.
     */ 
    struct Vector3
    {
      double data[3];
#ifndef __orogen
      Vector3() {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
      };
      
      Vector3(const Eigen::Vector3d& vec) 
      {
        x() = vec.x();
        y() = vec.y();
        z() = vec.z();
      }
      
      Vector3 &operator=(const Eigen::Vector3d &vec) {
        x() = vec.x();
        y() = vec.y();
        z() = vec.z();
        return *this;
      }

      double &x() 
      {
        return data[0];
      }

      const double &x() const
      {
        return data[0];
      }
      
      double &y() 
      {
        return data[1];
      }

      const double &y() const 
      {
        return data[1];
      }

      double &z() 
      {
        return data[2];
      }

      const double &z() const 
      {
        return data[2];
      }

      operator Eigen::Vector3d() const {
        return Eigen::Vector3d(x(), y(), z());
      }
#endif
    };

    /**
     * Wrapper class for Eigen quaternions
     */ 
    struct Quaternion
    {
      // store as imaginary and real part, so it comes out clear in the pocosim logs
      double im[3];
      double re;

#ifndef __orogen
      Quaternion() {
        im[0] = 0;
        im[1] = 0;
        im[2] = 0;
        re = 1.0;
      };
      
      Quaternion(const Eigen::Quaterniond &q) 
      {
        x() = q.x();
        y() = q.y();
        z() = q.z();
        w() = q.w();
      }

      Quaternion &operator=(const Eigen::Quaterniond &q) {
        x() = q.x();
        y() = q.y();
        z() = q.z();
        w() = q.w();
        return *this;
      }
      
      double &x() 
      {
        return im[0];
      }
      
      const double &x() const 
      {
        return im[0];
      }
      
      double &y() 
      {
        return im[1];
      }

      const double &y() const 
      {
        return im[1];
      }

      double &z() 
      {
        return im[2];
      }

      const double &z() const 
      {
        return im[2];
      }

      double &w() 
      {
        return re; 
      }

      const double &w() const 
      {
        return re;
      }
      
      operator Eigen::Quaterniond() const
      {
        return Eigen::Quaterniond(w(), x(), y(), z());
      }
#endif
    };
}

#endif
