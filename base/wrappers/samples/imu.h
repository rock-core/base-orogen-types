#ifndef BASE_WRAPPERS_SAMPLES_IMU_H__
#define BASE_WRAPPERS_SAMPLES_IMU_H__ 

#include <base/time.h>
#include <base/wrappers/eigen.h>

#ifndef __orogen
#include <base/samples/imu.h>
#endif

namespace wrappers { namespace samples {
    struct IMUSensors
    {
         /** Timestamp of the orientation reading */
        base::Time time;
        
        /** raw accelerometer readings */
        Vector3 acc;

        /** raw gyro reading*/
        Vector3 gyro;

        /** raw magnetometer reading*/
        Vector3 mag;

#ifndef __orogen
        IMUSensors() {}
        IMUSensors(base::samples::IMUSensors const& s)
            : time(s.time)
            , acc(s.acc)
            , gyro(s.gyro)
            , mag(s.mag) {}

        operator base::samples::IMUSensors() const
        {
            base::samples::IMUSensors s;
            s.time = time;
            s.acc  = acc;
            s.gyro = gyro;
            s.mag  = mag;
            return s;
        }
#endif
    };
}} // namespaces

#endif

