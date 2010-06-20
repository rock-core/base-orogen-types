#ifndef __WRAPPERS_SAMPLES_RIGID_BODY_ACCELERATION_HH
#define __WRAPPERS_SAMPLES_RIGID_BODY_ACCELERATION_HH

#ifndef __orogen
#include <base/samples/rigid_body_acceleration.h>
#endif

#include <base/time.h>
#include <base/wrappers/pose.h>

namespace wrappers { namespace samples {
    struct RigidBodyAcceleration 
    {
        base::Time time;
        
	/** Acceleration in m/s, world fixed frame of reference (East-North-Up) */
        wrappers::Vector3 acceleration;
	/** Covariance matrix of the acceleration
	 */
        Matrix3 cov_acceleration;


#ifndef __orogen
        RigidBodyAcceleration() {}

        RigidBodyAcceleration(base::samples::RigidBodyAcceleration const& s)
            : time(s.time)
            , acceleration(s.acceleration)
            {}

        operator base::samples::RigidBodyAcceleration() const
        {
            base::samples::RigidBodyAcceleration state;
            state.time = time;
            state.acceleration = acceleration;
            return state;
        }
#endif
    };
}}

#endif

