#ifndef __WRAPPERS_SAMPLES_RIGID_BODY_STATE_HH
#define __WRAPPERS_SAMPLES_RIGID_BODY_STATE_HH

#ifndef __orogen
#include <base/samples/rigid_body_state.h>
#endif

#include <base/time.h>
#include <base/wrappers/pose.h>

namespace wrappers { namespace samples {
    struct RigidBodyState
    {
        base::Time time;

	/** Position in m, 
	 * world fixed frame of reference (East-North-Up) */
        Position   position;
	/** Covariance matrix of the position
	 */
        Matrix3    cov_position;

        /** Orientation as a body->world transformation */
        Quaternion orientation;
	/** Covariance matrix of the orientation as an 
	 * axis/angle manifold in body coordinates
	 */
        Matrix3    cov_orientation;

	/** Velocity in m/s with respect to world fixed frame, 
	 * in body fixed frame (Right-Front-Up) */
	Vector3 velocity;
	/** Covariance of the velocity 
	 */
	Matrix3 cov_velocity;

	/** Angular Velocity in rad/s,
	 * in body fixed frame (Right-Front-Up) */
	Vector3 angular_velocity;
	/** Covariance of the angular velocity
	 */
	Matrix3 cov_angular_velocity;

#ifndef __orogen
        RigidBodyState() {}

        RigidBodyState(base::samples::RigidBodyState const& s)
            : time(s.time)
            , position(s.position)
            , cov_position(s.cov_position)
            , orientation(s.orientation)
            , cov_orientation(s.cov_orientation)
            , velocity(s.velocity)
            , cov_velocity(s.cov_velocity)
            , angular_velocity(s.angular_velocity)
            , cov_angular_velocity(s.cov_angular_velocity) {}

        operator base::samples::RigidBodyState() const
        {
            base::samples::RigidBodyState state;
            state.time = time;
            state.position = position;
            state.cov_position = cov_position;
            state.orientation = orientation;
            state.cov_orientation = cov_orientation;
            state.velocity = velocity;
            state.cov_velocity = cov_velocity;
            state.angular_velocity = angular_velocity;
            state.cov_angular_velocity = cov_angular_velocity;
            return state;
        }
#endif
    };
}}

#endif

