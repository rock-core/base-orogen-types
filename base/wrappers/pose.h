#ifndef __WRAPPERS_POSE_HH
#define __WRAPPERS_POSE_HH

#ifndef __orogen
#include <base/pose.h>
#endif

#include <base/wrappers/eigen.h>

namespace wrappers
{
    typedef wrappers::Vector3    Position;
    typedef wrappers::Quaternion Orientation;

    /** Wrapper class for base::Pose
     */
    struct Pose
    {
        Position   position;
        Orientation orientation;

#ifndef __orogen
        Pose()
            : position(), orientation() {}

        Pose(base::Position const& p, base::Orientation const& o)
            : position(p), orientation(o) {}

        Pose(base::Pose const& p)
            : position(p.position), orientation(p.orientation) {}

        operator base::Pose() const
        { return base::Pose(position, orientation); }
#endif
    };
}

#endif

