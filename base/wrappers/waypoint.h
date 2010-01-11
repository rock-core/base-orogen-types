#ifndef __WRAPPERS_WAYPOINT_HH
#define __WRAPPERS_WAYPOINT_HH

#ifndef __orogen
#include <base/waypoint.h>
#endif

#include <base/wrappers/eigen.h>

namespace wrappers
{
    typedef wrappers::Vector3    Position;

    /** Wrapper class for base::Waypoint
     */
    struct Waypoint
    {
        Position   position;
	double heading;
	
	double tol_position;
	double tol_heading;

#ifndef __orogen
        Waypoint()
	  : position(wrappers::Vector3(0,0,0)), heading(0), tol_position(0), tol_heading(0)  {}
      
	Waypoint(const Eigen::Vector3d &position, const double heading, const double tol_position, const double tol_heading):
	    position(position), heading(heading), tol_position(tol_position), tol_heading(tol_heading) {};

	    Waypoint(base::Waypoint wp) : position(wp.position), heading(wp.heading), tol_position(wp.tol_position), tol_heading(wp.tol_heading) 
	    {}
	    
        operator base::Waypoint() const
      { 
	Eigen::Vector3d position_eigen = position;
	return base::Waypoint(position_eigen, heading, tol_position, tol_heading); }
#endif
    };
}

#endif

