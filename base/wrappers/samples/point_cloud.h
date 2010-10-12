#ifndef BASE_WRAPPERS_SAMPLES_POINT_CLOUD_H__
#define BASE_WRAPPERS_SAMPLES_POINT_CLOUD_H__ 

#include <base/time.h>
#include <base/wrappers/eigen.h>

#ifndef __orogen
#include <vector>
#include <Eigen/Geometry>
#endif

namespace wrappers {

        struct PointCloud {
#ifndef __orogen

	PointCloud() {}

	PointCloud(const std::vector<Eigen::Vector3d> &pc, const Eigen::Transform3d &pc2World) {
	    for(std::vector<Eigen::Vector3d>::const_iterator it = pc.begin(); it != pc.end(); it++) {
		points.push_back(*it);
	    }
	    this->pc2World = pc2World.matrix();
	};
#endif
	wrappers::Matrix4 pc2World;
	std::vector<Vector3> points;
    };
    
}

#endif