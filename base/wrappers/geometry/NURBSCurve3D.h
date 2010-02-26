#ifndef BASE_WRAPPERS_GEOMETRY_NURBSCURVE3D
#define BASE_WRAPPERS_GEOMETRY_NURBSCURVE3D

#include "base/wrappers/eigen.h"
#ifndef __orogen
#include <vector>
#endif

namespace base {
    namespace geometry {
        class NURBSCurve3D;
    }
}

namespace wrappers {
namespace geometry {
    enum NURBSCurve3DType
    {
        POLYNOMIAL_BSPLINE = 1,
        RATIONAL_BSPLINE   = 2,
        POLYNOMIAL_BEZIER  = 3,
        RATIONAL_BEZIER    = 4
    };

    struct NURBSCurve3D
    {
        double geometric_resolution;

        std::vector<wrappers::Vector3> points;

        /** The curve order */
        int curve_order;

        /** The type of the curve */
        NURBSCurve3DType kind;

        /** The curve knots. This is vertex_count + curve_order values */
        std::vector<double> knots;

        /** The vertices, as vertex_count * dimension coordinates if kind is
         * non-rational and vertex_count * (dimension + 1) otherwise. */
        std::vector<double> vertices;

#ifndef __orogen
        static const int DIMENSION = 3;

        NURBSCurve3D()
            : geometric_resolution(1), kind(POLYNOMIAL_BSPLINE) {}
        NURBSCurve3D(::base::geometry::NURBSCurve3D const& source);
        operator ::base::geometry::NURBSCurve3D() const;
#endif
    };
}
}

#endif

