#ifndef BASE_WRAPPERS_GEOMETRY_SPLINE
#define BASE_WRAPPERS_GEOMETRY_SPLINE

#include <vector>

namespace base {
namespace geometry {
    class SplineBase;
}
}

namespace wrappers {
namespace geometry {
    enum SplineType
    {
        DEGENERATE         = 0,
        POLYNOMIAL_BSPLINE = 1,
        RATIONAL_BSPLINE   = 2,
        POLYNOMIAL_BEZIER  = 3,
        RATIONAL_BEZIER    = 4
    };

    struct Spline
    {
        double geometric_resolution;

        /** The dimension in which the curve lies */
        int dimension;

        /** The curve order */
        int curve_order;

        /** The type of the curve */
        SplineType kind;

        /** The curve knots. This is vertex_count + curve_order values */
        std::vector<double> knots;

        /** The vertices, as vertex_count * dimension coordinates if kind is
         * non-rational and vertex_count * (dimension + 1) otherwise. */
        std::vector<double> vertices;

        Spline() {}
        Spline(::base::geometry::SplineBase const&);
    };
}
}

#endif

