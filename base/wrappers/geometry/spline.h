#ifndef BASE_WRAPPERS_GEOMETRY_SPLINE
#define BASE_WRAPPERS_GEOMETRY_SPLINE

#include "base/wrappers/eigen.h"
#ifndef __orogen
#include <vector>
#include <base/geometry/spline.h>
#endif

namespace wrappers {
namespace geometry {
    enum SplineType
    {
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

#ifndef __orogen
        Spline()
            : geometric_resolution(1), kind(POLYNOMIAL_BSPLINE) {}
        Spline(base::geometry::SplineBase const& source);

        operator base::geometry::SplineBase() const;

        template<int DIM>
        base::geometry::Spline<DIM> cast() const
        {
            base::geometry::Spline<DIM> result;
            result = static_cast<base::geometry::SplineBase>(*this);
            return result;
        }
#endif
    };
}
}

#endif

