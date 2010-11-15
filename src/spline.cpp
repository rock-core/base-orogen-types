#include "base/wrappers/geometry/spline.h"
#include <base/geometry/spline.h>
#include <sisl/sisl.h>

using namespace wrappers::geometry;

Spline::Spline(::base::geometry::SplineBase const& source)
    : geometric_resolution(source.getGeometricResolution())
    , dimension(source.getDimension())
    , curve_order(source.getCurveOrder())
    , kind(DEGENERATE)
{
    SISLCurve const* curve = source.getSISLCurve();
    if (curve)
        kind = static_cast<SplineType>(curve->ikind);

    knots            = source.getKnots();
    vertices         = source.getCoordinates();
}

