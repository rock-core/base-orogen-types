/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include "Opaques.hpp"

/** All the opaques convertion functions to/from eigen are templated in
 * Opaques.hpp
 */

void orogen_typekits::toIntermediate(::wrappers::geometry::Spline& intermediate, ::base::geometry::SplineBase const& real_type)
{
    intermediate = wrappers::geometry::Spline(real_type);
}

void orogen_typekits::fromIntermediate(::base::geometry::SplineBase& real_type, ::wrappers::geometry::Spline const& intermediate)
{
    base::geometry::SplineBase result(intermediate.dimension, intermediate.geometric_resolution, intermediate.curve_order);
    if (!intermediate.vertices.empty())
        result.reset(intermediate.vertices, intermediate.knots, intermediate.kind);
    real_type = result;
}



