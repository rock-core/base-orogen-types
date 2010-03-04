#include "base/wrappers/geometry/spline.h"
#include <base/geometry/spline.h>
#include <sisl/sisl.h>

using namespace wrappers::geometry;

Spline::Spline(::base::geometry::SplineBase const& source)
    : geometric_resolution(source.getGeometricResolution())
    , dimension(source.getDimension())
    , curve_order(source.getCurveOrder())
{
    SISLCurve const* curve = source.getSISLCurve();
    if (curve)
    {
        int vertex_count = curve->in;
        curve_order      = curve->ik;
        kind             = static_cast<SplineType>(curve->ikind);

        knots.resize(vertex_count + curve_order);

        memcpy(&knots[0], &(curve->et[0]), sizeof(double) * knots.size());

        if (kind % 2 == 0) // rational curve
        {
            vertices.resize(vertex_count * (dimension + 1));
            memcpy(&vertices[0], &(curve->rcoef[0]), sizeof(double) * vertices.size());
        }
        else
        {
            vertices.resize(vertex_count * dimension);
            memcpy(&vertices[0], &(curve->ecoef[0]), sizeof(double) * vertices.size());
        }
    }
}

Spline::operator ::base::geometry::SplineBase() const
{
    if (!knots.empty()) // we have a SISL curve definition
    {
        int vertex_count;
        if (kind % 2 == 0) // rational curve;
            vertex_count = vertices.size() / (dimension + 1);
        else
            vertex_count = vertices.size() / dimension;

        SISLCurve* curve = newCurve(vertex_count,
                curve_order, const_cast<double*>(&knots[0]), const_cast<double*>(&vertices[0]), kind, dimension, 1);
        curve->cuopen = 1;
        return base::geometry::SplineBase(geometric_resolution, curve);
    }
    else
    {
        return base::geometry::SplineBase(dimension, geometric_resolution, curve_order);
    }
}

