#include "base/wrappers/geometry/NURBSCurve3D.h"
#include <base/geometry/NURBSCurve3D.h>
#include <sisl/sisl.h>

using namespace wrappers::geometry;

NURBSCurve3D::NURBSCurve3D(::base::geometry::NURBSCurve3D const& source)
{
    geometric_resolution = source.getGeometricResolution();
    curve_order  = source.getCurveOrder();

    std::vector<Eigen::Vector3d> const& source_points = source.getPoints();
    points.insert(points.end(),
            source_points.begin(), source_points.end());

    SISLCurve const* curve = source.getSISLCurve();
    if (curve)
    {
        int vertex_count    = curve->in;
        curve_order     = curve->ik;
        kind            = static_cast<NURBSCurve3DType>(curve->ikind);

        knots.resize(vertex_count + curve_order);

        memcpy(&knots[0], &(curve->et[0]), sizeof(double) * knots.size());

        if (kind % 2 == 0) // rational curve
        {
            vertices.resize(vertex_count * (DIMENSION + 1));
            memcpy(&vertices[0], &(curve->rcoef[0]), sizeof(double) * vertices.size());
        }
        else
        {
            vertices.resize(vertex_count * DIMENSION);
            memcpy(&vertices[0], &(curve->ecoef[0]), sizeof(double) * vertices.size());
        }
    }
}

NURBSCurve3D::operator ::base::geometry::NURBSCurve3D() const
{
    std::vector<Eigen::Vector3d> eigen_points(points.begin(), points.end());
    if (!vertices.empty())
    {
        int vertex_count;
        if (kind % 2 == 0) // rational curve;
            vertex_count = vertices.size() / (DIMENSION + 1);
        else
            vertex_count = vertices.size() / DIMENSION;

        SISLCurve* curve = newCurve(vertex_count,
                curve_order, const_cast<double*>(&knots[0]), const_cast<double*>(&vertices[0]), kind, DIMENSION, 1);
        curve->cuopen = 1;
        return base::geometry::NURBSCurve3D(geometric_resolution, curve_order, eigen_points, curve);
    }
    else
    {
        return base::geometry::NURBSCurve3D(geometric_resolution, curve_order, eigen_points);
    }
}

