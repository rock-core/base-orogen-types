#include <boost/test/auto_unit_test.hpp>
#include <base/wrappers/geometry/Spline.hpp>
#include <base/geometry/Spline.hpp>
#include <iostream>

using namespace std;

void initRandomCurve(base::geometry::Spline<3>& in)
{
    // Create 10 random points
    vector<base::Vector3d> points;
    base::Vector3d p(0, 0, 0);
    for (int i = 0; i < 10; ++i)
    {
        p += base::Vector3d(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
        points.push_back(p);
    }
    in.interpolate(points);
}

void checkSameCurve(base::geometry::Spline<3>& in, base::geometry::Spline<3>& out)
{
    // Now check that the two curves are really the same
    BOOST_REQUIRE_EQUAL(in.getStartParam(), out.getStartParam());
    BOOST_REQUIRE_EQUAL(in.getEndParam(), out.getEndParam());
    BOOST_REQUIRE( in.getCurvatureMax() > 0 );
    BOOST_REQUIRE( in.getCurveLength() > 0 );
    BOOST_REQUIRE_EQUAL(in.getCurvatureMax(), out.getCurvatureMax());
    BOOST_REQUIRE_EQUAL(in.getCurveLength(), out.getCurveLength());

    double start = in.getStartParam();
    double end   = in.getEndParam();
    for (double t = start; t < end; t += (end - start) / 10)
    {
        base::Vector3d in_p  = in.getPoint(t);
        base::Vector3d out_p = out.getPoint(t);
        BOOST_REQUIRE_CLOSE(in_p.x(), out_p.x(), 0.001);
        BOOST_REQUIRE_CLOSE(in_p.y(), out_p.y(), 0.001);
        BOOST_REQUIRE_CLOSE(in_p.z(), out_p.z(), 0.001);
    }
}

