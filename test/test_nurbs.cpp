#include <boost/test/auto_unit_test.hpp>
#include "base/wrappers/geometry/spline.h"
#include <base/geometry/spline.h>
#include <iostream>

using namespace std;

void initRandomCurve(base::geometry::NURBSCurve3D& in)
{
    // Create 10 random points
    vector<Eigen::Vector3d> points;
    Eigen::Vector3d p;
    for (int i = 0; i < 10; ++i)
    {
        // p += Eigen::Vector3d(rand(), rand(), rand());
        p += Eigen::Vector3d(1, 1, 1);
        points.push_back(p);
    }
    in.interpolate(points);
}

void checkSameCurve(base::geometry::NURBSCurve3D& in, base::geometry::NURBSCurve3D& out)
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
        Eigen::Vector3d in_p = in.getPoint(t);
        Eigen::Vector3d out_p = out.getPoint(t);
        BOOST_REQUIRE_SMALL(in_p.x() - out_p.x(), 0.001);
        BOOST_REQUIRE_SMALL(in_p.y() - out_p.y(), 0.001);
        BOOST_REQUIRE_SMALL(in_p.z() - out_p.z(), 0.001);
    }
}

BOOST_AUTO_TEST_CASE( test_nurbscurve3d_empty )
{
    base::geometry::NURBSCurve3D in;
    wrappers::geometry::Spline w(in);
    base::geometry::NURBSCurve3D out = w.cast<3>();
}

BOOST_AUTO_TEST_CASE( test_nurbscurve3d )
{
    base::geometry::NURBSCurve3D in;
    initRandomCurve(in);

    wrappers::geometry::Spline w(in);
    base::geometry::NURBSCurve3D out = w.cast<3>();

    BOOST_REQUIRE(in.getSISLCurve());
    BOOST_REQUIRE(out.getSISLCurve());

    checkSameCurve(in, out);
}

