#include <boost/test/auto_unit_test.hpp>
#include "base/wrappers/geometry/NURBSCurve3D.h"
#include <base/geometry/NURBSCurve3D.h>

void initRandomCurve(base::geometry::NURBSCurve3D& in)
{
    // Create 10 random points
    Eigen::Vector3d p;
    for (int i = 0; i < 10; ++i)
    {
        // p += Eigen::Vector3d(rand(), rand(), rand());
        p += Eigen::Vector3d(1, 1, 1);
        in.addPoint( p );
    }
}

void checkSameCurve(base::geometry::NURBSCurve3D& in, base::geometry::NURBSCurve3D& out)
{
    // Now check that the two curves are really the same
    BOOST_REQUIRE_EQUAL(in.getStartParam(), out.getStartParam());
    BOOST_REQUIRE_EQUAL(in.getEndParam(), out.getEndParam());

    double start = in.getStartParam();
    double end   = in.getEndParam();
    for (double t = start; t < end; t += (end - start) / 10)
    {
        Eigen::Vector3d in_p = in.getPoint(t);
        Eigen::Vector3d out_p = out.getPoint(t);
        BOOST_CHECK_CLOSE(in_p.x(), out_p.x(), 0.001);
        BOOST_CHECK_CLOSE(in_p.y(), out_p.y(), 0.001);
        BOOST_CHECK_CLOSE(in_p.z(), out_p.z(), 0.001);
    }
}

BOOST_AUTO_TEST_CASE( test_nurbscurve3d_not_updated )
{
    base::geometry::NURBSCurve3D in;

    // Create 10 random points
    Eigen::Vector3d p;
    for (int i = 0; i < 10; ++i)
    {
        p += Eigen::Vector3d(2, 2, 0);
        in.addPoint( p );
    }
    wrappers::geometry::NURBSCurve3D w(in);
    base::geometry::NURBSCurve3D out = w;

    BOOST_REQUIRE(!in.getSISLCurve());
    BOOST_REQUIRE(!out.getSISLCurve());
    in.update();
    out.update();
    checkSameCurve(in, out);
}

BOOST_AUTO_TEST_CASE( test_nurbscurve3d_empty )
{
    base::geometry::NURBSCurve3D in;
    wrappers::geometry::NURBSCurve3D w(in);
    base::geometry::NURBSCurve3D out = w;
}

BOOST_AUTO_TEST_CASE( test_nurbscurve3d )
{
    base::geometry::NURBSCurve3D in;
    initRandomCurve(in);
    in.update();

    wrappers::geometry::NURBSCurve3D w(in);
    base::geometry::NURBSCurve3D out = w;

    BOOST_REQUIRE(in.getSISLCurve());
    BOOST_REQUIRE(out.getSISLCurve());

    checkSameCurve(in, out);
}

