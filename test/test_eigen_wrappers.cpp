#include <boost/test/auto_unit_test.hpp>
#include "base/wrappers/eigen.h"
#include "Eigen/Core"

using namespace std;

BOOST_AUTO_TEST_CASE( test_matrix3d )
{
    Eigen::Matrix3d in = Eigen::Matrix3d::Identity();
    wrappers::Matrix3 w(in);
    Eigen::Matrix3d out = w;

    BOOST_REQUIRE(in == out);
}

