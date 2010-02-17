#define BOOST_TEST_MODULE WrapperTypes
#include <boost/test/included/unit_test.hpp>

#include "base/wrappers/eigen.h"
#include "Eigen/Core"

using namespace std;

BOOST_AUTO_TEST_CASE( time_test )
{
    wrappers::Matrix3 m1( Eigen::Matrix3d::Identity() );
    Eigen::Matrix3d e1 = m1;

    cout << e1 << endl;
}

