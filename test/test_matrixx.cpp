// \file  test_matrixx.cpp

#include <boost/test/auto_unit_test.hpp>
#include <base/eigen.h>
#include "base/wrappers/eigen.h"
#include "typekit/Opaques.hpp"

using namespace orogen_typekits;

BOOST_AUTO_TEST_CASE ( matrixxd_to_intermediate ) {

    base::MatrixXd m = base::MatrixXd::Random(3,6);
    
    wrappers::MatrixXd w;

    toIntermediate(w, m);

    BOOST_CHECK ( m.size() == w.data.size() );
    BOOST_CHECK ( m.rows() == w.rows );
    BOOST_CHECK ( m.cols() == w.cols );

    for ( int j = 0; j<m.cols(); j++)
        for ( int i = 0; i<m.rows(); i++)
            BOOST_CHECK( m(i,j) == w.data[i+j*m.rows()] );
}

BOOST_AUTO_TEST_CASE ( matrixxd_from_intermediate ) {

    wrappers::MatrixXd w;

    w.rows = 3;
    w.cols = 2;
    w.data.resize(6, 0.0);

    for ( int i = 0; i < 6; i++)
        w.data[i] = i * 1.37 - 2.0;

    base::MatrixXd m;

    fromIntermediate(m, w);

    BOOST_CHECK ( m.size() == w.data.size() );
    BOOST_CHECK ( m.rows() == w.rows );
    BOOST_CHECK ( m.cols() == w.cols );
    
    for ( int j = 0; j<m.cols(); j++)
        for ( int i = 0; i<m.rows(); i++)
            BOOST_CHECK( m(i,j) == w.data[i+j*m.rows()] );
}

BOOST_AUTO_TEST_CASE ( matrixxd ) {
    
    base::MatrixXd m = base::MatrixXd::Random(3,6), r;
    
    wrappers::MatrixXd w;

    toIntermediate(w, m);
    fromIntermediate(r, w);

    BOOST_CHECK( (m-r).norm() <= 1.0e-6 );

}

BOOST_AUTO_TEST_CASE ( vectorxd_to_intermediate ) {

    base::VectorXd v = base::VectorXd::Random(8);
    
    wrappers::VectorXd w;

    toIntermediate(w, v);

    BOOST_CHECK ( v.size() == w.data.size() );

    for ( int j = 0; j<v.size(); j++)
        BOOST_CHECK( v[j] == w.data[j] );
}

BOOST_AUTO_TEST_CASE ( vectorxd_from_intermediate ) {

    wrappers::VectorXd w;

    w.data.resize(7, 0.0);

    for ( int i = 0; i < 7; i++)
        w.data[i] = i * 1.37 - 2.0;

    base::VectorXd v;

    fromIntermediate(v, w);

    BOOST_CHECK ( v.size() == w.data.size() );
    for ( int j = 0; j<v.size(); j++)
        BOOST_CHECK( v[j] == w.data[j] );
    
}

BOOST_AUTO_TEST_CASE ( vectorxd ) {
    
    base::VectorXd v = base::VectorXd::Random(23), r;
    
    wrappers::VectorXd w;

    toIntermediate(w, v);
    fromIntermediate(r, w);

    BOOST_CHECK( (v-r).norm() <= 1.0e-6 );

}
