#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 


#include "auto_complete_graph/covariance.hpp"

BOOST_AUTO_TEST_CASE(trying)
{

	double oldnorm = 12;
	double diff_norm = 6;
	
	double min = (oldnorm / 2);
	double max = oldnorm;
	double max_range = 1;
	double min_range = 0;
	
	double diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
	
	BOOST_CHECK_EQUAL(diff_norm_normalized, 1);


	diff_norm = 9;	
	diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
	
	BOOST_CHECK_EQUAL(diff_norm_normalized, 0.5);
	
	diff_norm = 3;
	min = 0;
	max = oldnorm;
	max_range = 1;
	min_range = 0;
	
	diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
	
	BOOST_CHECK_EQUAL(diff_norm_normalized, 0.75);
	
	diff_norm = 0;
	min = 0;
	max = 4.08483 / 2;
	max_range = 1;
	min_range = 0;
	
	diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
	
	BOOST_CHECK_EQUAL(diff_norm_normalized, 1);
	
	diff_norm = 4.08483 / 2;
	min = 0;
	max = 4.08483 / 2;
	max_range = 1;
	min_range = 0;
	
	diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
	
	BOOST_CHECK_EQUAL(diff_norm_normalized, 0);

	Eigen::Vector2d eigenvec; eigenvec << -0.00111417, -3.81661;
	std::pair<double, double> eigenval;
	eigenval.first = 0.001;
	eigenval.second = 0.01;
	
	std::cout << "out " << AASS::acg::getCovarianceVec(eigenvec, eigenval) << std::endl;
	std::cout << "inv " << AASS::acg::getCovarianceVec(eigenvec, eigenval).inverse() << std::endl;
	
	Eigen::Vector2d eigenvec2; eigenvec2 << -0.00111417, -3.81661;
	std::pair<double, double> eigenval2;
	eigenval2.first = 0.3;
	eigenval2.second = 0.01;
	
	std::cout << "out " << AASS::acg::getCovarianceVec(eigenvec2, eigenval2) << std::endl;
	std::cout << "inv " << AASS::acg::getCovarianceVec(eigenvec2, eigenval2).inverse() << std::endl;
	
	Eigen::Vector2d eigenvec3; eigenvec3 << -0.00111417, -3.81661;
	std::pair<double, double> eigenval3;
	eigenval3.first = 0.05;
	eigenval3.second = 0.01;
	
	std::cout << "out " << AASS::acg::getCovarianceVec(eigenvec3, eigenval3) << std::endl;
	std::cout << "inv " << AASS::acg::getCovarianceVec(eigenvec3, eigenval3).inverse() << std::endl;
}
