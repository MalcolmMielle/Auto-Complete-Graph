#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 


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
	
}