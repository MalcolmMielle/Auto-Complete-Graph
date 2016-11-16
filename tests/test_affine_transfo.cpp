#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "auto_complete_graph/PriorLoaderInterface.hpp"

cv::Point2d transf(const cv::Mat& warp_transfo, const cv::Mat point){
	//Matrix multiplication
	cv::Mat mat_out = warp_transfo * point;
	std::cout << "Mat out " << mat_out << std::endl;
	cv::Point2d point_out;
	point_out.x = mat_out.at<double>(0);
	point_out.y = mat_out.at<double>(1);
	return point_out;
}

BOOST_AUTO_TEST_CASE(trying)
{
	cv::Mat warp_transfo = (cv::Mat_<double>(2,3) << 1, 0, 0, 0, 1, 0);
	cv::Mat point = (cv::Mat_<double>(3,1) << 1, 1, 1);
		
	//Matrix multiplication
	auto point_out = transf(warp_transfo, point);
	std::cout << "Point out " << point_out << std::endl;
	
	BOOST_CHECK_EQUAL(cv::Point2d(1., 1.), point_out);
	
	//___________________________________________
	
	cv::Mat warp_transfo2 = (cv::Mat_<double>(2,3) << 1, 1, 0, 1, 1, 5);
	cv::Mat point2 = (cv::Mat_<double>(3,1) << -1.5, 1, 1);
		
	//Matrix multiplication
	auto point_out2 = transf(warp_transfo2, point2);
	std::cout << "Point out " << point_out2 << std::endl;
	
	BOOST_CHECK_EQUAL(cv::Point2d(-0.5, 4.5), point_out2);
	
	
	
	
}