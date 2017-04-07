/**
 * @file SURF_FlannMatcher
 * @brief SURF detector + descriptor + FLANN Matcher
 * @author A. Huaman
 */

#include "opencv2/opencv_modules.hpp"
#include <stdio.h>
# include "opencv2/core/core.hpp"
# include "opencv2/features2d/features2d.hpp"
# include "opencv2/highgui/highgui.hpp"
#include "das/CornerDetector.hpp"

//SIFT IS NON FREE :(
# include "opencv2/nonfree/features2d.hpp"

using namespace cv;

/**
 * @function main
 * @brief Main function
 */
int main( int argc, char** argv )
{
	
	cv::Mat src = cv::imread( "/home/malcolm/Pictures/aaa.png", 1 ), src_gray;
	cv::cvtColor( src, src_gray, CV_BGR2GRAY );
	
	cv::imshow("IMG", src_gray);
	
	cv::KeyPoint keypoint;
	keypoint.pt = cv::Point2f( src_gray.cols/2, src_gray.rows/2 );
	keypoint.size = 50 ;
	keypoint.angle = -1 ;
	keypoint.octave = 1 ;
	
	std::vector<cv::KeyPoint> keypoint_v;
	keypoint_v.push_back(keypoint);
	
	cv::Mat descriptors_1;
	cv::SiftDescriptorExtractor extractor;
	extractor.compute(src_gray, keypoint_v, descriptors_1);
	
	std::cout << descriptors_1.rows << " " << descriptors_1.cols << std::endl;
	assert(descriptors_1.rows == 1);
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );
				
}

