#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "auto_complete_graph/acg_conversion.hpp"


BOOST_AUTO_TEST_CASE(trying)
{
	
	cv::Mat src = cv::imread( "tests/grid1.png", CV_LOAD_IMAGE_COLOR ), grid1t;
	cv::cvtColor(src, grid1t, CV_RGB2GRAY );
	cv::Mat grid1;
	grid1t.convertTo(grid1, CV_16UC1);
	
	cv::Mat src2 = cv::imread( "tests/grid2.png", CV_LOAD_IMAGE_COLOR ), grid2t;
	cv::cvtColor(src2, grid2t, CV_RGB2GRAY );
	cv::Mat grid2;
	grid2t.convertTo(grid2, CV_16UC1);
	
	cv::Mat src3 = cv::imread( "tests/grid3.png", CV_LOAD_IMAGE_COLOR ), grid3t;
	cv::cvtColor(src3, grid3t, CV_RGB2GRAY );
	cv::Mat grid3;
	grid3t.convertTo(grid3, CV_16UC1);
	
	cv::Mat src4 = cv::imread( "tests/grid4.png", CV_LOAD_IMAGE_COLOR ), grid4t;
	cv::cvtColor(src4, grid4t, CV_RGB2GRAY );
	cv::Mat grid4;
	grid4t.convertTo(grid4, CV_16UC1);
	
// 	std::cout << grid2 << std::endl;

	grid_map::GridMap map;
	std::cout << "Res before init " << map.getResolution() << std::endl;
	map.setGeometry(grid_map::Length(1, 1), 0.5);
	grid_map::GridMap map2;
	map2.setGeometry(grid_map::Length(grid2.rows/2, grid2.cols/2), 0.5);
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid1, "combined", map, 0.0, 255);
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid2, "combined", map2, 0.0, 255);
	
	
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid1, "el", map, 0.0, 255);
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid2, "el2", map, 0.0, 255);
	
	map.add("sum");
	map["sum"] = map["el2"] + map["el"];

	cv::Mat originalImagePrtt;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map, "sum", CV_16UC1, 0.0, 1, originalImagePrtt);
	cv::imwrite("/home/malcolm/grid1sum.png", originalImagePrtt);	
	
	
	
	grid_map::GridMap map3;
	map3.setGeometry(grid_map::Length(1, 1), 0.5);
	grid_map::GridMap map4;
	map4.setGeometry(grid_map::Length(1, 1), 0.5);
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid3, "combined", map3, 0.0, 255);
	grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(grid4, "combined", map4, 0.0, 255);
	
	grid_map::GridMap modifiedMap;
	grid_map::GridMapCvProcessing::changeResolution(map2, modifiedMap, 1);
	
	cv::Mat originalImagePr;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(modifiedMap, "combined", CV_16UC1, 0.0, 1, originalImagePr);
	cv::imwrite("/home/malcolm/grid1r.png", originalImagePr);
	
	cv::Mat originalImageP;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map, "combined", CV_16UC1, 0.0, 1, originalImageP);
	cv::imwrite("/home/malcolm/grid1.png", originalImageP);
	
	cv::Mat originalImageP2;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map2, "combined", CV_16UC1, 0.0, 1, originalImageP2);
	cv::imwrite("/home/malcolm/grid2.png", originalImageP2);
	
	std::cout << "FUSING!!!!" << std::endl;
	grid_map::GridMap map_tmp;
	AASS::acg::fuseGridMap(map2, map4, map_tmp, "combined");
// 	AASS::acg::fuseGridMap(map, map2, "combined");
	
	std::cout << "DONE FUSING!!!!" << std::endl;
	
	cv::Mat originalImageP2f_tmp;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map_tmp, "combined", CV_16UC1, 0.0, 1, originalImageP2f_tmp);
	cv::imwrite("/home/malcolm/grid2f_tmp.png", originalImageP2f_tmp);
	
	
	exit(0);
	
	
	cv::Mat originalImageP2f;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map2, "combined", CV_16UC1, 0.0, 1, originalImageP2f);
	cv::imwrite("/home/malcolm/grid2f.png", originalImageP2f);
	
// 	map["sum"] = map["sum"] + map4["combined"];

	cv::Mat originalImagePrttt;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map4, "combined", CV_16UC1, 0.0, 1, originalImagePrttt);
	cv::imwrite("/home/malcolm/grid1sum4.png", originalImagePrttt);	
	
	
// 	AASS::acg::fuseGridMap(map, map2, "combined");
	
	std::vector<grid_map::GridMap> maps;
	maps.push_back(map);
	maps.push_back(map2);
	maps.push_back(map3);
	maps.push_back(map4);
	
	grid_map::GridMap map_fused;
	std::vector<std::string> layers;
	layers.push_back("combined");
	layers.push_back("combined");
	layers.push_back("combined");
	layers.push_back("combined");
	AASS::acg::fuseGridMap(maps, layers, map_fused, "/world", 0.5);
	
	cv::Mat originalImageP2fu;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map_fused, "combined", CV_16UC1, 0.0, 1, originalImageP2fu);
	cv::imwrite("/home/malcolm/grid2fused.png", originalImageP2fu);
	
}