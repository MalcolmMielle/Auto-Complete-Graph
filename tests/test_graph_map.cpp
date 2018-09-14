#include "stdio.h"
#include "iostream"
#include "graph_map/graph_map.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "graph_map/graphfactory.h"
#include "ros/ros.h"
#include "Eigen/Geometry"
//#include "visualization/graph_plot.h"
#include "graph_map/graph_map_navigator.h"

using namespace std;
using namespace Eigen;

using namespace perception_oru;
using namespace graph_map;

int main(int argc, char **argv){



//    perception_oru::NDTMap* map = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
//    map->initialize(0.0,0.0,0.0,10,10,10);


std::cout << "Hellow 5 :(" << std::endl;

	//   ros::init(argc, argv, "testGraphLib");
	//   ros::NodeHandle n;
	string maptype = "ndt_map";

	//   n.param<std::string>("map_type",maptype,"ndt_map");
	cout<<"Testing graph_map with map type: "<<maptype<<endl;

	Eigen::Affine3d initPose=Affine3d::Identity();
	initPose.translation()<<2.5,2.5,0.01;//Create initiali pose of graph

	Eigen::Affine3d diff=Affine3d::Identity();

	diff= AngleAxisd(0.0*M_PI, Vector3d::UnitX())
	      * AngleAxisd(0.0*M_PI, Vector3d::UnitY())
	      * AngleAxisd(0.2*M_PI, Vector3d::UnitZ())*Translation3d(1,1,0);//Transformation between subsequent map nodes

	Matrix6d cov;
	Eigen::DiagonalMatrix<double,6> diag1;
	diag1.diagonal()<<0.15,0.15,0.15,0.01,0.01,0.01;
	cov=diag1; //Create covariance to represent uncertainty betweem mpde

	MapParamPtr mapParam_=GraphFactory::CreateMapParam(maptype);

	std::cout << "createddddddd map param" << std::endl;
	//
	GraphMapNavigatorParamPtr graph_param=GraphMapNavigatorParamPtr(new GraphMapNavigatorParam());
	std::cout << "createddddddd map navigator param" << std::endl;
	GraphMapNavigatorPtr graph =GraphMapNavigatorPtr(new GraphMapNavigator( initPose, mapParam_, graph_param));
	cout<<"size of graph :"<<graph->Size()<<endl;
	//
	//  graph->AddMapNode(diff,cov);
	//  graph->AddMapNode(diff,cov);
	//  graph->AddMapNode(diff,cov);
	//  cout<<"size of graph :"<<graph->Size()<<endl;
	//  GraphPlot::PlotPoseGraph(graph);
	//  cout<<graph->ToString()<<endl;
	// //  pcl::PointCloud<pcl::PointXYZ> cloud;

}



