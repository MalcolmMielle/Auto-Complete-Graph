#include <ros/ros.h>
#include "ndt_feature/NDTGraphMsg.h"
#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/ndtgraph_conversion.h"
#include "auto_complete_graph/ACG.hpp"
#include "auto_complete_graph/OptimizableAutoCompleteGraph.hpp"
#include "auto_complete_graph/Basement.hpp"
#include "auto_complete_graph/BasementFull.hpp"
#include "auto_complete_graph/VisuACG.hpp"


ros::Publisher map_pub_;




inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose) {

  Eigen::Affine3d map_origin;
  tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
  Eigen::Affine3d new_map_origin = pose*map_origin;
  tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}



void gotGraph(const ndt_feature::NDTGraphMsg::ConstPtr msg, AASS::acg::AutoCompleteGraph* acg, AASS::acg::VisuAutoCompleteGraph& visu){
	std::cout << "Got a new graph " << std::endl;
	
	ndt_feature::NDTFeatureGraph graph;
	
	std::string frame;
	ndt_feature::msgToNDTGraph(*msg, graph, frame);	
	

	acg->updateNDTGraph(graph);
	
	
	std::string file_out = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/acg_";
	std::ostringstream convert;   // stream used for the conversion
	convert << graph.getNbNodes(); 
	file_out = file_out + convert.str();
	file_out = file_out + "nodes.g2o";
	acg->getGraph().save(file_out.c_str());
	
	std::cout << "saved to " << file_out << std::endl;
	
// 	exit(0);
		
}



void gotGraphandOptimize(const ndt_feature::NDTGraphMsg::ConstPtr msg, AASS::acg::AutoCompleteGraph* oacg, AASS::acg::VisuAutoCompleteGraph& visu){
	std::cout << "Got a new graph " << std::endl;
	
	ndt_feature::NDTFeatureGraph graph;
	
	std::string frame;
	ndt_feature::msgToNDTGraph(*msg, graph, frame);	
	

	oacg->updateNDTGraph(graph, true, 0.5);
	
	
	std::string file_out = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/oacg_before_";
	std::ostringstream convert;   // stream used for the conversion
	convert << graph.getNbNodes(); 
	file_out = file_out + convert.str();
	file_out = file_out + "nodes.g2o";
	oacg->getGraph().save(file_out.c_str());
	
	std::cout << "saved to " << file_out << std::endl;
	
// 	oacg->initializeOptimization();
// 	oacg->initialGuess();
	//Prepare the graph : marginalize + initializeOpti
	oacg->getGraph().setFirst();
	oacg->prepare();
	oacg->optimize();
	
	std::string file_out_after = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/oacg_after_";
	std::ostringstream convert_after;   // stream used for the conversion
	convert_after << graph.getNbNodes(); 
	file_out_after = file_out_after + convert.str();
	file_out_after = file_out_after + "nodes.g2o";
	oacg->getGraph().save(file_out_after.c_str());
	
// 	visu.toRviz(*oacg);
	
// 	visu.updateRviz();
	visu.updateRviz();
	
	nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
	nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
	AASS::acg::ACGtoOccupancyGrid(*oacg, occ_outt);
	grid_map::GridMap gridMap({"all"});
	grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_outt, "all", gridMap);

	
	std::cout << "WELLL HERE IT IS : " << occ_outt->info.origin.position << " ori " << occ_outt->info.origin.orientation << std::endl << std::endl;	
	
	cv::Mat originalImageP;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
	std::string file_outg = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/occupancygrid_full_";
	std::ostringstream convertg;   // stream used for the conversion
	convertg << oacg->getRobotNodes().size(); 
	file_outg = file_outg + convert.str();
	file_outg = file_outg + "nodes.png";

	cv::imwrite(file_outg, originalImageP);
	
	
	
	
	
	
	
// 	nav_msgs::OccupancyGrid omap; 
// 	lslgeneric::toOccupancyGrid(graph.getMap(), omap, 0.4, "/world");
// 	moveOccupancyMap(omap, graph.getT());
// 	
// 	map_pub_.publish(omap);
	
// 	visu.updateRviz();
	
// 	std::cout << "saved to " << file_out_after << std::endl;
	
// 	exit(0);
		
}




int main(int argc, char **argv)
{
	
	AASS::acg::BasementFull basement;
	basement.extractCornerPrior();
// 	basement.transformOntoSLAM();
// 	auto graph_priortmp = basement.getGraph();
// 	ndt_feature::NDTFeatureGraph graph;
// 	AASS::acg::AutoCompleteGraph acgtmp(g2o::SE2(0.2, 0.1, -0.1),
// 		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
// 		DEG2RAD(2.), 				//Rotation noise for robot
// 		Eigen::Vector2d(0.05, 0.05), //Landmarks noise
// 		Eigen::Vector2d(1, 0.01), //Prior noise
// 		DEG2RAD(2.), //Prior rot	
// 		Eigen::Vector2d(0.2, 0.2) //Link noise,
// 
// 	);
// 	acgtmp.addPriorGraph(graph_priortmp);
// 	std::string file_outt = "/home/malcolm/ACG_folder/acg_priortmp.g2o";
// 	acgtmp.getGraph().save(file_outt.c_str());
// 	std::cout << "saved to " << file_outt << std::endl;
	
	basement.transformOntoSLAM();
	auto graph_prior = basement.getGraph();
	
	
	//TODO : test no sensor offset
	AASS::acg::AutoCompleteGraph acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.5, 0.5), //Landmarks noise
		Eigen::Vector2d(1, 0.01), //Prior noise
		DEG2RAD(2.), //Prior rot							 
		Eigen::Vector2d(0.002, 0.002) //Link noise,
	);
	
	acg.addPriorGraph(graph_prior);
	std::string file_out = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/acg_0_prior.g2o";
	acg.getGraph().save(file_out.c_str());
	std::cout << "saved to " << file_out << std::endl;
	
	//TODO : test no sensor offset
// 	AASS::acg::AutoCompleteGraph oacg(g2o::SE2(0.2, 0.1, -0.1),
// 		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
// 		DEG2RAD(2.), 				//Rotation noise for robot
// 		Eigen::Vector2d(0.05, 0.05), //Landmarks noise
// 		Eigen::Vector2d(1, 0.01), //Prior noise
// 		DEG2RAD(2.), //Prior rot							 
// 		Eigen::Vector2d(0.2, 0.2) //Link noise,
// 	);
	
	
	std::ifstream infile("/home/malcolm/ACG_folder/param.txt");
			
	double a, b, c;
	infile >> a >> b;
	Eigen::Vector2d tn; tn << a, b;

	infile >> a;
	double rn = a;
	
	infile >> a >> b;
	Eigen::Vector2d ln; ln << a, b;

	infile >> a >> b;
	Eigen::Vector2d pn; pn << a, b;

	infile >> a;
	double pr = a;
	infile >> a >> b;
	Eigen::Vector2d lin; lin << a, b;

	AASS::acg::AutoCompleteGraph oacg(g2o::SE2(0.2, 0.1, -0.1), "/home/malcolm/ACG_folder/param.txt");
// 	AASS::acg::AutoCompleteGraph oacg(g2o::SE2(0.2, 0.1, -0.1),
// 		tn, //Robot translation noise
// 		rn, 				//Rotation noise for robot
// 		ln, //Landmarks noise
// 		pn, //Prior noise
// 		DEG2RAD(2.), //Prior rot							 
// 		Eigen::Vector2d(0.2, 0.2) //Link noise,
// 	);
	oacg.addPriorGraph(graph_prior);

	oacg.useUserCovForPrior(false);
	oacg.useUserCovForRobotPose(true);
	// 	std::string file_out = "/home/malcolm/ACG_folder/acg_0_prior.g2o";
// 	oacg.getGraph().save(file_out.c_str());
// 	std::cout << "saved to " << file_out << std::endl;
	
// 	exit(0);
	
    ros::init(argc, argv, "auto_complete_graph_rviz_small_optimi");
	ros::Subscriber ndt_graph_sub;
    ros::NodeHandle nh("~");
	
// 	AASS::acg::VisuAutoCompleteGraph visu(&oacg);
	AASS::acg::VisuAutoCompleteGraph visu(&oacg, nh);
	visu.setImageFileNameOut("/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/optimization_rviz_small");
	
// 	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("ndt_graph", 10, boost::bind(&gotGraph, _1, &acg, visu));
	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("/ndt_graph", 10, boost::bind(&gotGraphandOptimize, _1, &oacg, visu));
    
	map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_grid", 1000);
	
	while(ros::ok()){
// 		std::cout <<"SPIN auto_complete" << std::endl;
		ros::spinOnce();
		visu.updateRvizV2();
// 		std::cout << oacg.getLinkEdges().size()<< std::endl;
	}

    return 0;
}
