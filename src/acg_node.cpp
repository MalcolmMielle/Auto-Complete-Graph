#include <ros/ros.h>
#include "ndt_feature/NDTGraphMsg.h"
#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/ndtgraph_conversion.h"
#include "auto_complete_graph/ACG.hpp"
#include "auto_complete_graph/Basement.hpp"


void gotGraph(const ndt_feature::NDTGraphMsg::ConstPtr msg, AASS::acg::AutoCompleteGraph* acg){
	std::cout << "Got a new graph " << std::endl;
	
	ndt_feature::NDTFeatureGraph graph;
	
	std::string frame;
	ndt_feature::msgToNDTGraph(*msg, graph, frame);	
	

	acg->updateNDTGraph(graph);
	
	
	std::string file_out = "/home/malcolm/ACG_folder/acg_";
	std::ostringstream convert;   // stream used for the conversion
	convert << graph.getNbNodes(); 
	file_out = file_out + convert.str();
	file_out = file_out + "nodes.g2o";
	acg->getGraph().save(file_out.c_str());
	
	std::cout << "saved to " << file_out << std::endl;
	
// 	exit(0);
		
}



int main(int argc, char **argv)
{
	
	AASS::acg::Basement basement;
	basement.extractCornerPrior();
// 	basement.transformOntoSLAM();
	auto graph_priortmp = basement.getGraph();
	ndt_feature::NDTFeatureGraph graph;
	AASS::acg::AutoCompleteGraph acgtmp(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.0005, 0.0005), //Landmarks noise
		Eigen::Vector2d(1, 0.001), //Prior noise
		Eigen::Vector2d(0.2, 0.2) //Link noise,

	);
	acgtmp.addPriorGraph(graph_priortmp);
	std::string file_outt = "/home/malcolm/ACG_folder/acg_priortmp.g2o";
	acgtmp.getGraph().save(file_outt.c_str());
	std::cout << "saved to " << file_outt << std::endl;
	
	basement.transformOntoSLAM();
	auto graph_prior = basement.getGraph();
	
	AASS::acg::AutoCompleteGraph acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.0005, 0.0005), //Landmarks noise
		Eigen::Vector2d(1, 0.001), //Prior noise
		Eigen::Vector2d(0.2, 0.2) //Link noise,

	);
	acg.addPriorGraph(graph_prior);
	std::string file_out = "/home/malcolm/ACG_folder/acg_0_prior.g2o";
	acg.getGraph().save(file_out.c_str());
	std::cout << "saved to " << file_out << std::endl;
	
// 	exit(0);
	
    ros::init(argc, argv, "auto_complete_graph");
	ros::Subscriber ndt_graph_sub;
    ros::NodeHandle nh;
	
	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("ndt_graph", 10, boost::bind(&gotGraph, _1, &acg));
    
	while(ros::ok()){
// 		std::cout <<"SPIN auto_complete" << std::endl;
		ros::spinOnce();
	}

    return 0;
}
