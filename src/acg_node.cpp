#include <ros/ros.h>
#include "ndt_feature/NDTGraphMsg.h"


void gotGraph(const ndt_feature::NDTGraphMsg::ConstPtr msg){
	std::cout << "Got a new graph " << std::endl;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_complete_graph");
	ros::Subscriber ndt_graph_sub;
    ros::NodeHandle nh;
	
	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("ndt_graph", 10, gotGraph);
    
	while(ros::ok()){
// 		std::cout <<"SPIN" << std::endl;
		ros::spinOnce();
	}

    return 0;
}
