#include <ros/ros.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/NDTMapMsg.h>
#include "ndt_map/ndt_conversions.h"
#include <ros/package.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ndt_mcl_init_node");
	ros::NodeHandle param("~");
	/*while( (int)ros::Time(0).toSec() == 0 and ros::ok()){
		std::cout << "Not good " << ros::Time(0) << std::endl;
	}*/

	std::string path_to_acg = ros::package::getPath("auto_complete_graph");
	std::string file = path_to_acg + "/jff_maps/basement2d_laser_gustav_map_r05.jff";
// 	map.loadFromJFF(file.c_str());
	double resolution = 0.2;
	auto mapGrid = new perception_oru::LazyGrid(resolution);
	perception_oru::NDTMap map(mapGrid);
	if(map.loadFromJFF(file.c_str()) < 0)
		std::cout << "File didn't load" << std::endl;
	std::cout << "File loaded" << std::endl;

	ndt_map::NDTMapMsg mapmsg;
	perception_oru::toMessage(&map, mapmsg, "world");

	ros::Publisher ndt_map_pub = param.advertise<ndt_map::NDTMapMsg>("ndt_map_init_mcl",10);

	while(ros::ok()){
		ros::spinOnce();
		std::cout << "Enter anything to publish the map" << std::endl;
		int a; std::cin >> a;
		ndt_map_pub.publish(mapmsg);
		std::cout << "Map Pubkished" << std::endl;
	}

	return 0;
}

