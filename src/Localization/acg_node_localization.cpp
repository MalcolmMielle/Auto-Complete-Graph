#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sys/time.h>
#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>


#include "ndt_feature/NDTGraphMsg.h"
#include "ndt_feature/ndt_feature_graph.h"
#include "ndt_feature/ndtgraph_conversion.h"
#include "auto_complete_graph/Localization/ACG_localization.hpp"
#include "auto_complete_graph/OptimizableAutoCompleteGraph.hpp"
#include "auto_complete_graph/Basement.hpp"
#include "auto_complete_graph/BasementFull.hpp"
#include "auto_complete_graph/Localization/VisuACGLocalization.hpp"
#include "auto_complete_graph/Localization/GoodMatchingLocalization.hpp"

#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


bool optimize_prior = true;
bool abort_f = false;
bool testing_pause = false;

ros::Publisher map_pub_;
ros::Publisher last_ndtmap_full;
ros::Publisher occupancy_grid;
ros::Publisher occ_send;
ros::Publisher prior_cloud, prior_ndt;
ros::Publisher acg_gdim;
ros::Publisher acg_gdim_om;

ros::Publisher last_grid_map;
ros::Publisher last_ndtmap;

ros::Time timef;

int count = 0;

bool new_node = false;
bool was_init = false;

std::vector<double> time_extract_corner_ndt;
std::vector<double> time_opti;
std::vector<double> all_node_times;

nav_msgs::OccupancyGrid::Ptr occ_map_global;

bool updated = true;


inline bool exists_test3 (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}


tf::StampedTransform getPoseTFTransform(const std::string& base_frame, const std::string& to_frame, ros::Time time=ros::Time(0)){
	std::cout << "GEt pose " << std::endl;
	tf::TransformListener listener;
	tf::StampedTransform transform;
// 	int i = 0;
// 	while(i < 10){
// 	++i;
	try {
		time=ros::Time(0);
		bool hunm = listener.waitForTransform(base_frame, to_frame, time, ros::Duration(1.0));
		listener.lookupTransform(base_frame, to_frame, time, transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
		exit(0);
	}
	return transform;
}

nav_msgs::OccupancyGrid::Ptr createOccupancyMap(const AASS::acg::AutoCompleteGraphLocalization& oacg){

	//std::cout << "Creating occ grid" << std::endl;
	//grid_map::GridMap gridMap;
	//AASS::acg::ACGToGridMap(oacg, gridMap);
	//nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
	//nav_msgs::OccupancyGrid::Ptr occ_out(omap_tmp);
	//grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "all", 0, 1, *occ_out);


	//nav_msgs::OccupancyGrid::Ptr omap_occ;

	nav_msgs::OccupancyGrid* omap_tmp = new nav_msgs::OccupancyGrid();
	nav_msgs::OccupancyGrid::Ptr occ_out(omap_tmp);
	int size_rl = acg.getRobotPoseLocalization().size();
	if(size_rl > 0){
		perception_oru::toOccupancyGrid(acg.getRobotPoseLocalization()[size_rl - 1]->getMap().get(), *occ_out, 0.1, "/world");
	}
	std::cout << "Occupancy grid sent ! " << std::endl;
	return occ_out;

}

void sendMapAsOcc(const AASS::acg::AutoCompleteGraphLocalization& oacg){
	
	if(updated == true){
		occ_map_global = createOccupancyMap();
		updated = false;
	}
	//Just to make sure
	occ_send.publish<nav_msgs::OccupancyGrid>(*occ_map_global);
}

void latchOccGrid(const std_msgs::Bool::ConstPtr msg, AASS::acg::AutoCompleteGraphLocalization* oacg){
	if(msg->data == true){
		sendMapAsOcc(*oacg);
	}
}

void publishPriorNDT(const AASS::acg::AutoCompleteGraphLocalization& oacg){

	//		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_prior = AASS::acg::ACGPriortoPointCloud<AASS::acg::AutoCompleteGraphPriorSE2>(*oacg.getPrior(), 0.1, 0.1/4);
//		sensor_msgs::PointCloud2 pcl_prior_msg;
//		pcl::toROSMsg<pcl::PointXYZ>(*pcl_prior, pcl_prior_msg);
//		pcl_prior_msg.header.frame_id = "world";
//		pcl_prior_msg.header.stamp = ros::Time::now();
//		prior_cloud.publish(pcl_prior_msg);

	perception_oru::NDTMap* ndt_prior = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
	AASS::acg::ACGPriorToNDTMap<AASS::acg::AutoCompleteGraphPriorXY>(*oacg.getPrior(), *ndt_prior, oacg.getZElevation(), 0.1);

//		auto allcells = ndt_prior->getAllCellsShared();
//		assert(allcells.size() > 0);

	ndt_map::NDTMapMsg priormapmsg;
	perception_oru::toMessage(ndt_prior, priormapmsg, "world");
	prior_ndt.publish(priormapmsg);

	delete ndt_prior;
}



void publishPriorNDT(const std_msgs::Bool::ConstPtr msg, AASS::acg::AutoCompleteGraphLocalization& oacg) {
	publishPriorNDT(oacg);
	if(optimize_prior == false){
		oacg.clearPrior();
	}
}

void publishACGOM(const AASS::acg::AutoCompleteGraphLocalization& oacg){

	//Puclish message for GDIM
	auto_complete_graph::ACGMaps mapmsg;
	std::cout << "PUSH acg maps message" << std::endl;
	AASS::acg::ACGToACGMapsMsg(oacg, mapmsg);
	acg_gdim.publish(mapmsg);


	auto_complete_graph::ACGMapsOM mapmsg_om;
	std::cout << "PUSH acg maps OM message" << std::endl;
	AASS::acg::ACGToACGMapsOMMsg(oacg, mapmsg_om);
	acg_gdim_om.publish(mapmsg_om);

	//Publish the last grid map as a message to make sure that they look like something
	int size_g = mapmsg_om.ndt_maps_om.size();
	if(size_g > 0) {
		std::cout << "Last grid map" << std::endl;
		last_grid_map.publish(mapmsg_om.ndt_maps_om[size_g - 1]);

		//Publish last occ grid to make sure that they look like something
		int size_o = mapmsg.ndt_maps.maps.size();
// 				nav_msgs::OccupancyGrid* omap = new nav_msgs::OccupancyGrid();
// 					initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
//                 perception_oru::toOccupancyGrid(&mapmsg.ndt_maps.maps[size_o -1], *omap, 0.1, "/world");
		std::cout << "Last ndtmap" << std::endl;
		last_ndtmap.publish(mapmsg.ndt_maps.maps[size_o - 1]);


	}
}


void publishACGOM(const std_msgs::Bool::ConstPtr msg, AASS::acg::AutoCompleteGraphLocalization& oacg) {
	publishACGOM(oacg);
	if(optimize_prior == false){
		oacg.clearPrior();
	}
}


inline void exportResultsGnuplot(const std::string& file_out){
	
		/*** TIMES ***/
// 	std::vector<double> time_extract_corner_ndt;
// 	std::vector<double> time_opti;
// 	std::vector<double> all_node_times;
	
	boost::filesystem::path p(file_out);
	std::string name = p.filename().stem().string();
	
	std::string result_file = file_out;
	std::ofstream myfile;
	myfile.open (result_file, std::ios::out | std::ios::app);
	
	if (myfile.is_open())
	{
		myfile << "Full time\n";
		
		double mean_times = 0 ;
		for(auto it = all_node_times.begin() ; it != all_node_times.end() ; ++it){
			mean_times += *it;
			myfile << *it << "\n";
		}
		mean_times = mean_times / all_node_times.size();
		
		myfile << "\nExtract time\n";
		
		double mean_extract = 0 ;
		for(auto it = time_extract_corner_ndt.begin() ; it != time_extract_corner_ndt.end() ; ++it){
			mean_extract += *it;
			myfile << *it << "\n";
		}
		mean_extract = mean_extract / time_extract_corner_ndt.size();
		
		myfile << "\nOpti time\n";
		
		double mean_opti = 0 ;
		for(auto it = time_opti.begin() ; it != time_opti.end() ; ++it){
			mean_opti += *it;
			myfile << *it << "\n";
		}
		mean_opti = mean_opti / time_opti.size();
		
		myfile << "\nMean time, mean extract, mean opti\n" << mean_times << " " << mean_extract << " " << mean_opti << "\n";
		myfile.close();
	}
	else std::cout << "Unable to open file";
}

inline double getTime() //in millisecond
{
	//assuming unix-type systems
	//timezone tz;
	timeval  tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec*1000000+tv.tv_usec)*1.0/1000;
}

inline void printImages(AASS::acg::AutoCompleteGraph* oacg){
	nav_msgs::OccupancyGrid* omap_tmpt = new nav_msgs::OccupancyGrid();
	nav_msgs::OccupancyGrid::Ptr occ_outt(omap_tmpt);
	AASS::acg::ACGtoOccupancyGrid(*oacg, occ_outt);
	grid_map::GridMap gridMap({"all"});
	grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_outt, "all", gridMap);
// 
// 	
// 	std::cout << "WELLL HERE IT IS : " << occ_outt->info.origin.position_in_robot_frame << " ori " << occ_outt->info.origin.orientation << std::endl << std::endl;
// 	
	cv::Mat originalImageP;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap, "all", CV_16UC1, 0.0, 1, originalImageP);
	std::string file_outg = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/occupancygrid_full_";
	std::ostringstream convert;   // stream used for the conversion
	convert << oacg->getRobotNodes().size(); 
	
	std::ostringstream count_str;   // stream used for the conversion
	count_str << count; 
	file_outg = file_outg + convert.str();
	file_outg = file_outg + "nodes_";
	file_outg = file_outg + count_str.str();
	file_outg = file_outg + ".png";

	cv::imwrite(file_outg, originalImageP);
	
	nav_msgs::OccupancyGrid* omap_tmpt_partial = new nav_msgs::OccupancyGrid();
	nav_msgs::OccupancyGrid::Ptr occ_outt_partial(omap_tmpt_partial);
	AASS::acg::ACGtoOccupancyGrid(*oacg, occ_outt_partial, oacg->getRobotNodes().size() - 1);
	grid_map::GridMap gridMap_partial({"all"});
	grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_outt_partial, "all", gridMap_partial);
// 
// 	
	std::cout << "WELLL HERE IT IS : " << occ_outt_partial->info.origin.position << " ori " << occ_outt_partial->info.origin.orientation << std::endl << std::endl;	
// 	
	cv::Mat originalImageP_partial;
	grid_map::GridMapCvConverter::toImage<unsigned short, 1>(gridMap_partial, "all", CV_16UC1, 0.0, 1, originalImageP_partial);
	std::string file_outg_partial = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/occupancygrid_full_partial_";
	std::ostringstream convertg_partial;   // stream used for the conversion
	convertg_partial << oacg->getRobotNodes().size(); 
	file_outg_partial = file_outg_partial + convert.str();
	file_outg_partial = file_outg_partial + "nodes_";
	file_outg_partial = file_outg_partial + count_str.str();
	file_outg_partial = file_outg_partial + ".png";
	
// 
	cv::imwrite(file_outg_partial, originalImageP_partial);
}

inline void moveOccupancyMap(nav_msgs::OccupancyGrid &occ_grid, const Eigen::Affine3d &pose) {

  Eigen::Affine3d map_origin;
  tf::poseMsgToEigen(occ_grid.info.origin, map_origin);
  Eigen::Affine3d new_map_origin = pose*map_origin;
  tf::poseEigenToMsg(new_map_origin, occ_grid.info.origin);
}



void gotGraph(const ndt_feature::NDTGraphMsg::ConstPtr msg, AASS::acg::AutoCompleteGraph* acg, AASS::acg::VisuAutoCompleteGraphLocalization& visu){
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


void testMsg(const ndt_feature::NDTGraphMsg::ConstPtr msg){
	abort_f = true;
	std::string frame;
	ndt_feature::NDTFeatureGraph graph;
	ndt_feature::msgToNDTGraph(*msg, graph, frame);
	
	ndt_map::NDTMapMsg msgio;
// 			ATTENTION Frame shouldn't be fixed
	bool good = perception_oru::toMessage(graph.getNode(0).map->map, msgio, "/world");
// 			perception_oru::NDTMap* map_copy = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution));
	
	perception_oru::NDTMap* map_copy;
	perception_oru::LazyGrid* lz;
// 			bool good = perception_oru::fromMessage(lz, fuser.map, m.map, frame);
	bool good2 = perception_oru::fromMessage(lz, map_copy, msgio, frame, true);
	std::shared_ptr<perception_oru::NDTMap> s(map_copy);
	
// 	delete map_copy;
}



void gotGraphandOptimize(const auto_complete_graph::GraphMapLocalizationMsg::ConstPtr msg, AASS::acg::AutoCompleteGraphLocalization* oacg, AASS::acg::VisuAutoCompleteGraphLocalization& visu){
// void gotGraphandOptimize(const ndt_feature::NDTGraphMsg::ConstPtr msg, AASS::acg::AutoCompleteGraph* oacg){
// 	try{

		updated = true;
		new_node = true;
	// 	abort_f = true;
		std::cout << "Got a new graph " << std::endl;
		
		ros::Time start = ros::Time::now();
		
// 		ndt_feature::NDTFeatureGraph graph;
		
		std::string frame;
		//ATTENTION: THE BAD GUY !
// 		ndt_feature::msgToNDTGraph(*msg, graph, frame);	
		
		
		ros::Time start_corner = ros::Time::now();
		oacg->updateNDTGraph(*msg);
		ros::Time end_corner = ros::Time::now();

		std::cout << "MAP UPDATED in main" << std::endl;
		
		double corner_extract_tt = (start_corner - end_corner).toSec();
		time_extract_corner_ndt.push_back(corner_extract_tt);
		
//		if(oacg->getRobotNodes().size() > 0){

		if(testing_pause) {
			std::cout << "RVIZ " << std::endl;
			visu.updateRviz(*oacg);
			std::cout << "RVIZ DONE" << std::endl;
		}
		
		// 	std::string file_out = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/oacg_before_";
		// 	std::ostringstream convert;   // stream used for the conversion
		// 	convert << graph.getNbNodes(); 
		// 	file_out = file_out + convert.str();
		// 	file_out = file_out + "nodes.g2o";
		// 	oacg->getGraph().save(file_out.c_str());
		// 	
		// 	std::cout << "saved to " << file_out << std::endl;
			
			/*** image****/
		// 	visu.updateRvizNoNDT();
		// 		if(was_init == true){
		// 		int a;
		// 		std::cout << "PRESS ANYTHING NOT OPTIMISED YET" << std::endl;
		// 		std::cin >>a;
				
			// 	oacg->initializeOptimization();
			// 	oacg->initialGuess();
				
				//Prepare the graph : marginalize + initializeOpti
				
				ros::Time start_opti = ros::Time::now();
				bool optiquest = true;
				if(testing_pause) {
					std::cout << "Optimize ?" << std::endl;
					std::cin >> optiquest;
				}
				if( /*oacg->checkAbleToOptimize() &&*/  optiquest) {
					oacg->setFirst();
					oacg->prepare();
					oacg->optimize();

				}
				ros::Time end_opti = ros::Time::now();	
				double opti = (start_opti - end_opti).toSec();
				time_opti.push_back(opti);
				
		// 		if(was_init == true){
					
		// 		}

				if(optimize_prior == true) {
					std::cout << "Publishing the new prior ndt map" << std::endl;
					publishPriorNDT(*oacg);
				}

//				std::cout << "Publish an occupancy grid for you Asif :3 ! " << std::endl;
//				sendMapAsOcc(*oacg);


				std::cout << "RVIZ " << std::endl;
				visu.updateRviz(*oacg);
				publishACGOM(*oacg);
				std::cout << "RVIZ DONE" << std::endl;

				if(testing_pause) {
					std::cout << "Result of optimization. Enter a number to continue" << std::endl;
					int aaa;
					std::cin >> aaa;
				}
		// 		std::cout << "PRESS ANYTHING OPTIMISED" << std::endl;
		// 		std::cin >>a;
				
			// 	printImages(oacg);
			// 	std::string file_out_after = "/home/malcolm/ACG_folder/ACG_RVIZ_SMALL/oacg_after_";
			// 	std::ostringstream convert_after;   // stream used for the conversion
			// 	convert_after << graph.getNbNodes(); 
			// 	file_out_after = file_out_after + convert.str();
			// 	file_out_after = file_out_after + "nodes.g2o";
			// 	oacg->getGraph().save(file_out_after.c_str());
				
			// 	visu.toRviz(*oacg);
				
			// 	visu.updateRviz();
			// 	visu.updateRviz();
			// 	

				timef = ros::Time::now();
				all_node_times.push_back((timef - start).toSec());

			std::cout << "*************** DESCRIPTION **************" << std::endl;
			oacg->print();
			std::cout << "*************** DESCRIPTION **************" << std::endl;

				
			// 	nav_msgs::OccupancyGrid omap; 
			// 	perception_oru::toOccupancyGrid(graph.getMap(), omap, 0.4, "/world");
			// 	moveOccupancyMap(omap, graph.getT());
			// 	
			// 	map_pub_.publish(omap);
				
			// 	visu.updateRviz();
				
			// 	std::cout << "saved to " << file_out_after << std::endl;
				
			// 	exit(0);
		// 	}
		// 		else{
		// 			visu.updateRvizNoNDT();
		// 		}
//		}
// 	}
// 	catch(...){
// 		
// 		std::cout << "Error " << std::endl;
// 		visu.updateRviz();
// // 		int a;
// // 		std::cin >> a;
// 	}
}



void initAll(AASS::acg::AutoCompleteGraphLocalization& oacg, AASS::acg::RvizPointsLocalization& initialiser, AASS::acg::PriorLoaderInterface& priorloader){
	
	std::cout << "INIT ALL" << std::endl;
	std::vector<cv::Point2f> slam_pt;
	std::vector<cv::Point2f> prior_pt;
	auto match = initialiser.getMatches();
	for(auto it = match.begin() ; it != match.end() ; ++it){

		auto pose = it->getPriorPoint();
		prior_pt.push_back(pose);
		
		auto pose2de = it->getLandmarkPoint();
		slam_pt.push_back(pose2de);
	}
	
	priorloader.initialize(slam_pt, prior_pt);
	
	//We use the already registered points
// 	priorloader.extractCornerPrior();
	priorloader.transformOntoSLAM();
	auto graph_prior = priorloader.getGraph();	
	
	std::cout << "clear prior" << std::endl;
	oacg.clearPrior();
	std::cout << "add prior" << std::endl;
	oacg.addPriorGraph(graph_prior);
	
	initialiser.clear();

	publishPriorNDT(oacg);
	
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_complete_graph_rviz_small_optimi");
	ros::Subscriber ndt_graph_sub;
	ros::Subscriber call_for_publish_occ, publish_prior_ndt, publish_acg_om_maps;
    ros::NodeHandle nh("~");

	/**************PARAMETERS**************/
	nh.param("pause_for_testing",testing_pause,false);
	bool use_prior = true;
	nh.param("use_prior",use_prior,true);
	bool use_robot_maps = true;
	nh.param("use_robot_maps",use_robot_maps,true);
	bool use_corner = true;
	nh.param("use_corner",use_corner,true);
	bool use_corner_orientation = true;
	nh.param("use_corner_orientation",use_corner_orientation,true);
	bool own_registration = true;
	nh.param("own_registration",own_registration,true);
	bool link_to_prior = true;
	nh.param("link_to_prior",link_to_prior,true);
	bool use_corner_covariance = true;
	nh.param("corner_covariance",use_corner_covariance,true);
	bool use_covariance_to_find_links = false;
	nh.param("covariance_to_find_links",use_covariance_to_find_links,false);
	bool use_mcl_observation_on_prior;
	nh.param("mcl_observation_on_prior",use_mcl_observation_on_prior,false);
	///@brief Add link in between ndt corner and prior corner based on a distance threshold.
	bool use_links_prior_classic_ssrr;
	nh.param("links_prior_classic_ssrr",use_links_prior_classic_ssrr,false);
//	bool optimize_prior;
	nh.param("optimize_prior",optimize_prior,true);
	bool use_mcl_cov_to_find_prior_observed;
	nh.param("use_mcl_cov_to_find_prior_observed",use_mcl_cov_to_find_prior_observed,false);
	double gaussian_scale = 1;
	nh.param<double>("gaussian_scaling_factor", gaussian_scale, 0);
	double threshold_score_link_creation = 0.5;
	nh.param<double>("threshold_score_link_creation", threshold_score_link_creation, 0.5);
	if(threshold_score_link_creation > 1 || threshold_score_link_creation < 0){
		std::cout << "threshold_score_link_creation needs to be between 0 and 1. Fix it" << std::endl;
		return 0;
	}
	std::string world_frame;
	nh.param<std::string>("world_frame",world_frame,"/world");
	std::string sensor_frame;
	nh.param<std::string>("sensor_frame",sensor_frame,"/velodyne");
	double deviation = 0;
	nh.param<double>("deviation", deviation, 0);
	double angle = 0;
	nh.param<double>("angle", angle, 0);
	double scale = 1;
	nh.param<double>("scale", scale, 1.0);
	cv::Point2f center(0, 0);


	double max_deviation_corner_in_prior = 1;
	nh.param<double>("max_deviation_corner_in_prior", max_deviation_corner_in_prior, 45 * 3.14159 / 180);


	std::string prior_file = "";
	nh.param<std::string>("prior_file",prior_file,"/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/tests/emergbasement_flipped_nodoor.png");

//	if(argc > 1){
//		deviation = strtod(argv[1], NULL);
//		if(argc > 2){
//			angle = strtod(argv[2], NULL);
//			if(argc > 3){
//				scale = strtod(argv[3], NULL);
//				if(argc > 5){
//					center.x = strtod(argv[4], NULL);
//					center.y = strtod(argv[5], NULL);
//				}
//			}
//		}
//	}

	AASS::acg::PriorLoaderInterface priormap(prior_file, deviation, angle, scale, center);
	priormap.setMaxDeviationForCornerInRad(max_deviation_corner_in_prior);
	priormap.extractCornerPrior();
	priormap.transformOntoSLAM();
	auto graph_prior = priormap.getGraph();

	auto sensor_pose = getPoseTFTransform(world_frame, sensor_frame);
	//Create graph instance
	//
	std::string path_to_acg = ros::package::getPath("auto_complete_graph");
	std::string param = path_to_acg + "/ACG_folder/param.txt";
	AASS::acg::AutoCompleteGraphLocalization oacg(g2o::SE2(0.2, 0.1, -0.1), param);
	
	//Use corner orientation ?
	oacg.useCornerOrientation(use_corner_orientation);
	oacg.extractCorners(use_corner);
	oacg.doOwnRegistrationBetweenSubmaps(own_registration);
	oacg.setZElevation(sensor_pose.getOrigin().getZ());
	oacg.useCornerCovariance(use_corner_covariance);
	oacg.useCovarianceToFindLinks(use_covariance_to_find_links);
	oacg.useLinksPriorSSRR(use_links_prior_classic_ssrr);
	oacg.useMCLObservationOnPrior(use_mcl_observation_on_prior);
	oacg.useRobotMaps(use_robot_maps);
//	oacg.optimizePrior(optimize_prior);

	oacg.useMCLCovToFindPriorObserved(use_mcl_cov_to_find_prior_observed);

	oacg.setScalingFactorOfGaussians(gaussian_scale);
	oacg.setThrehsoldOfScoreForCreatingLink(threshold_score_link_creation);

	oacg.usePrior(use_prior);
	//ATTENTION
	if(use_prior) {
		oacg.addPriorGraph(graph_prior);
		oacg.setPriorReference();
	}

//	oacg.linkToPrior(link_to_prior);


	std::cout << "*************** DESCRIPTION INIT **************" << std::endl;
	oacg.print();
	std::cout << "*************** DESCRIPTION INIT **************" << std::endl;
		
	//Create initialiser
	AASS::acg::RvizPointsLocalization initialiser(nh, &oacg);
	
	
	AASS::acg::VisuAutoCompleteGraphLocalization visu(nh);
	std::string path_images = path_to_acg + "/ACG_folder/Images";
	visu.setImageFileNameOut(path_images);
	
	ndt_graph_sub = nh.subscribe<auto_complete_graph::GraphMapLocalizationMsg>("/graph_node/graph_map_localization", 1000, boost::bind(&gotGraphandOptimize, _1, &oacg, visu));
	call_for_publish_occ = nh.subscribe<std_msgs::Bool>("/publish_occ_acg", 1, boost::bind(&latchOccGrid, _1, &oacg));
	publish_prior_ndt = nh.subscribe<std_msgs::Bool>("/publish_prior_ndt", 1, boost::bind(&publishPriorNDT, _1, boost::ref(oacg) ));
	publish_acg_om_maps = nh.subscribe<std_msgs::Bool>("/publish_acg_om_maps", 1, boost::bind(&publishACGOM, _1, boost::ref(oacg) ));
	// 	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("/ndt_graph", 1000, boost::bind(&testMsg, _1));
// 	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("/ndt_graph", 1000, boost::bind(&gotGraphandOptimize, _1, &oacg));
	map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_grid", 1000);
	occ_send = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_map_asif", 10, true);


	acg_gdim = nh.advertise<auto_complete_graph::ACGMaps>("acg_maps", 10);
	acg_gdim_om = nh.advertise<auto_complete_graph::ACGMapsOM>("acg_maps_om", 10);
	last_ndtmap = nh.advertise<ndt_map::NDTMapMsg>("lastgraphmap_acg", 10);

	last_grid_map = nh.advertise<grid_map_msgs::GridMap>("last_grid_map", 10);

	last_ndtmap_full = nh.advertise<nav_msgs::OccupancyGrid>("occ_grid_ndt", 10);

	prior_cloud = nh.advertise<sensor_msgs::PointCloud2>("prior_point_cloud", 10);
	prior_ndt = nh.advertise<ndt_map::NDTMapMsg>("prior_ndt", 10);
	
// 	visu.updateRvizNoNDT();
	
	timef = ros::Time::now();
	ros::Time time_begin = ros::Time::now();

	std::cout << "READY :D" << std::endl;
	
	bool flag = true;
	while(ros::ok() && flag == true ){
// 		std::cout <<"SPIN auto_complete" << std::endl;
		ros::spinOnce();
		
		if(abort_f == true){
			return EXIT_SUCCESS;
		}
		
		if(initialiser.size() >= 2){
			was_init = true;
			initAll(oacg, initialiser, priormap);
			visu.updateRvizNoNDT(oacg);
		}
		
// 		visu.updateRvizV2();
// 		std::cout << oacg.getLinkEdges().size()<< std::endl;
		
		ros::Time future = ros::Time::now();
// 		std::cout << "future " << (future - timef).toSec() << " since " << oacg.getRobotNodes().size() << ">" << 5 << " "<< new_node << "==" << true  << " "<< was_init << "==" << true << std::endl;
// 		exit(0);
		
// 		if( (future - timef).toSec() >= 10 && oacg.getRobotNodes().size() > 5 && new_node == true && was_init == true){
// // 			std::cout << "Out " << (future - timef).toSec() << " "<< (timef).toSec() << " " <<(future).toSec() <<std::endl;
// // 			exit(0);
// // 			visu.updateRvizNoNDT();	
// 		// 	oacg->initializeOptimization();
// 		// 	oacg->initialGuess();
// 			//Prepare the graph : marginalize + initializeOpti
// // 			oacg.getGraph().setFirst();
// 			oacg.prepare();
// 			oacg.optimize();
// 			count++;
// // 			printImages(&oacg);
// 			
// 			std::cout << "********************************************************" << std::endl << std::endl;
// 			std::cout << "Final number of nodes : " << oacg.getGraph().vertices().size() << " time : " << (future - time_begin).toSec() << std::endl;
// 			std::cout << "Mean time for processing a node " << node_process_time/cycles <<std::endl;
// 			std::cout << "********************************************************" << std::endl << std::endl;
// // 			flag = false;
			
// 			visu.toOcc();
						
// 			new_node = false;
			
// 		}	
		
		visu.updateRviz(oacg);



		
		
	}
	
	exportResultsGnuplot("result.txt");

    return 0;
}
