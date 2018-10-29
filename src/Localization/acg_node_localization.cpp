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

#include <nav_msgs/Odometry.h>

#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>

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
ros::Publisher last_occ_map;
ros::Publisher last_occ_map2;

ros::Time timef;

int count = 0;

bool new_node = false;
bool was_init = false;
bool euclidean_dist_occ_map = false;
bool export_iteration_count = false;
bool use_gt_for_odom = false;

std::vector<double> time_extract_corner_ndt;
std::vector<double> time_opti;
std::vector<double> all_node_times;

nav_msgs::OccupancyGrid::Ptr occ_map_global;

std::string world_frame;
std::string map_frame;
std::string sensor_frame;

bool updated = true;

double scaling_gaussian_occ_map = 1/2;
double occupancy_grid_resolution = 0.05;
bool add_noise_odometry = false;

double max_iteration = 100;


std::vector<std::pair <uint32_t, uint32_t> > times_of_nodes;

std::vector<nav_msgs::Odometry> gt_odometry;


std::tuple<double, double> error_kitti_benchmark_single(const Eigen::Vector3d& pose, const Eigen::Vector3d& pose_2, const Eigen::Vector3d& gt, const Eigen::Vector3d& gt_2){
	Eigen::Vector2d motion = pose.head(2) - pose_2.head(2);
	Eigen::Vector2d motion_gt = gt.head(2) - gt_2.head(2);
	double error = (motion - motion_gt).norm();

	double angle = pose(2) - pose_2(2);
	double angle_gt = gt(2) - gt_2(2);
	double error_rot = std::abs(angle - angle_gt);
	if(error_rot >=M_PI){
		error_rot =(2 * M_PI) - error_rot;
	}

	return std::make_tuple(error, error_rot);
}

std::tuple<double, double> error_kitti_benchmark_translation(const std::vector<Eigen::Vector3d>& pose, const std::vector<Eigen::Vector3d> gt){
	assert(pose.size() == gt.size() );
	double error_t = 0;
	double error_rot = 0;
	for(int i = 1 ; i < pose.size() ; ++i){

		auto errors = error_kitti_benchmark_single(pose[i-1], pose[i], gt[i-1], gt[i]);

		error_t = error_t + std::get<0>(errors);
		error_rot = error_rot + std::get<1>(errors);

//		Eigen::Vector2d motion = pose[i-1].head(2) - pose[i].head(2);
//		Eigen::Vector2d motion_gt = gt[i-1].head(2) - gt[i].head(2);
//		error = error + (motion - motion_gt).norm();
//
//		double angle = pose[i-1](2) - pose[i](2);
//		double angle_gt = gt[i-1](2) - gt[i](2);
//		error_rot = error_rot + std::abs(angle - angle_gt);

	}
	error_t = error_t / pose.size();
	error_rot = error_rot / pose.size();

	return std::make_tuple(error_t, error_rot);

}


inline bool exists_test3 (const std::string& name) {
	struct stat buffer;   
	return (stat (name.c_str(), &buffer) == 0); 
}


void getGroundTruth(const nav_msgs::OdometryConstPtr& odom){
//	std::cout << "Savong odom " << odom->header.stamp.sec << " " << odom->header.stamp.nsec << std::endl;
	gt_odometry.push_back(*odom);
}


tf::StampedTransform getPoseTFTransform(const std::string& base_frame, const std::string& to_frame, ros::Time time=ros::Time(0)){
	std::cout << "GEt pose " << std::endl;
	tf::TransformListener listener;
	tf::StampedTransform transform;
// 	int i = 0;
// 	while(i < 10){
// 	++i;
//	try {
//		time=ros::Time(0);
//		bool hunm = listener.waitForTransform(base_frame, to_frame, time, ros::Duration(1.0));
//		listener.lookupTransform(base_frame, to_frame, time, transform);
//	} catch (tf::TransformException ex) {
//		ROS_ERROR("%s",ex.what());
//		exit(0);
//	}
	bool good_transformation = true;
	do {
		try {
			time = ros::Time(0);
			bool hunm = listener.waitForTransform(base_frame, to_frame, time, ros::Duration(1.0));
			listener.lookupTransform(base_frame, to_frame, time, transform);
			good_transformation = true;
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			good_transformation = false;
		}
	} while(good_transformation == false);
	return transform;
}

int getClosestTime(g2o::VertexSE2RobotLocalization* robot_vertex){

	uint32_t sec = robot_vertex->time_sec;
	uint32_t nsec = robot_vertex->time_nsec;

	ros::Time node_time; node_time.sec = sec; node_time.nsec = nsec;

//	std::cout << "sec: " << sec << " size " << gt_odometry.size() << std::endl;
	assert(sec != -1);
	assert(gt_odometry.size() > 0);

	int place_in_vec = 0;
	int iteration_tmp = 0;
	ros::Duration diff_duration = node_time - gt_odometry[0].header.stamp;
	double diff = std::abs(diff_duration.toSec());


//	std::cout << "close ? " << gt_odometry[0].header.stamp.sec << " robot " << sec << " nsec " << gt_odometry[0].header.stamp.nsec << " robot " << nsec << std::endl;

//			std::cout << "Sec Diff " << diff_sec << std::endl;

	for(int i = 0; i < gt_odometry.size() ; ++i){
		ros::Duration diff_duration_tmp = node_time - gt_odometry[i].header.stamp;
		double diff_tmp = std::abs(diff_duration_tmp.toSec());

//				if(count > 0) {
//					std::cout << "Sec Diff_tmo " << diff_sec_tmp << std::endl;
//					std::cout << "Sec Diff " << diff_sec << std::endl;
//					std::cout << "Sec Diffn_tmo " << diff_nsec_tmp << std::endl;
//					std::cout << "Sec Diffn " << diff_nsec << std::endl;
//				}

		assert(diff >= 0);
		assert(diff_tmp >= 0);

		if(diff_tmp <= diff){

//					std::cout << "Sec Diff " << diff << std::endl;
//					std::cout << "Sec Diff_tmp " << diff_tmp << std::endl;
//					std::cout << "close ? " << gt_odometry[i].header.stamp.sec << " robot " << sec << " nsec " << gt_odometry[i].header.stamp.nsec << " robot " << nsec << std::endl;

			place_in_vec = i;
			diff = diff_tmp;

		}
	}

	std::cout << "CLosest " << place_in_vec << std::endl;
	return place_in_vec;
}


bool exportErrorNoiseOdometry(const AASS::acg::AutoCompleteGraphLocalization& oacg){

	std::string file_out = "/home/malcolm/ros_catkin_ws/lunar_ws/acg_error_odometry.dat";
//	std::ostringstream convert;   // stream used for the conversion
	std::ofstream infile(file_out, std::ofstream::app);

	infile << std::endl << std::endl;

	double error_t_mean = 0;
	double error_a_mean = 0;
	double error_tn_mean = 0;
	double error_an_mean = 0;
	double error_t_mean_gt = 0;
	double error_a_mean_gt = 0;

	int count = 0;
	infile << "# node_number error_translation error_translation_noisy error_translation_gt error_angle error_angle_noisy error_angle_gt time_sec time_nsec" << std::endl;

	std::vector<Eigen::Vector3d> poses_robot;
	std::vector<Eigen::Vector3d> poses_gt;

	for(auto robot_vertex : oacg.getRobotPoseLocalization()){

		Eigen::Affine3d pose_original = robot_vertex->getPose();
		Eigen::Isometry2d pose_original_iso = AASS::acg::Affine3d2Isometry2d(pose_original);
		g2o::SE2 pose_original_se2(pose_original_iso);
		g2o::SE2 robot_pose = robot_vertex->estimate();
		g2o::SE2 robot_pose_noisy = robot_vertex->initial_noisy_estimate;

		poses_robot.push_back(robot_pose.toVector());


		double error_t = (robot_pose.toVector().head(2) - pose_original_se2.toVector().head(2)).norm();
		double error_a = std::abs(robot_pose.toVector()(2) - pose_original_se2.toVector()(2));
		double error_t_n = (robot_pose_noisy.toVector().head(2) - pose_original_se2.toVector().head(2)).norm();
		double error_a_n = std::abs(robot_pose_noisy.toVector()(2) - pose_original_se2.toVector()(2));
		if(error_a_n >=M_PI){
			error_a_n = (2 * M_PI) - error_a_n;
		}
		if(error_a >=M_PI){
			error_a = (2 * M_PI) - error_a;
		}

		error_t_mean += error_t;
		error_tn_mean += error_t_n;
		error_a_mean += error_a;
		error_an_mean += error_a_n;

		if(use_gt_for_odom == false) {
			infile << count << " " << error_t << " " << error_t_n << " " << -1 << " " << error_a << " " << error_a_n << " " << -1 << " "
			       << std::fixed << std::setprecision(8) << times_of_nodes[count].first << " "
			       << times_of_nodes[count].second << std::endl;
		}
		else{

			int place_in_vec = getClosestTime(robot_vertex);

			nav_msgs::Odometry gt_pose = gt_odometry[place_in_vec];

			double x = gt_pose.pose.pose.position.x;
			double y = gt_pose.pose.pose.position.y;
			tf::Quaternion quat;
			tf::quaternionMsgToTF(gt_pose.pose.pose.orientation, quat);
			// the tf::Quaternion has a method to acess roll pitch and yaw
			double roll, pitch, yaw;
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			Eigen::Vector3d gt_pose_save; gt_pose_save << x, y, yaw;
			poses_gt.push_back(gt_pose_save);

			std::cout << "RPY " << roll << " " << pitch << " " << yaw << std::endl;
			std::cout << "xy" << x << " " << y << std::endl;
			std::cout << "Robot pose " << robot_pose.toVector().head(2) << std::endl;

//			Eigen::Vector2d position_gt; position_gt << x, y;
//			double angle_gt; angle_gt << roll;

			std::tuple<double, double> errors_gt = std::make_tuple(0, 0);
			if(poses_robot.size() > 1) {
				errors_gt = error_kitti_benchmark_single(poses_robot.back(), poses_robot.rbegin()[1],
				                                              poses_gt.back(), poses_gt.rbegin()[1]);
			}
			error_t_mean_gt += std::get<0>(errors_gt);
			error_a_mean_gt += std::get<1>(errors_gt);

			infile << count << " " << error_t << " " << error_t_n << " " << std::get<0>(errors_gt) << " " << error_a << " " << error_a_n << " " << std::get<1>(errors_gt) << " "
			       << std::fixed << std::setprecision(8) << times_of_nodes[count].first << " "
			       << times_of_nodes[count].second << std::endl;
		}
		++count;

	}


	count = 0;
	poses_robot.clear();
	poses_gt.clear();

	error_t_mean = error_t_mean / oacg.getRobotPoseLocalization().size();
	error_tn_mean = error_tn_mean / oacg.getRobotPoseLocalization().size();
	error_t_mean_gt = error_t_mean_gt / oacg.getRobotPoseLocalization().size();
	error_a_mean = error_a_mean / oacg.getRobotPoseLocalization().size();
	error_an_mean = error_an_mean / oacg.getRobotPoseLocalization().size();
	error_a_mean_gt = error_a_mean_gt / oacg.getRobotPoseLocalization().size();

	double sum_sqd_t = 0;
	double sum_sqd_tn = 0;
	double sum_sqd_a = 0;
	double sum_sqd_an = 0;
	double sum_sqd_t_gt = 0;
	double sum_sqd_a_gt = 0;

	for(auto robot_vertex : oacg.getRobotPoseLocalization()){

		Eigen::Affine3d pose_original = robot_vertex->getPose();
		Eigen::Isometry2d pose_original_iso = AASS::acg::Affine3d2Isometry2d(pose_original);
		g2o::SE2 pose_original_se2(pose_original_iso);
		g2o::SE2 robot_pose = robot_vertex->estimate();
		g2o::SE2 robot_pose_noisy = robot_vertex->initial_noisy_estimate;

		poses_robot.push_back(robot_pose.toVector());

		double error_t = (robot_pose.toVector().head(2) - pose_original_se2.toVector().head(2)).norm();
		double error_a = std::abs(robot_pose.toVector()(2) - pose_original_se2.toVector()(2));
		double error_t_n = (robot_pose_noisy.toVector().head(2) - pose_original_se2.toVector().head(2)).norm();
		double error_a_n = std::abs(robot_pose_noisy.toVector()(2) - pose_original_se2.toVector()(2));
		if(error_a_n >=M_PI){
			error_a_n = (2 * M_PI) - error_a_n;
		}
		if(error_a >=M_PI){
			error_a = (2 * M_PI) - error_a;
		}

		sum_sqd_t = sum_sqd_t + ( (error_t - error_t_mean) * (error_t - error_t_mean) );
		sum_sqd_tn = sum_sqd_tn + ( (error_t_n - error_tn_mean) * (error_t_n - error_tn_mean) );
		sum_sqd_a = sum_sqd_a + ( (error_a - error_a_mean) * (error_a - error_a_mean) );
		sum_sqd_an = sum_sqd_an + ( (error_a_n - error_an_mean) * (error_a_n - error_an_mean) );

		if(use_gt_for_odom == true){

			int place_in_vec = getClosestTime(robot_vertex);

			nav_msgs::Odometry gt_pose = gt_odometry[place_in_vec];

			double x = gt_pose.pose.pose.position.x;
			double y = gt_pose.pose.pose.position.y;
			tf::Quaternion quat;
			tf::quaternionMsgToTF(gt_pose.pose.pose.orientation, quat);
			// the tf::Quaternion has a method to acess roll pitch and yaw
			double roll, pitch, yaw;
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			Eigen::Vector3d gt_pose_save; gt_pose_save << x, y, yaw;

			poses_gt.push_back(gt_pose_save);

			std::tuple<double, double> errors_gt = std::make_tuple(0, 0);
			if(poses_robot.size() > 1) {
				errors_gt = error_kitti_benchmark_single(poses_robot.back(), poses_robot.rbegin()[1],
				                                         poses_gt.back(), poses_gt.rbegin()[1]);
			}
//			error_t_mean_gt += std::get<0>(errors_gt);
//			error_a_mean_gt += std::get<1>(errors_gt);

			sum_sqd_t_gt = sum_sqd_t_gt + ( (std::get<0>(errors_gt) - error_t_mean_gt) * (std::get<0>(errors_gt) - error_t_mean_gt) );
			sum_sqd_a_gt = sum_sqd_a_gt + ( (std::get<1>(errors_gt) - error_a_mean_gt) * (std::get<1>(errors_gt) - error_a_mean_gt) );
		}

		++count;

	}

	sum_sqd_t = std::sqrt(sum_sqd_t / (count + 1));
	sum_sqd_a = std::sqrt(sum_sqd_a / (count + 1));
	sum_sqd_tn = std::sqrt(sum_sqd_tn / (count + 1));
	sum_sqd_an = std::sqrt(sum_sqd_an / (count + 1));

	infile << std::endl << "# number_of_nodes mean_error_translation std mean_error_translation_noisy std mean_error_gt std mean_error_angle std mean_error_angle_noisy std mean_error_angle_gt std" << std::endl;
	infile << oacg.getRobotPoseLocalization().size() << " " << error_t_mean << " " << sum_sqd_t << " " << error_tn_mean << " " << sum_sqd_tn << " " << error_t_mean_gt << " " << sum_sqd_t_gt << " " << error_a_mean << " " << sum_sqd_a << " " << error_an_mean << " " << sum_sqd_an << " " << error_a_mean_gt << " " << sum_sqd_a_gt << std::endl;

	if(use_gt_for_odom == true) {
		auto error_translation_kitti = error_kitti_benchmark_translation(poses_robot, poses_gt);
		infile << std::endl << "# kitti_translation_error kitti_rotation_error" << std::endl;
		infile << std::get<0>(error_translation_kitti) << " " << std::get<1>(error_translation_kitti) << std::endl;
	}

	infile.close();
}




bool exportIterationCOunt(const AASS::acg::AutoCompleteGraphLocalization& oacg, const std::pair<int, int>& iterations, double time_corner, double time_opti){

	std::cout << "Exporting number of iterations and time" << std::endl;

	std::string file_out = "/home/malcolm/ros_catkin_ws/lunar_ws/iterations.dat";
	std::ostringstream convert;   // stream used for the conversion
	int node_number = oacg.getRobotPoseLocalization().size();
	node_number += oacg.getLandmarkNodes().size();
	node_number += oacg.getPrior()->getNodes().size();
	node_number += oacg.getNDTCells().size();

	int edge_number = oacg.getOdometryEdges().size();
	edge_number += oacg.getPrior()->getEdges().size();
	edge_number += oacg.getPriorObservations().size();
	edge_number += oacg.getLandmarkEdges().size();
	edge_number += oacg.getNDTCellAssociations().size();
	edge_number += oacg.getNDTCellObservations().size();

	convert << oacg.getRobotNodes().size();
//	file_out = file_out + convert.str();
//	file_out = file_out + ".txt";
	std::ofstream infile(file_out, std::ofstream::app);
// 			int co = 0;
	std::cout << node_number << " " << edge_number << " " << iterations.first << " " << iterations.second << " " << time_corner << " " << time_opti << std::endl;
	infile << node_number << " " << edge_number << " " << iterations.first << " " << iterations.second << " " << time_corner << " " << time_opti << std::endl;
	infile.close();

	std::cout << "Done" << std::endl;
	return true;
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
	int size_rl = oacg.getRobotPoseLocalization().size();
	if(size_rl > 0){
		perception_oru::toOccupancyGrid(oacg.getRobotPoseLocalization()[size_rl - 1]->getMap().get(), *occ_out, 0.1, "/world");
	}
	std::cout << "Occupancy grid sent ! " << std::endl;
	return occ_out;

}

void sendMapAsOcc(const AASS::acg::AutoCompleteGraphLocalization& oacg){
	
	if(updated == true){
		occ_map_global = createOccupancyMap(oacg);
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

//void publishPriorNDT(const AASS::acg::AutoCompleteGraphLocalization& oacg){
//
//	std::cout << "Publishing the prior" << std::endl;
////	std::cout << "PUB1" << std::endl;
//	//		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_prior = AASS::acg::ACGPriortoPointCloud<AASS::acg::AutoCompleteGraphPriorSE2>(*oacg.getPrior(), 0.1, 0.1/4);
////		sensor_msgs::PointCloud2 pcl_prior_msg;
////		pcl::toROSMsg<pcl::PointXYZ>(*pcl_prior, pcl_prior_msg);
////		pcl_prior_msg.header.frame_id = "world";
////		pcl_prior_msg.header.stamp = ros::Time::now();
////		prior_cloud.publish(pcl_prior_msg);
//
//	perception_oru::NDTMap* ndt_prior = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
//	AASS::acg::ACGPriorToNDTMap<AASS::acg::AutoCompleteGraphPriorXY>(*oacg.getPrior(), *ndt_prior, oacg.getZElevation(), 0.1);
//
////		auto allcells = ndt_prior->getAllCellsShared();
////		assert(allcells.size() > 0);
//
//	ndt_map::NDTMapMsg priormapmsg;
//	perception_oru::toMessage(ndt_prior, priormapmsg, map_frame);
//	prior_ndt.publish(priormapmsg);
//
//	delete ndt_prior;
//}



//void publishPriorNDT(const std_msgs::Bool::ConstPtr msg, AASS::acg::AutoCompleteGraphLocalization& oacg) {
//	publishPriorNDT(oacg);
//	if(optimize_prior == false){
//		oacg.clearPrior();
//	}
//}

void publishACGOM(const AASS::acg::AutoCompleteGraphLocalization& oacg){

//	std::cout << "PUB2" << std::endl;
	//Puclish message for GDIM
	auto_complete_graph::ACGMaps mapmsg;
	ROS_INFO("PUSH acg maps message");
	AASS::acg::ACGToACGMapsMsg(oacg, mapmsg, map_frame);
	acg_gdim.publish(mapmsg);


//	auto_complete_graph::ACGMapsOM mapmsg_om;
//	ROS_INFO("PUSH acg maps OM message");
//	AASS::acg::ACGToACGMapsOMMsg(oacg, mapmsg_om, map_frame, occupancy_grid_resolution, scaling_gaussian_occ_map, euclidean_dist_occ_map);
//	acg_gdim_om.publish(mapmsg_om);

	//Publish the last grid map as a message to make sure that they look like something
	int size_g = mapmsg.ndt_maps.maps.size();
	if(size_g > 0) {
//		ROS_DEBUG("Last grid map");
//		last_grid_map.publish(mapmsg.ndt_maps.maps[size_g - 1]);

		//Publish last occ grid to make sure that they look like something
		int size_o = mapmsg.ndt_maps.maps.size();
		ROS_DEBUG("Last ndtmap");
		last_ndtmap.publish(mapmsg.ndt_maps.maps[size_o - 1]);

        nav_msgs::OccupancyGrid omap;
//		= new nav_msgs::OccupancyGrid();
		perception_oru::NDTMap* last_map = oacg.getRobotNodes()[oacg.getRobotNodes().size() - 1 ]->getMap().get();

//		std::cout << "SCALIN GAUSSIAN OCC " << scaling_gaussian_occ_map << std::endl;
		perception_oru::toOccupancyGrid(last_map, omap, occupancy_grid_resolution, map_frame, scaling_gaussian_occ_map, euclidean_dist_occ_map);
		last_occ_map.publish(omap);
//		delete omap;


//		nav_msgs::OccupancyGrid omap2;
//		grid_map::GridMap mapNDT;
//		grid_map::GridMapRosConverter converter;
//		converter.fromMessage(mapmsg.ndt_maps.maps[mapmsg.ndt_maps.maps.size() - 1], mapNDT);
		//THis ruin prior because they are of different sizes ! Need my custom fuse function :)
//		grid_map::GridMapRosConverter::toOccupancyGrid(mapNDT, "ndt", 0, 255, omap2);
//		last_occ_map2.publish(omap2);


//        initOccupancyGrid(*omap, 250, 250, 0.4, "/world");
//        perception_oru::toOccupancyGrid(&mapmsg.ndt_maps.maps[size_o -1], *omap, 0.1, "/world");


	}
	ROS_INFO("Done");
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
//	std::cout << "WELLL HERE IT IS : " << occ_outt_partial->info.origin.position << " ori " << occ_outt_partial->info.origin.orientation << std::endl << std::endl;
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
	ROS_DEBUG("Got a new graph ");
	
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
	bool good = perception_oru::toMessage(graph.getNode(0).map->map, msgio, map_frame);
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

//	std::cout << "GOT GRAPH" << std::endl;

		times_of_nodes.clear();

		for(auto node : msg->graph_map.nodes){
			std::cout << "TIME : " << std::fixed << std::setprecision(8) << (double)node.time_sec.data << " " << (double)node.time_nsec.data << std::endl;
			times_of_nodes.push_back(std::pair <uint32_t, uint32_t>(node.time_sec.data, node.time_nsec.data) );
		}


		updated = true;
		new_node = true;
	// 	abort_f = true;
		ROS_INFO("Got a new graph ");
		
		ros::Time start = ros::Time::now();
		
// 		ndt_feature::NDTFeatureGraph graph;
		
		std::string frame;
		//ATTENTION: THE BAD GUY !
// 		ndt_feature::msgToNDTGraph(*msg, graph, frame);	
		
		
		ros::Time start_corner = ros::Time::now();
		oacg->updateNDTGraph(*msg);
		ros::Time end_corner = ros::Time::now();

		ROS_DEBUG("MAP UPDATED in main");
		
		double corner_extract_tt = (end_corner - start_corner).toSec();
		time_extract_corner_ndt.push_back(corner_extract_tt);
		
//		if(oacg->getRobotNodes().size() > 0){

		if(testing_pause) {
			ROS_DEBUG("RVIZ ");
			visu.updateRviz(*oacg);
			ROS_DEBUG("RVIZ DONE");
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
				std::pair<int, int> iterations;
				ros::Time start_opti = ros::Time::now();
				bool optiquest = true;
				if(testing_pause) {
					std::cout << "Optimize ?" << std::endl;
					std::cin >> optiquest;
				}
				if( /*oacg->checkAbleToOptimize() &&*/  optiquest) {
					oacg->setFirst();
					oacg->prepare();
					std::cout << "ITERATIO COUNT " << max_iteration << std::endl;
					iterations = oacg->optimize(max_iteration);

				}
				else{
					oacg->testInfoNonNul("Just making sure");
				}
				ros::Time end_opti = ros::Time::now();
	std::cout << "Start opti time " << start_opti.toSec() << " " << end_opti.toSec() << std::endl;
	std::cout << "Start opti time " << start_opti.toNSec() << " " << end_opti.toNSec() << std::endl;
				double opti = (end_opti - start_opti).toSec();
				time_opti.push_back(opti);
				
		// 		if(was_init == true){
					
		// 		}

				if(optimize_prior == true) {
					ROS_DEBUG("Publishing the new prior ndt map");
					publishACGOM(*oacg);
				}

//				std::cout << "Publish an occupancy grid for you Asif :3 ! " << std::endl;
//				sendMapAsOcc(*oacg);


	ROS_DEBUG("RVIZ " );
				visu.updateRviz(*oacg);
				publishACGOM(*oacg);
	ROS_DEBUG("RVIZ DONE");

				if(testing_pause) {
					std::cout << "Result of optimization. Enter a number to continue" << std::endl;
					int aaa;
					std::cin >> aaa;
				}




				if(export_iteration_count == true && optiquest){
					bool done = exportIterationCOunt(*oacg, iterations, corner_extract_tt, opti);
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

//			std::cout << "*************** DESCRIPTION **************" << std::endl;
//			oacg->print();
//			std::cout << "*************** DESCRIPTION **************" << std::endl;

				
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

	if(optiquest && add_noise_odometry){
		exportErrorNoiseOdometry(*oacg);
	}
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

	ROS_DEBUG("clear prior");
	oacg.clearPrior();
	ROS_DEBUG("add prior");
	oacg.addPriorGraph(graph_prior);
	
	initialiser.clear();

//	publishPriorNDT(oacg);
	publishACGOM(oacg);

	if(optimize_prior == false){
		oacg.clearPrior();
	}
	
}


//
//void initManual() {
//
//	std::cout << "INIT ALL" << std::endl;
//	std::vector<cv::Point2f> slam_pt;
//	std::vector<cv::Point2f> prior_pt;
//
//	priorloader.initialize(slam_pt, prior_pt);
//
//	//We use the already registered points
//// 	priorloader.extractCornerPrior();
//	priorloader.transformOntoSLAM();
//	auto graph_prior = priorloader.getGraph();
//
//	ROS_DEBUG("clear prior");
//	oacg.clearPrior();
//	ROS_DEBUG("add prior");
//	oacg.addPriorGraph(graph_prior);
//
//	initialiser.clear();
//
//	publishPriorNDT(oacg);
//	publishACGOM(oacg);
//
//}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_complete_graph_rviz_small_optimi");
	ros::Subscriber ndt_graph_sub;
	ros::Subscriber call_for_publish_occ, publish_prior_ndt, publish_acg_om_maps, gt_odom;
    ros::NodeHandle nh("~");

	/**************PARAMETERS**************/
	nh.param("pause_for_testing",testing_pause,false);
	bool use_prior = true;
	nh.param("export_iteration_count",export_iteration_count,false);

	nh.param("use_gt_for_odom", use_gt_for_odom, false);
	nh.param("add_noise_odometry",add_noise_odometry,false);
	bool not_incremental_optimization = false;
	nh.param("not_incremental_optimization",not_incremental_optimization,false);
	bool export_iteration_count = false;
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
	nh.param<std::string>("world_frame",world_frame,"/world");
	nh.param<std::string>("map_frame", map_frame,"/map");
	nh.param<std::string>("sensor_frame",sensor_frame,"/velodyne");
	double deviation = 0;
	nh.param<double>("deviation", deviation, 0);
	double angle = 0;
	nh.param<double>("angle", angle, 0);
	double scale = 1;
	nh.param<double>("scale", scale, 1.0);
	cv::Point2f center(0, 0);
	bool match_ndt_maps = false;
	nh.param("match_ndt_maps",match_ndt_maps,false);
	double min_val_cov = 0.1;
	nh.param<double>("min_val_cov_ndt_cell", min_val_cov, 0.1);
	double min_corner_ndt_cell_distance = 1;
	nh.param<double>("min_corner_ndt_cell_distance", min_corner_ndt_cell_distance, 1);
	double max_distance_ndt_cell_from_robot_pose = -1;
	nh.param<double>("max_distance_ndt_cell_from_robot_pose", max_distance_ndt_cell_from_robot_pose, -1);
	bool use_huber_kernel;
	nh.param("use_huber_kernel",use_huber_kernel,true);
	bool use_dcs_kernel;
	nh.param("use_dcs_kernel",use_dcs_kernel,true);
	bool use_robust_kernel;
	nh.param("use_robust_kernel",use_robust_kernel,true);
	double main_axis_prior_noise = 50;
	nh.param<double>("main_axis_prior_noise", main_axis_prior_noise, 50);
	double other_axis_prior_noise = 0.005;
	nh.param<double>("other_axis_prior_noise", other_axis_prior_noise , 0.005);
	bool use_user_cov_odometry;
	nh.param("use_user_cov_odometry",use_user_cov_odometry,false);
	double error_under_witch_stop_optimization = 1;
	nh.param<double>("error_under_witch_stop_optimization", error_under_witch_stop_optimization, 1);
	double landmark_noise_x = 0.05;
	nh.param<double>("landmark_noise_x", landmark_noise_x, 0.05);
	double landmark_noise_y = 0.05;
	nh.param<double>("landmark_noise_y", landmark_noise_y, 0.05);

//	double max_iteration = 100;
	nh.param<double>("max_iteration_acg", max_iteration, 100);
	std::cout << "INIT MAX ITE " << max_iteration;

	double noise_odom_perc = 0.1;
	nh.param<double>("noise_odom_percentage", noise_odom_perc, 0.1);


	double max_deviation_corner_in_prior = 1;
	nh.param<double>("max_deviation_corner_in_prior", max_deviation_corner_in_prior, 45 * 3.14159 / 180);


	std::string prior_file = "";
	nh.param<std::string>("prior_file",prior_file,"/home/malcolm/ros_catkin_ws/lunar_ws/src/auto_complete_graph/tests/emergbasement_flipped_nodoor.png");


	nh.param<double>("gaussian_scaling_occ", scaling_gaussian_occ_map, 0.5);
	nh.param<double>("occupancy_grid_resolution", occupancy_grid_resolution, 0.05);
	nh.param("euclidean_dist_occ_map",euclidean_dist_occ_map,false);

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
//	std::string param = path_to_acg + "/ACG_folder/param.txt";

	Eigen::Vector2d ln; ln << landmark_noise_x, landmark_noise_y;
	Eigen::Vector2d pn; pn << main_axis_prior_noise, other_axis_prior_noise;

	AASS::acg::AutoCompleteGraphLocalization oacg(g2o::SE2(0.2, 0.1, -0.1), ln, pn);
	
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
	oacg.matchRobotMaps(match_ndt_maps);
	oacg.addNoiseToOdometryMeasurements(add_noise_odometry);
	oacg.addIncrementalOptimization(not_incremental_optimization);
	oacg.minValueCovNDTCell(min_val_cov);
	oacg.minDistanceToCornerNDTCell(min_corner_ndt_cell_distance);
	oacg.maxDistanceOfNDTCellToRobotPose(max_distance_ndt_cell_from_robot_pose);
	oacg.useHuberKernel(use_huber_kernel);
	oacg.useDCSKernel(use_dcs_kernel);
	oacg.useRobustKernel(use_robust_kernel);
	oacg.getPrior()->setPriorNoise(main_axis_prior_noise, other_axis_prior_noise );
	oacg.useUserCovForRobotPose(use_user_cov_odometry);
	oacg.setLandmarkNoise(landmark_noise_x, landmark_noise_y);
	oacg.setPercentageNoiseOdometry(noise_odom_perc);
	oacg.errorUnderWhichWeStopTheOptimization(error_under_witch_stop_optimization);

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


	//Init GDIM and MAPPING
//	publishPriorNDT(oacg);
//	publishACGOM(oacg);

//	oacg.linkToPrior(link_to_prior);


	ROS_DEBUG("*************** DESCRIPTION INIT **************");
	oacg.print();
	ROS_DEBUG("*************** DESCRIPTION INIT **************");
		
	//Create initialiser
	AASS::acg::RvizPointsLocalization initialiser(nh, &oacg);
	

	AASS::acg::VisuAutoCompleteGraphLocalization visu(nh, map_frame);
	std::string path_images = path_to_acg + "/ACG_folder/Images";
	visu.setImageFileNameOut(path_images);
	
	ndt_graph_sub = nh.subscribe<auto_complete_graph::GraphMapLocalizationMsg>("/graph_node/graph_map_localization", 1000, boost::bind(&gotGraphandOptimize, _1, &oacg, visu));
	call_for_publish_occ = nh.subscribe<std_msgs::Bool>("/publish_occ_acg", 1, boost::bind(&latchOccGrid, _1, &oacg));
	publish_prior_ndt = nh.subscribe<std_msgs::Bool>("/publish_acg_maps", 1, boost::bind(&publishACGOM, _1, boost::ref(oacg) ));
	publish_acg_om_maps = nh.subscribe<std_msgs::Bool>("/publish_acg_om_maps", 1, boost::bind(&publishACGOM, _1, boost::ref(oacg) ));

	if(use_gt_for_odom){
		gt_odom = nh.subscribe<nav_msgs::Odometry>("/vmc_navserver/state", 100, getGroundTruth);
	}

	// 	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("/ndt_graph", 1000, boost::bind(&testMsg, _1));
// 	ndt_graph_sub = nh.subscribe<ndt_feature::NDTGraphMsg>("/ndt_graph", 1000, boost::bind(&gotGraphandOptimize, _1, &oacg));
	map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map_grid", 1000);
	occ_send = nh.advertise<nav_msgs::OccupancyGrid>("/occupancy_map_asif", 10, true);


	acg_gdim = nh.advertise<auto_complete_graph::ACGMaps>("acg_maps", 10);
	acg_gdim_om = nh.advertise<auto_complete_graph::ACGMapsOM>("acg_maps_om", 10);
	last_ndtmap = nh.advertise<ndt_map::NDTMapMsg>("lastgraphmap_acg", 10);
	last_occ_map = nh.advertise<nav_msgs::OccupancyGrid>("occ_lastgraphmap_acg", 10);
	last_occ_map2 = nh.advertise<nav_msgs::OccupancyGrid>("two_occ_lastgraphmap_acg", 10);

	last_grid_map = nh.advertise<grid_map_msgs::GridMap>("last_grid_map", 10);

	last_ndtmap_full = nh.advertise<nav_msgs::OccupancyGrid>("occ_grid_ndt", 10);

	prior_cloud = nh.advertise<sensor_msgs::PointCloud2>("prior_point_cloud", 10);
	prior_ndt = nh.advertise<ndt_map::NDTMapMsg>("prior_ndt", 10);
	
// 	visu.updateRvizNoNDT();
	
	timef = ros::Time::now();
	ros::Time time_begin = ros::Time::now();

	std::cout << "READY :D" << std::endl;



//	ros::Duration(5).sleep();

	//Init GDIM and MAPPING
//	publishPriorNDT(oacg);
	publishACGOM(oacg);


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
