
#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>
//#include <rosbag/bag.h>
//#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
#include "ndt_generic/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>

#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/NDTVectorMapMsg.h>

#include "graph_map/lidarUtils/lidar_utilities.h"

// #include "graph_map/GraphMapMsg.h"
#include "auto_complete_graph/GraphMapLocalizationMsg.h"
#include "ndt_map/NDTVectorMapMsg.h"
#include "graph_map/graph_map_conversion.h"

#include "auto_complete_graph/Localization/AcgMclLocalization.hpp"
#include "auto_complete_graph/conversion.hpp"

#include "ndt_localization/particle_filter.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>
#include <cstdio>
#include "tf_conversions/tf_eigen.h"
#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#define MAX_TRANSLATION_DELTA 2.0
#define MAX_ROTATION_DELTA 0.5
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */

std::vector<std::pair <uint32_t, uint32_t> > times;


void getAngles(const std::string& base_frame, const std::string& to_frame, double& roll, double& pitch, double& yaw){
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		bool hunm = listener.waitForTransform(to_frame, base_frame, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform(to_frame, base_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
			exit(0);
	}
//	double x = transform.getOrigin().x();
//	double y = transform.getOrigin().y();
//	double z = 0;

	transform.getBasis().getRPY(roll, pitch, yaw);
}

tf::StampedTransform getPoseTFTransform(const std::string& base_frame, const std::string& to_frame, ros::Time time=ros::Time(0)){
	ROS_DEBUG("GEt pose ");
	tf::TransformListener listener;
	tf::StampedTransform transform;
// 	int i = 0;
// 	while(i < 10){
// 	++i;

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


Eigen::Affine3d getPose(const std::string& base_frame, const std::string& to_frame, ros::Time time=ros::Time(0)){

	auto transform = getPoseTFTransform(base_frame, to_frame, time);

	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();

	//TEST
// 	x = 16.6;
// 	y = 3.0;
//	std::cout << "Between " << base_frame << " and well " << to_frame << " " << x << " " << y << " " << z << " at " << transform.stamp_ << std::endl;
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);

	Eigen::Affine3d pose_sensor = Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;

//	std::cout << "pose " << pose_sensor.matrix() << std::endl;
// 		exit(0);
// 	}
// 	exit(0);
	return pose_sensor;

}





using namespace perception_oru::graph_map;
using namespace ::graph_map;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsGTOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class GraphMapFuserNode {

protected:


	tf::Transform Transform_map;


//   boost::shared_ptr<perception_oru::particle_filter> ndtmcl_;
//   boost::shared_ptr<NDTMCL> ndtmcl_;


  // Our NodeHandle
  ros::NodeHandle nh_;
  GraphMapFuser *fuser_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
  plotmarker plot_marker;

  message_filters::Subscriber<nav_msgs::Odometry> *gt_fuser_sub_;
  ros::Subscriber gt_sub,points2OdomTfSub, publisher_graph_map;

  ros::Subscriber ndt_mcl_map;
	bool publisher_timer;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_;
  ros::Publisher graphmap_pub_, graphmap_localization_pub;
  Eigen::Affine3d pose_, T, sensorPose_;

	ros::Timer timer_transformation_map;


  unsigned int frame_nr_;
  double varz;
  tf::Transform tf_sensor_pose_;
  std::string map_type_name,reg_type_name;
  std::string map_name="graph_map";
  std::string points_topic, laser_topic, map_dir, odometry_topic,odometry_adjusted_topic, robot_frame;
  std::string file_format_map=".JFF";
  std::string world_link_id, map_link_id, odometry_link_id, fuser_base_link_id,laser_link_id, init_pose_frame, gt_topic, bag_name,state_base_link_id;
  double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
  bool visualize, match2D, matchLaser, beHMT, useOdometry,
  initPoseFromGT, initPoseFromTF, initPoseSet, gt_mapping;
  double zfilter_min_;

  double pose_init_x,pose_init_y,pose_init_z,
  pose_init_r,pose_init_p,pose_init_t;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
  sensor_pose_r,sensor_pose_p,sensor_pose_t;
  double sensor_offset_t_, fraction, cutoff, initVar;

  bool use_graph_map_registration_;

  bool init_fuser_;
  laser_geometry::LaserProjection projector_;
  message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
  message_filters::Synchronizer< LaserPoseSync > *sync_lp_;

  message_filters::Synchronizer< PointsGTOdomSync > *sync_GTodom_;
  message_filters::Synchronizer< PointsOdomSync > *sync_po_;
  message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
  ros::ServiceServer save_map_;
  ros::Time time_now,time_last_itr;
  ros::Publisher map_publisher_,laser_publisher_,point2_publisher_,odom_publisher_,adjusted_odom_publisher_,fuser_odom_publisher_, graph_map_vector_;
  nav_msgs::Odometry fuser_odom,adjusted_odom_msg;
  Eigen::Affine3d last_odom, this_odom,last_gt_pose;

  bool use_tf_listener_, init_pose_tf_;
  Eigen::Affine3d last_tf_frame_;
  perception_oru::MotionModel2d::Params motion_params;
  boost::mutex m;
  bool use_mcl_;

  bool mcl_loaded_ = false;
  ros::Publisher mcl_pub_, laser_point_cloud_pub_, mcl_local_map_, mcl_map_pub_;
  ros::Publisher particlePub_;
  double cov_x_mcl, cov_y_mcl, cov_yaw_mcl, scale_gaussian_mcl;

  boost::shared_ptr<AASS::acg::ACGMCLLocalization> acg_localization;

	double _cell_neighborhood_size_mcl_in_meters;
	bool _use_euclidean_mcl, _use_euclidean_for_long_distances, _use_hybrid_strategy_mcl;
	bool _use_mean_score_mcl;

	int SIRCount = 0;
	double var_part = 0.1;

public:
  // Constructor
  GraphMapFuserNode(ros::NodeHandle param_nh) : frame_nr_(0), mcl_loaded_(false), init_fuser_(false), fuser_(NULL)
  {

	  param_nh.param("SIRCount", SIRCount ,0);
	  param_nh.param("var_new_particle", var_part ,0.1);
//	  param_nh.param("pose_init_t",pose_init_t,0.);

	  assert(fuser_ == NULL);
    ///if we want to build map reading scans directly from bagfile


    ///topic to wait for point clouds, if available
    param_nh.param<std::string>("points_topic",points_topic,"points");
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

    ///only match 2ith 3dof
    param_nh.param("match2D",match2D,true);
    ///enable for LaserScan message input
    param_nh.param("matchLaser",matchLaser,true);


    param_nh.param<std::string>("registration_type",reg_type_name,"default_reg");
    ///range to cutoff sensor measurements
    ///
    param_nh.param("sensor_range",sensor_range,3.);

    ///visualize in a local window
    param_nh.param("visualize",visualize,true);

    std::string marker_str;
    param_nh.param<std::string>("plot_marker",marker_str,"sphere");
    if(marker_str.compare("sphere")==0)
      plot_marker=plotmarker::sphere;
    else if(marker_str.compare("point")==0)
      plot_marker=plotmarker::point;
    else
      plot_marker=plotmarker::sphere;



    ///range to cutoff sensor measurements
    param_nh.param("min_laser_range",min_laser_range_,0.1);


    ///if using the HMT fuser, NDT maps are saved in this directory.
    ///a word of warning: if you run multiple times with the same directory,
    ///the old maps are loaded automatically
    param_nh.param<std::string>("map_directory",map_dir,"/map/");
    param_nh.param<std::string>("map_type",map_type_name,"default_map");
    param_nh.param<std::string>("file_format_map",file_format_map,".JFF");


    param_nh.param("resolution",resolution,1.);

    ///initial pose of the vehicle with respect to the map
    param_nh.param("pose_init_x",pose_init_x,0.);
    param_nh.param("pose_init_y",pose_init_y,0.);
    param_nh.param("pose_init_z",pose_init_z,0.);
    param_nh.param("pose_init_r",pose_init_r,0.);
    param_nh.param("pose_init_p",pose_init_p,0.);
    param_nh.param("pose_init_t",pose_init_t,0.);

    ///pose of the sensor with respect to the vehicle odometry frame
    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
    param_nh.param("sensor_offset_t",sensor_offset_t_,0.);
    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
    param_nh.param("size_x_meters",size_x,10.);
    param_nh.param("size_y_meters",size_y,10.);
    param_nh.param("size_z_meters",size_z,10.);

    param_nh.param<double>("motion_params_Cd", motion_params.Cd, 0.005);
    param_nh.param<double>("motion_params_Ct", motion_params.Ct, 0.01);
    param_nh.param<double>("motion_params_Dd", motion_params.Dd, 0.001);
    param_nh.param<double>("motion_params_Dt", motion_params.Dt, 0.01);
    param_nh.param<double>("motion_params_Td", motion_params.Td, 0.001);
    param_nh.param<double>("motion_params_Tt", motion_params.Tt, 0.005);

    bool do_soft_constraints;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    param_nh.param("laser_variance_z",varz,resolution/4);

    param_nh.param<std::string>("bagfile_name",bag_name,"data.bag");
    cout<<"bagfile_name"<<points_topic<<endl;


    ///if we want to create map based on GT pose
    param_nh.param("renderGTmap",gt_mapping,false);
    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
    ///if we want to get the initial pose of the vehicle relative to a different frame
    param_nh.param("initPoseFromGT",initPoseFromGT,false);
    //plot the map from the GT track if available


    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

    param_nh.param<std::string>("odometry_adjusted",odometry_adjusted_topic,"odometry_adjusted");
    //get it from TF?
    param_nh.param("initPoseFromTF",initPoseFromTF,false);

    //the frame to initialize to

	  param_nh.param<std::string>("world_frame",world_link_id,"/world");
	  param_nh.param<std::string>("map_frame",map_link_id,"/map");
    //our frame
    param_nh.param<std::string>("fuser_frame_id",fuser_base_link_id,"/fuser_base_link");
    param_nh.param<std::string>("laser_frame_id",laser_link_id,"/velodyne");

    param_nh.param<std::string>("state_base_link_id",state_base_link_id,"/state_base_link");

    ///use standard odometry messages for initialuess
    param_nh.param("useOdometry",useOdometry,true);

    param_nh.param<bool>("use_tf_listener", use_tf_listener_, false);
    param_nh.param<std::string>("odometry_frame_id", odometry_link_id, std::string("/odom_base_link"));

    initPoseSet = false;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);


    param_nh.param("init_pose_tf",init_pose_tf_,false);
    param_nh.param<std::string>("robot_frame",robot_frame,"/base_link");


	param_nh.param("use_mcl",use_mcl_,true);
	param_nh.param("use_graph_map_registration",use_graph_map_registration_,true);
	param_nh.param<double>("zfilter_min",zfilter_min_,0);

    fuser_odom.header.frame_id=world_link_id;
    adjusted_odom_msg.header.frame_id=world_link_id;
    laser_publisher_ =param_nh.advertise<sensor_msgs::LaserScan>("laserscan_in_fuser_frame",50);

    point2_publisher_ =param_nh.advertise<sensor_msgs::PointCloud2>("point2_fuser",15);
    fuser_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("fuser",50);
    adjusted_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("odom_gt_init",50);

    param_nh.param<double>("fraction", fraction, 1.0);
    param_nh.param<double>("cutoff", cutoff, -10);
    param_nh.param<double>("init_var", initVar, 0.5);
	cov_x_mcl, cov_y_mcl, cov_yaw_mcl, scale_gaussian_mcl;
    param_nh.param<double>("cov_x_mcl", cov_x_mcl, 0.5);
    param_nh.param<double>("cov_y_mcl", cov_y_mcl, 0.5);
    param_nh.param<double>("cov_yaw_mcl", cov_yaw_mcl, 0.5);
	  param_nh.param<double>("scale_gaussian_mcl", scale_gaussian_mcl, 0.005);
	  param_nh.param<double>("cell_neighborhood_size_mcl_in_meters", _cell_neighborhood_size_mcl_in_meters, 1);
	  param_nh.param("use_euclidean_for_long_distances", _use_euclidean_for_long_distances, true);
	  param_nh.param("use_hybrid_strategy_mcl", _use_hybrid_strategy_mcl, true);
	  param_nh.param("use_euclidean_mcl", _use_euclidean_mcl, false);
	  param_nh.param("use_mean_score_mcl", _use_mean_score_mcl, false);
	  param_nh.param("publisher_timer", publisher_timer, false);

    if(gt_mapping)
      use_tf_listener_= use_tf_listener_ && state_base_link_id != std::string("");// check if odometry topic exists
    else
      use_tf_listener_= use_tf_listener_ && odometry_link_id != std::string("");// check if odometry topic exists

	std::cout << "SHould init from tf ? " << init_pose_tf_ << std::endl;

	sensorPose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
        Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

    tf::poseEigenToTF(sensorPose_,tf_sensor_pose_);

    if(!initPoseFromGT && !init_pose_tf_){
      pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
          Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
      initPoseSet=true;

      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_,sensorPose_);
      cout<<"set fuser viz="<<visualize<<endl;
      fuser_->Visualize(visualize);

    }


    cout<<"node: initial pose =\n"<<pose_.translation()<<endl;

	std::cout << "param \n" << matchLaser << " " << useOdometry << " " << gt_mapping << " " << use_tf_listener_ << std::endl;

	if(!matchLaser) {
			std::cout << "No match laser " << std::endl;
		points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,2);
		if(useOdometry) {
			if(gt_mapping){
			if(!use_tf_listener_){
				gt_fuser_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,gt_topic,10);
				sync_GTodom_ = new message_filters::Synchronizer< PointsGTOdomSync >(PointsGTOdomSync(SYNC_FRAMES), *points2_sub_, *gt_fuser_sub_);
				std::cout << "use no tf: " <<points_topic<< std::endl;
				sync_GTodom_->registerCallback(boost::bind(&GraphMapFuserNode::GTLaserPointsOdomCallback, this, _1, _2));
			}
			else
				std::cout << "use tf: " <<points_topic<< std::endl;
				points2OdomTfSub=nh_.subscribe <sensor_msgs::PointCloud2>(points_topic,10,&GraphMapFuserNode::GTLaserPointsOdomCallbackTF,this);

			}
			else{
			if(!use_tf_listener_){
				odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
				sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
				std::cout << "use no gt mapping no tf: " <<points_topic<< " and odom " << odometry_topic << std::endl;
				sync_po_->registerCallback(boost::bind(&GraphMapFuserNode::points2OdomCallback, this,_1, _2));
			}
			else{
				std::cout << "use no gt mapping tf: " <<points_topic<< std::endl;
				points2OdomTfSub=nh_.subscribe <sensor_msgs::PointCloud2>(points_topic,10,&GraphMapFuserNode::points2OdomCallbackTF,this);
			}
			}
		}
		}
		else
		{
		std::cout << "match laser: " << laser_topic << std::endl;
		laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,laser_topic,2);
		if(useOdometry) {
			std::cout << "USE ODOM" << std::endl;
			std::cout << "use no gt mapping no tf: " <<laser_topic<< " and odom " << odometry_topic << std::endl;
			odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
			sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
			sync_lo_->registerCallback(boost::bind(&GraphMapFuserNode::laserOdomCallback, this, _1, _2));
		}
		else{
			std::cout << "DO NOTHING " << std::endl;
	// 		  exit(0);
			laser_sub_->registerCallback(boost::bind( &GraphMapFuserNode::laserCallback, this, _1));
	// //         ((void)0); //Do nothing, seriously consider using a laser only callback (no odometry sync)
		}
		}
		if(initPoseFromGT) {
			std::cout << "init pose gt" << std::endl;
		gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&GraphMapFuserNode::gt_callback, this);
		}
		save_map_ = param_nh.advertiseService("save_map", &GraphMapFuserNode::save_map_callback, this);
		cout<<"init done"<<endl;

		//Publisher of graph_map message

	  graphmap_pub_ = param_nh.advertise<::graph_map::GraphMapMsg>("graph_map",50);
	  graph_map_vector_ = param_nh.advertise<ndt_map::NDTVectorMapMsg>("graph_map_vector",50);
	  graphmap_localization_pub = param_nh.advertise<auto_complete_graph::GraphMapLocalizationMsg>("graph_map_localization", 50);

	if(use_mcl_){


		//TEST
// // 		std::string file = "/home/malcolm/Documents/basement2d_map.jff";
// // 	// 	map.loadFromJFF(file.c_str());
// // 		double resolution = 0.2;
// // 		auto mapGrid = new perception_oru::LazyGrid(resolution);
// // 		perception_oru::NDTMap map(mapGrid);
// // 		if(map.loadFromJFF(file.c_str()) < 0)
// // 			std::cout << "File didn't load" << std::endl;
// // 		std::cout << "File loaded" << std::endl;
// // 		mcl_loaded_ = true;


	// 	mcl
	// 	perception_oru::NDTMap ndtmap;
		std::cout << "new mcl" << std::endl;
// 		NDTMCL* ndtmcl = new NDTMCL(resolution, map, -0.5);

// 		NDTMCL* ndtmcl = new NDTMCL();
// 		std::cout << "new mcl done" << std::endl;
// 		ndtmcl_ = boost::shared_ptr<NDTMCL>(ndtmcl);
		ndt_mcl_map = nh_.subscribe<auto_complete_graph::ACGMaps>("acg_maps", 10, &GraphMapFuserNode::updateAll, this);

		mcl_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_mcl",10);

		laser_point_cloud_pub_ =param_nh.advertise<sensor_msgs::PointCloud2>("mcl_point_cloud",50);

		mcl_local_map_ = param_nh.advertise<ndt_map::NDTMapMsg>("local_mcl_map",50);
		mcl_map_pub_ = param_nh.advertise<ndt_map::NDTMapMsg>("mcl_map_internal",50);
		particlePub_ = param_nh.advertise<geometry_msgs::PoseArray>("particles", 1);

		std::cout << "Init mcl done" << std::endl;

		publisher_graph_map=nh_.subscribe<std_msgs::Bool::ConstPtr>("/pub_graph_map_now", 10, &GraphMapFuserNode::pubGraphMap, this);


	}
	else{
		std::cout << "ATTENTION NO MCL" << std::endl;
	}

	  timer_transformation_map = nh_.createTimer(ros::Duration(0.1), &GraphMapFuserNode::timerCallback, this);

	  Transform_map.setIdentity();

	ROS_INFO("All init done 1" );

  }



	void pubGraphMap(const std_msgs::Bool::ConstPtr msg) {

		ROS_INFO("Publishing graph map message");
		ndt_map::NDTVectorMapMsg vector_maps;
		perception_oru::graph_map::graphMapToVectorMap(*(fuser_->GetGraph()), vector_maps, world_link_id);
		graph_map_vector_.publish(vector_maps);

		//Save the new pose associated with the node.
//		assert(nb_of_node_new - 1 > 0);

//				 acg_localization->savePos(nb_of_node_new - 1);
		ROS_DEBUG("PUBLISH: now");
		//Publish message
		::graph_map::GraphMapMsg graphmapmsg;
		perception_oru::graph_map::graphMapToMsg(*(fuser_->GetGraph()), graphmapmsg, map_link_id);

		assert(times.size() == graphmapmsg.nodes.size());
		int count = 0;
		for(auto time : times){
			graphmapmsg.nodes[count].time_sec.data = time.first;
			graphmapmsg.nodes[count].time_nsec.data = time.second;
			std::cout << "Adding the time "  << std::fixed << std::setprecision(8) << (double)time.first << " " << (double)time.second << std::endl;
			count++;
		}


		// 			std::cout << "PUBLISH " << graphmapmsg.nodes.size() << std::endl;
		//
		if (use_mcl_ && mcl_loaded_) {
			ROS_DEBUG("*************************** > Real Localization < ****************************");

//			std::cout << acg_localization->getLocalizations().size() << " == " <<  nb_of_node_new << std::endl;
//			assert(acg_localization->getLocalizations().size() == nb_of_node_new);

			auto_complete_graph::GraphMapLocalizationMsg graphmaplocalizationmsg;
			graphmaplocalizationmsg.graph_map = graphmapmsg;
			acg_localization->toMessage(graphmaplocalizationmsg);
			graphmap_localization_pub.publish(graphmaplocalizationmsg);
			//Export all localization information.
		}
		else{

			ROS_DEBUG("*************************** > Mock Localization < ****************************");

			auto_complete_graph::GraphMapLocalizationMsg graphmaplocalizationmsg;
			graphmaplocalizationmsg.graph_map = graphmapmsg;
			mockLocalizationMessage(graphmaplocalizationmsg, fuser_->GetGraph()->GetNodes().size());
			graphmap_localization_pub.publish(graphmaplocalizationmsg);

//					 bool unstop = true;
//					 while(nb_of_node_new >= 6 && unstop){
//						 std::cout << "Keep publishing ? " << std::endl;
//						 std::cin >> unstop;
//						 graphmap_localization_pub.publish(graphmaplocalizationmsg);
//
//					 }

		}

		graphmap_pub_.publish(graphmapmsg);
		ROS_INFO("Published the graph msg");
		// 			std::cout << "PUBLISHED" << std::endl;
		// 		exit(0);
	}



		geometry_msgs::PoseArray ParticlesToMsg(std::vector<perception_oru::particle, Eigen::aligned_allocator<perception_oru::particle> > particles){
		//ROS_DEBUG("publishing particles");
		geometry_msgs::PoseArray ret;
		for(int i = 0; i < particles.size(); i++){
			geometry_msgs::Pose temp;
			double x, y, z, r, p, t;
			particles[i].GetXYZ(x, y, z);
			particles[i].GetRPY(r, p, t);
			temp.position.x = x;
			temp.position.y = y;
			temp.position.z = z;
			temp.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, t);
			ret.poses.push_back(temp);
		}
		ret.header.frame_id = world_link_id;
		return ret;
	}

	void updateAll(const auto_complete_graph::ACGMaps::ConstPtr& acg_maps){

		std::cout << "Updating the prior and graph map" << std::endl;
		loadNDTMap(acg_maps->prior);
//		AASS::acg::updateGraphMap(acg_maps, fuser_->GetGraph());

	}

	void updateMCLNDTMap(const ndt_map::NDTMapMsg& mapmsg){
		if (use_mcl_ && mcl_loaded_) {

			perception_oru::NDTMap* map;
			perception_oru::LazyGrid* lz;
			std::string frame;
			perception_oru::fromMessage(lz, map, mapmsg, frame, false, false);
			acg_localization->setMap(*map);
			delete map;

		}
	}

	void initMCL(const ndt_map::NDTMapMsg& mapmsg){
		double numPart = 250;
		bool forceSIR = false;

		ROS_INFO("INIT MCL FROM TF");
		auto init_pose = getPoseTFTransform(world_link_id, laser_link_id);
//		std::cout << "Pose found " << pose_.matrix() << std::endl;

		perception_oru::NDTMap *map;
		perception_oru::LazyGrid *lz;
		std::string frame;
		perception_oru::fromMessage(lz, map, mapmsg, frame, false, false);

//		std::cout << "GOT MAP FROM MESSAGE" << std::endl;

// 		ndtmcl_->changeMapAndInitializeFilter(*map, x, y, yaw, v_x, v_y, v_yaw, numPart);

		double xx = init_pose.getOrigin().x();
		double yy = init_pose.getOrigin().y();
		double zz = init_pose.getOrigin().z();
		double roll, pitch, yaw;
		init_pose.getBasis().getRPY(roll, pitch, yaw);


		AASS::acg::ACGMCLLocalization *pMCL = new AASS::acg::ACGMCLLocalization(map, numPart, true, forceSIR, 0, SIRCount);
		acg_localization = boost::shared_ptr<AASS::acg::ACGMCLLocalization>(pMCL);

		acg_localization->init(xx, yy, yaw, initVar, cov_x_mcl, cov_y_mcl, cov_yaw_mcl, scale_gaussian_mcl, numPart,
		                       forceSIR);

		acg_localization->setVarNewParticle(var_part);

		acg_localization->useEuclideanDistance(_use_euclidean_mcl);
		acg_localization->setCellNeighborToConsiderInMeters(_cell_neighborhood_size_mcl_in_meters);
		acg_localization->useEuclideanForLongDistances(_use_euclidean_for_long_distances);
		acg_localization->useHybridStrategy(_use_hybrid_strategy_mcl);
		acg_localization->useMeanOfAllScores(_use_mean_score_mcl);


		auto sensorpose_tmp = getPoseTFTransform(robot_frame, laser_link_id);
		//Only do the rotation:
		double x = sensorpose_tmp.getOrigin().getX();
		double y = sensorpose_tmp.getOrigin().getY();
		double z = sensorpose_tmp.getOrigin().getZ();
//			std::cout << "xyz : " << x << " " << y << " " << z << std::endl;
		//TEST
		// 	x = 16.6;
		// 	y = 3.0;
		double rollt, pitcht, yawt;
		sensorpose_tmp.getBasis().getRPY(rollt, pitcht, yawt);


		Eigen::Affine3d sensorpose = Eigen::Translation<double,3>(x,y,z)*
		                             Eigen::AngleAxis<double>(rollt,Eigen::Vector3d::UnitX()) *
		                             Eigen::AngleAxis<double>(pitcht,Eigen::Vector3d::UnitY()) *
		                             Eigen::AngleAxis<double>(yawt,Eigen::Vector3d::UnitZ()) ;
		acg_localization->setSensorPose(sensorpose);


// 		perception_oru::particle_filter* pMCL = new perception_oru::particle_filter(map, numPart, true, forceSIR);
// 		ndtmcl_ = boost::shared_ptr<perception_oru::particle_filter>(pMCL);

// 		ndtmcl_->setMotionModelCovX(cov_x_mcl);
// 		ndtmcl_->setMotionModelCovY(cov_y_mcl);
// 		ndtmcl_->setMotionModelCovYaw(cov_yaw_mcl);
// 		ndtmcl_->setScalingFactorGaussian(scale_gaussian_mcl);

// 		std::cout << "Init at " << xx << " " << yy << " " << zz << std::endl; exit(0);
// 		ndtmcl_->InitializeNormal(xx, yy, yaw, initVar);

//         ndtmcl_->changeMapAndInitializeFilter(*map, resolution, zfilter_min_, xx, yy, yaw, x_va, y_va, yaw_va, numPart);
		mcl_loaded_ = true;

		delete map;

		//ATTENTION MEMORY LEAK :(. But if I delete it it's deleted from the particule filter...
// 		delete map;

		ROS_DEBUG("MAP LOADED SUCCESSFULLY :)");
	}

	void loadNDTMap(const ndt_map::NDTMapMsg& mapmsg){

		if(!mcl_loaded_) {
			ROS_INFO("Init new prior map");
			initMCL(mapmsg);
		}
		else{
			ROS_INFO("Loading new prior map");
			updateMCLNDTMap(mapmsg);
		}

	}

	///Laser scan to PointCloud expressed in the base frame
	std::tuple<Eigen::Vector3d, Eigen::Matrix3d> localization( const Eigen::Affine3d& Tmotion, const pcl::PointCloud<pcl::PointXYZ>& cloud, ros::Time time = ros::Time::now()){


		if(mcl_loaded_ = true){

//			std::cout << "MCL Localization. Sensor frame between : " <<  robot_frame << " , " << laser_link_id << std::endl;

			auto sensorpose_tmp = getPoseTFTransform(robot_frame, laser_link_id);
			//Only do the rotation:
			double x = sensorpose_tmp.getOrigin().getX();
			double y = sensorpose_tmp.getOrigin().getY();
			double z = sensorpose_tmp.getOrigin().getZ();
//			std::cout << "xyz : " << x << " " << y << " " << z << std::endl;
			//TEST
		// 	x = 16.6;
		// 	y = 3.0;
			double roll, pitch, yaw;
			sensorpose_tmp.getBasis().getRPY(roll, pitch, yaw);


			Eigen::Affine3d sensorpose = Eigen::Translation<double,3>(x,y,z)*
			Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;

			Eigen::Matrix3d cov;
			Eigen::Vector3d mean;

			std::tie(mean, cov) = acg_localization->localization(Tmotion, cloud, sensorpose, fraction, cutoff);

			pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
			pcl::transformPointCloud(cloud, transformed_cloud, sensorpose);
//
// // 			sensor_msgs::PointCloud2 pcloudmsg;
// // 			pcl::toROSMsg(transformed_cloud, pcloudmsg);
// // 			pcloudmsg.header.frame_id = world_link_id;
// // 			pcloudmsg.header.stamp = ros::Time::now();
// // 			laser_point_cloud_pub_.publish(pcloudmsg);
//
// 			std::cout << "Resolution should be 0.2 : " << resolution << std::endl;
// 			double cellx, celly, cellz;
// 			bool out = ndtmcl_->getReferenceMapCellSizes(cellx, celly, cellz);
//
// 			std::cout << "Got size" << out << std::endl;
//
// 			std::cout << "Cells size " << cellx << " " << celly << " " << cellz << std::endl;
//
// 			assert(cellx == celly);
// 			assert(celly == cellz);
//
// 			if(cellx != celly) exit(0);
// 			if(celly != cellz) exit(0);
//
// 			//Cellx/celly/cellz represent the resolution of the internal map of the localization if they are all equal.
//
// 			perception_oru::NDTMap *localMap = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
// 			localMap->loadPointCloud(transformed_cloud);
// 			localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
//
// 			ndt_map::NDTMapMsg mapmsgndt;
// 			perception_oru::toMessage(localMap, mapmsgndt, world_link_id);
// 			mcl_local_map_.publish(mapmsgndt);
//
// 			ndt_map::NDTMapMsg mapmsgndt2;
// 			perception_oru::toMessage(ndtmcl_->getMap(), mapmsgndt2, world_link_id);
// 			mcl_map_pub_.publish(mapmsgndt2);

// 			cloud.clear();
// 			transformed_cloud->clear();

// 			auto pose_of_a_mcl_particule = ndtmcl_->UpdateAndPredictEff(Tmotion, localMap, fraction, cutoff);
			// MCL->UpdateAndPredict(tMotion, localMap);

// 			delete localMap;

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/// Now we have the sensor origin and pointcloud -- Lets do MCL
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 			ndtmcl_->updateAndPredict(Tmotion, cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)
// 			std::cout << "update and predicted" << std::endl;

// 			Eigen::Vector3d dm = ndtmcl_->getMean(); ///Maximum aposteriori pose
// 			Eigen::Matrix3d cov = ndtmcl_->pf.getDistributionVariances(); ///Pose covariance


			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 		isFirstLoad = false;
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			nav_msgs::Odometry mcl_pose;
// 			Eigen::Vector3d mean_pose=ndtmcl_->GetMeanPose2D();

// 			ROS_DEBUG_STREAM(mean_pose);

// 			Pose2DToTF(mean_pose,time,T);
			//ROS_DEBUG_STREAM(T);
// 			ndtmcl_->GetPoseMeanAndVariance2D(mean, cov);
			mcl_pose = Pose2DToMsg(mean, cov, time, T);

			mcl_pose.header.stamp = time;
			mcl_pub_.publish(mcl_pose);



			Eigen::Affine3d sensorpose2 = Eigen::Translation<double,3>(mean(0),mean(1),0)*
			Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxis<double>(mean(2),Eigen::Vector3d::UnitZ()) ;

			pcl::PointCloud<pcl::PointXYZ> transformed_cloud2;

			pcl::transformPointCloud(cloud, transformed_cloud2, sensorpose2);

			sensor_msgs::PointCloud2 pcloudmsg;
			pcl::toROSMsg(transformed_cloud2, pcloudmsg);
			pcloudmsg.header.frame_id = world_link_id;
			pcloudmsg.header.stamp = ros::Time::now();
			laser_point_cloud_pub_.publish(pcloudmsg);


//			perception_oru::NDTMap *localMap = new perception_oru::NDTMap(new perception_oru::LazyGrid(0.5));
//			localMap->loadPointCloud(transformed_cloud);
//			localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
//
//
//
//			delete localMap;


			particlePub_.publish(ParticlesToMsg(acg_localization->particleCloud));

		// 	if(counter%50==0){
		// 		sendMapToRviz(ndtmcl->map);
		// 	}

			return std::make_tuple(mean, cov);
		//
		}
		else{
			ROS_INFO("You need to init MCL to start MCL localization");
			Eigen::Matrix3d cov;
			Eigen::Vector3d mean;
			return std::make_tuple(mean, cov);
		}
	}

	void Pose2DToTF(Eigen::Vector3d mean, ros::Time ts, Eigen::Affine3d Todometry){
		static tf::TransformBroadcaster br, br_mapOdom;
		tf::Transform transform;
		tf::Quaternion q;
		q.setRPY(0, 0, mean[2]);
		transform.setOrigin( tf::Vector3(mean[0], mean[1], 0.0) );
		transform.setRotation( q );
		br.sendTransform(tf::StampedTransform(transform, ts, world_link_id, "mcl_tf"));
		// // br.sendTransform(tf::StampedTransform(transform, ts, rootTF, "/mcl_pose"));
		// ///Compute TF between map and odometry frame
		// Eigen::Affine3d Tmcl = getAsAffine(mean[0], mean[1], mean[2]);
		// Eigen::Affine3d Tmap_odo = Tmcl * Todometry.inverse();
		// tf::Transform tf_map_odo;
		// tf_map_odo.setOrigin( tf::Vector3(Tmap_odo.translation() (0), Tmap_odo.translation() (1), 0.0) );
		// tf::Quaternion q_map_odo;
		// q_map_odo.setRPY( 0, 0, Tmap_odo.rotation().eulerAngles(0, 1, 2) (2) );
		// tf_map_odo.setRotation( q_map_odo );
		// // /// broadcast TF
		//br_mapOdom.sendTransform(tf::StampedTransform(tf_map_odo, ts + ros::Duration(0.3), rootTF, odomTF));
	}

	nav_msgs::Odometry Pose2DToMsg(Eigen::Vector3d mean, Eigen::Matrix3d cov, ros::Time ts, Eigen::Affine3d Todometry){
		nav_msgs::Odometry O;
		static int seq = 0;

		//	O.header.stamp = ts;
		//O.header.seq = seq;
		O.header.frame_id = world_link_id;
		O.child_frame_id = "mcl_tf";

		O.pose.pose.position.x = mean[0];
		O.pose.pose.position.y = mean[1];
		tf::Quaternion q;
		q.setRPY(0, 0, mean[2]);
		O.pose.pose.orientation.x = q.getX();
		O.pose.pose.orientation.y = q.getY();
		O.pose.pose.orientation.z = q.getZ();
		O.pose.pose.orientation.w = q.getW();

		O.pose.covariance[0] = cov(0, 0);
		O.pose.covariance[1] = cov(0, 1);
		O.pose.covariance[2] = 0;
		O.pose.covariance[3] = 0;
		O.pose.covariance[4] = 0;
		O.pose.covariance[5] = 0;

		O.pose.covariance[6] = cov(1, 0);
		O.pose.covariance[7] = cov(1, 1);
		O.pose.covariance[8] = 0;
		O.pose.covariance[9] = 0;
		O.pose.covariance[10] = 0;
		O.pose.covariance[11] = 0;

		O.pose.covariance[12] = 0;
		O.pose.covariance[13] = 0;
		O.pose.covariance[14] = 0;
		O.pose.covariance[15] = 0;
		O.pose.covariance[16] = 0;
		O.pose.covariance[17] = 0;

		O.pose.covariance[18] = 0;
		O.pose.covariance[19] = 0;
		O.pose.covariance[20] = 0;
		O.pose.covariance[21] = 0;
		O.pose.covariance[22] = 0;
		O.pose.covariance[23] = 0;

		O.pose.covariance[24] = 0;
		O.pose.covariance[25] = 0;
		O.pose.covariance[26] = 0;
		O.pose.covariance[27] = 0;
		O.pose.covariance[28] = 0;
		O.pose.covariance[29] = 0;

		O.pose.covariance[30] = 0;
		O.pose.covariance[31] = 0;
		O.pose.covariance[32] = 0;
		O.pose.covariance[33] = 0;
		O.pose.covariance[34] = 0;
		O.pose.covariance[35] = cov(2, 2);

		seq++;
		//    ROS_DEBUG("publishing tf");
// 		static tf::TransformBroadcaster br, br_mapOdom;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(mean[0], mean[1], 0.0) );
		transform.setRotation( q );
		//br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));
		tf_.sendTransform(tf::StampedTransform(transform, ts, world_link_id, "mcl_tf"));
		///Compute TF between map and odometry frame
// 		Eigen::Affine3d Tmcl = getAsAffine(mean[0], mean[1], mean[2]);
// 		Eigen::Affine3d Tmap_odo = Tmcl * Todometry.inverse();
// 		tf::Transform tf_map_odo;
// 		tf_map_odo.setOrigin( tf::Vector3(Tmap_odo.translation() (0), Tmap_odo.translation() (1), 0.0) );
// 		tf::Quaternion q_map_odo;
// 		q_map_odo.setRPY( 0, 0, Tmap_odo.rotation().eulerAngles(0, 1, 2) (2) );
// 		tf_map_odo.setRotation( q_map_odo );
// 		// /// broadcast TF
// 		tf_.sendTransform(tf::StampedTransform(tf_map_odo, ts, world_link_id, "mcl_tf"));
		return O;
	}

	Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
		Eigen::Matrix3d m;

		m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
		Eigen::Translation3d v(x, y, 0);
		Eigen::Affine3d T = v * m;
		return T;
	}


	void timerCallback(const ros::TimerEvent&){
		publishMapTransform();
	}

	void publishMapTransform( const ros::Time& time=ros::Time::now() ) {
		if (publisher_timer){
//			std::cout << "Publishing at time " << time << std::endl;
			tf_.sendTransform(tf::StampedTransform(Transform_map, time, map_link_id, world_link_id));
		}
	}




	void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion, const ros::Time& time=ros::Time::now() ) {

//	  std::cout << "MOTION !!! " << Tmotion.matrix() << std::endl;
		std::cout << "Time : " << time.toSec() << " or " << time.sec << " " << time.nsec << std::endl;

	 Eigen::Vector3d mcl_mean;
	 Eigen::Matrix3d mcl_cov;
	 if(use_mcl_ && mcl_loaded_){
		 if(!use_graph_map_registration_){
			 frame_nr_++;
		 }
		 std::tie(mcl_mean, mcl_cov) = localization(Tmotion, cloud);
	 }

	 if(use_graph_map_registration_ == true){

		 if(!use_mcl_ || (use_mcl_ && mcl_loaded_) ) {

			 if (init_pose_tf_ == true && init_fuser_ == false) {
				 ROS_DEBUG("INIT FROM TF");
				 // 		sensorPose_ = getPose(robot_frame, laser_link_id);

				 /////USED FOR BASEMENT BAG FILES
				 // 		//Probably need this because the sensor is too hight above the map otherwise. Or something equally weird.
//				 double roll = 0, pitch = 0, yaw = 3.1415;
//				 // 		getAngles(robot_frame, laser_link_id, roll, pitch, yaw);
//
//				 sensorPose_ = Eigen::Translation<double, 3>(0, 0, 0) *
//				               Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX()) *
//				               Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) *
//				               Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
//
////				 std::cout << "SENSOR " << sensorPose_.matrix() << std::endl;
//
//				 tf::poseEigenToTF(sensorPose_, tf_sensor_pose_);

				 pose_ = getPose(world_link_id, robot_frame);
//				 std::cout << "Pose found " << pose_.matrix() << std::endl;
				 sensorPose_ = getPose(robot_frame, laser_link_id);

//				 std::cout << "\n\nSENSOR actuel " << sensorPose_.matrix() << std::endl;
				 // 			exit(0);
//				 std::cout << "Fuser " << fuser_ << std::endl;
				 assert(fuser_ == NULL);
				 fuser_ = new GraphMapFuser(map_type_name, reg_type_name, pose_, sensorPose_);
				 ROS_DEBUG_STREAM( "----------------------------FUSER------------------------" );
				 ROS_DEBUG_STREAM( fuser_->ToString() );
				 fuser_->Visualize(visualize, plot_marker);
				 fuser_->SetFuserOptions(false);
				 ROS_DEBUG_STREAM( "---------------------------------------------------------" );
				 initPoseSet = true;
				 init_fuser_ = true;

				 times.push_back(std::pair<uint32_t, uint32_t>(time.sec, time.nsec) );

//				 std::cout << "Pose " << pose_.matrix() << std::endl;
//				 std::cout << "SENSOR " << sensorPose_.matrix() << std::endl;

				 // 		int a;
				 // 		std::cin >> a ;

				 // 		exit(0);

			 }


			 if (!initPoseSet)
				 return;

			 frame_nr_++;
//			 cout << "frame nr=" << frame_nr_ << endl;
			 if ((Tmotion.translation().norm() < 0.005 && Tmotion.rotation().eulerAngles(0, 1, 2).norm() < 0.005) &&
			     useOdometry) {    //sanity check for odometry
				 //       return;
			 }
			 (void) ResetInvalidMotion(Tmotion);

			 ROS_DEBUG_STREAM("Last pose " << fuser_->GetPoseLastFuse().matrix() << std::endl );
			 ROS_DEBUG_STREAM("From this " << fuser_->GetPoseLastFuse().inverse().matrix() << " * " << pose_.matrix()
			 );

			 ROS_DEBUG_STREAM("frame=" << frame_nr_ << "movement="
			      << (fuser_->GetPoseLastFuse().inverse() * pose_).translation().norm() );

			 auto nb_of_node = fuser_->GetGraph()->GetNodes().size();

			 ros::Time tplot = ros::Time::now();
			 plotPointcloud2(cloud, tplot);
			 m.lock();
			 fuser_->ProcessFrame<pcl::PointXYZ>(cloud, pose_, Tmotion);
			 m.unlock();
			 fuser_->PlotMapType();
			 tf::Transform Transform;
			 tf::transformEigenToTF(pose_, Transform);

			 // 	std::cout << "POSE NOW" << pose_.matrix() << std::endl;
			  	std::cout << "Transform " << Transform.getOrigin().getX() << " " << Transform.getOrigin().getY() << " " << Transform.getOrigin().getZ() << " between " << world_link_id << " " << fuser_base_link_id << std::endl;


			 tf_.sendTransform(tf::StampedTransform(Transform, time, world_link_id, fuser_base_link_id));
			 // 	auto pose_tmp = getPose(world_link_id, fuser_base_link_id, tplot);
			 // 	std::cout << "Trans NOW" << pose_tmp.matrix() << std::endl;
			 // 	assert(pose_tmp == pose_);
			 // 	exit(0);
			 //I'm not sure why this transformation is published :/
			 //     tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, tplot, fuser_base_link_id, "/fuser_laser_link"));
			 fuser_odom.header.stamp = time;

			 tf::poseEigenToMsg(pose_, fuser_odom.pose.pose);
			 fuser_odom_publisher_.publish(fuser_odom);


			 //Publish transform from map to world.
			 auto robot_pose = getPose(world_link_id, robot_frame, time);
			 Eigen::Affine3d map_to_world = pose_ * robot_pose.inverse();

			 tf::transformEigenToTF(map_to_world, Transform_map);
			 tf_.sendTransform(tf::StampedTransform(Transform_map, time, map_link_id, world_link_id));

			 publishMapTransform(time);


			 auto nb_of_node_new = fuser_->GetGraph()->GetNodes().size();
			  	std::cout << "Well " << nb_of_node << " != " << nb_of_node_new << " and id " << fuser_->GetGraph()->GetNodes()[0]->GetId() << std::endl;
			 for (int i = 0; i < nb_of_node_new; ++i) {
				 ROS_DEBUG_STREAM( "Well " << nb_of_node << " != " << nb_of_node_new << " and id "
				           << fuser_->GetGraph()->GetNodes()[i]->GetId() );
			 }

			 if (use_mcl_ && mcl_loaded_) {
				 //If a new node was added, we save the location.
				 if (acg_localization->getLocalizations().size() != nb_of_node_new) {
					 ROS_DEBUG_STREAM(" saving pose ");
					 acg_localization->savePos(nb_of_node_new - 1);

					 //Taking current cov for precedent node
					 if(acg_localization->getLocalizations().size() >= 2 ) {
						 acg_localization->saveCov(nb_of_node_new - 2);
					 }

					 assert(acg_localization->getLocalizations().size() == nb_of_node_new);
				 }
				 assert(acg_localization->getLocalizations().size() == nb_of_node_new);
			 }




			 if (nb_of_node != nb_of_node_new) {
				 times.push_back(std::pair<uint32_t, uint32_t>(time.sec, time.nsec));
			    std_msgs::Bool::Ptr bool_msg;
				pubGraphMap(bool_msg);

			 }
		 }
		 else{
			 ROS_DEBUG_STREAM("NDT MCL needs to be init for the registration to start if you want to use both MCL and registration");
		 }

	 }

  }


	void mockLocalizationMessage(auto_complete_graph::GraphMapLocalizationMsg& msg, int how_many){

		for(int i = 0; i < how_many ; ++i) {
			AASS::acg::Localization loc;
			auto_complete_graph::LocalizationMsg loc_msg = AASS::acg::toMessage(loc);
			msg.localizations.push_back(loc_msg);
		}


	}

  //bool save_map_callback(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res )
  bool ResetInvalidMotion(Eigen::Affine3d &Tmotion){
    if(Tmotion.translation().norm() > MAX_TRANSLATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max transl)!\n";
      std::cerr<<Tmotion.translation().transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else if(Tmotion.rotation().eulerAngles(0,1,2)(2) > MAX_ROTATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max rot)!\n";
      std::cerr<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else return false;
  }
  bool save_map_callback(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res ) {
    char path[1000];
    string time=ndt_generic::currentDateTimeString();

    if(fuser_!=NULL){
      snprintf(path,999,"%s/%s_.MAP",map_dir.c_str(),time.c_str());
      m.lock();
      if(file_format_map.compare(".JFF")==0)
        fuser_->SaveCurrentNodeAsJFF(path);
      else
        fuser_->SaveGraphMap(path);

      m.unlock();
      ROS_DEBUG("Current map was saved to path= %s", path);
      return true;
    }
    else
      ROS_DEBUG("No data to save");
    return false;
  }

  inline bool getAffine3dTransformFromTF(const ros::Time &time,const std::string &link_id,Eigen::Affine3d& ret,const ros::Duration &wait) {
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    tf_listener_.waitForTransform(world_link_id, link_id,  time ,wait);
    try{
      tf_listener_.lookupTransform(world_link_id, link_id, time, transform);
      tf::poseTFToEigen(transform, ret);
	    ROS_DEBUG_STREAM("found "<<ret.translation().transpose());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
    return true;
  }

  // Callback
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
	  ROS_DEBUG_STREAM("laser callback");
// 	exit(0);
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZ pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    T.setIdentity();
	  ROS_DEBUG_STREAM("node: laser call back, process frame");
    this->processFrame(pcl_cloud,T, msg_in->header.stamp);


  }

  // Callback
  void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
                         const nav_msgs::Odometry::ConstPtr& odo_in)
  {
	  ROS_DEBUG_STREAM("laser odom callback");
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
    Eigen::Affine3d Tm;

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);
    if (frame_nr_  <= 1){
	    ROS_DEBUG_STREAM("Identeity motion: " << frame_nr_ );
      Tm.setIdentity();
    }
    else{

      Tm = last_odom.inverse()*this_odom;
	    ROS_DEBUG_STREAM("new Tmotion: " <<frame_nr_  << "\n"<< Tm.matrix() );
	}

    last_odom = this_odom;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);
    sensor_msgs::LaserScan msg_out=*msg_in;
    msg_out.header.stamp=ros::Time::now();
    msg_out.header.frame_id="/fuser_laser_link";
    laser_publisher_.publish(msg_out);
    pcl::PointXYZ pt;

    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    this->processFrame(pcl_cloud,Tm, msg_in->header.stamp);
	  ROS_DEBUG_STREAM("publish fuser data");
  }

  void plotPointcloud2(pcl::PointCloud<pcl::PointXYZ> & cloud,ros::Time time = ros::Time::now()){
	  ROS_DEBUG_STREAM("plot point cloud");
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(cloud,msg_out);
    msg_out.header.frame_id=laser_link_id;
    msg_out.header.stamp=time;
    point2_publisher_.publish(msg_out);
  }
  void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                           const nav_msgs::Odometry::ConstPtr& odo_in)//callback is used in conjunction with odometry time filter.
  {
//	  std::cout << "This function " << std::endl;
    ros::Time tstart=ros::Time::now();

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);

    Eigen::Affine3d Tm;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (frame_nr_ == 0)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom = this_odom;
    pcl::fromROSMsg (*msg_in, cloud);
    this->processFrame(cloud,Tm, msg_in->header.stamp);
    ros::Time tend=ros::Time::now();
//    cout<<"Total execution time= "<<tend-tstart<<endl;
  }
  void points2OdomCallbackTF(const sensor_msgs::PointCloud2::ConstPtr& msg_in){//this callback is used to look up tf transformation for scan data
//	  cout<<"point odom callback tf"<<endl;
    Eigen::Affine3d Tm;
    static bool last_odom_found=false;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg_in, cloud);
    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),odometry_link_id,this_odom,ros::Duration(0.1));
    if (frame_nr_ =0 || !found_odom||!last_odom_found)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom_found=found_odom;

    last_odom = this_odom;
    this->processFrame(cloud,Tm, msg_in->header.stamp);
//    cout<<"TF callback Point2Odom"<<endl;
  }

  void GTLaserPointsOdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                                 const nav_msgs::Odometry::ConstPtr& odo_in)//this callback is used for GT based mapping
  {
//    cout<<"GT diff:"<<(msg_in->header.stamp-odo_in->header.stamp).toSec()<<endl;
    Eigen::Affine3d Tmotion;
    if(frame_nr_==0){
      Tmotion=Eigen::Affine3d::Identity();
    }
    Eigen::Affine3d GT_pose;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::poseMsgToEigen(odo_in->pose.pose,pose_);
    pcl::fromROSMsg (*msg_in, cloud);
    ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
    tf::Transform gt_base;
    tf::poseMsgToTF(odo_in->pose.pose,gt_base);
    tf_.sendTransform(tf::StampedTransform(gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
    tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
    plotPointcloud2(cloud);
    m.lock();
    fuser_->ProcessFrame(cloud,pose_,Tmotion);
    fuser_->PlotMapType();
    m.unlock();
  }
  void GTLaserPointsOdomCallbackTF(const sensor_msgs::PointCloud2::ConstPtr& msg_in)//this callback is used for GT based mapping with TF lookup
  {
//    cout<<"GT-TF toffset:"<<sensor_offset_t_<<endl;
    Eigen::Affine3d tmp_pose;
    Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();
    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),state_base_link_id,tmp_pose,ros::Duration(0.1));

    if(found_odom){
      pose_=tmp_pose;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg (*msg_in, cloud);
      ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
      tf::Transform tf_gt_base;
      tf::poseEigenToTF(pose_,tf_gt_base);
      tf_.sendTransform(tf::StampedTransform(tf_gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
      tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
      plotPointcloud2(cloud,t_stamp);
      fuser_->ProcessFrame(cloud,pose_,Tmotion);
      fuser_->PlotMapType();
      m.unlock();
    }

  }
  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
//cout<<"gt callback"<<endl;
	  exit(0);
    Eigen::Affine3d gt_pose;
    tf::poseMsgToEigen(msg_in->pose.pose,gt_pose);

    if(initPoseFromGT && !initPoseSet) {
      pose_ = gt_pose;
      ROS_DEBUG("Set initial pose from GT track");
      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_,sensorPose_);
      cout<<"----------------------------FUSER------------------------"<<endl;
      cout<<fuser_->ToString()<<endl;
      fuser_->Visualize(visualize,plot_marker);
      cout<<"---------------------------------------------------------"<<endl;
      initPoseSet = true;

    }
  }

public:
  // map publishing function
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_map_fuser_node");
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::spinOnce();
  ros::NodeHandle param("~");
	/*while( (int)ros::Time(0).toSec() == 0 and ros::ok()){
		std::cout << "Not good " << ros::Time(0) << std::endl;
	}*/

  GraphMapFuserNode t(param);
	while(ros::ok()){
		ros::spinOnce();
	}

  return 0;
}



