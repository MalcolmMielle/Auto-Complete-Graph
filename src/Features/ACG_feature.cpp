#include "auto_complete_graph/ACG_feature.hpp"

void AASS::acg::AutoCompleteGraphFeature::extractCornerNDTMap(const std::shared_ptr< perception_oru::NDTMap >& map, AASS::acg::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos)
{
	
	//HACK For now : we translate the Corner extracted and not the ndt-maps
	auto cells = map->getAllCellsShared();
	std::cout << "got all cell shared" << std::endl;
	double x2, y2, z2;
	map->getCellSizeInMeters(x2, y2, z2);
	std::cout << "got all cell sized" << std::endl;
	double cell_size = x2;
	
	std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement> corners_end;	
	getAllCornersNDTTranslatedToGlobalAndRobotFrame(map, robot_ptr, robot_pos, corners_end);

	/***************** ADD THE CORNERS INTO THE GRAPH***********************/
	
	for(size_t i = 0 ; i < corners_end.size() ; ++i){
// 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;	
		bool seen = false;
		AASS::acg::VertexLandmarkNDT* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2D landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));
			
			double res = cv::norm(point_land - corners_end[i].position);
			
// 			std::cout << "res : " << res << " points "  << point_land << " " << corners_end[i].point << "  cell size " << cell_size << std::endl;
			
			//If we found the landmark, we save the data
			if( res < cell_size * 2){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}
		if(seen == false){
// 			std::cout << "New point " << i << " " <<  ret_opencv_point_corner.size() << std::endl;
// 			assert(i < ret_export.size());
			g2o::Vector2D position_globalframe;
			position_globalframe << corners_end[i].position.x, corners_end[i].position.y ;
// 			AASS::acg::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_export[i].getMeanOpenCV(), 1);
			
			cv::Point2f p_observation;
			p_observation.x = corners_end[i].getObservations()(0);
			p_observation.y = corners_end[i].getObservations()(1);
			AASS::acg::VertexLandmarkNDT* ptr = addLandmarkPose(position_globalframe, p_observation, 1);
			ptr->addAnglesOrientations(corners_end[i].getAngleWidths(), corners_end[i].getOrientations());
			ptr->first_seen_from = robot_ptr;
			
			//TESTING to visualise which cells gave the corner
			ptr->cells_that_gave_it_1 = corners_end[i].cells1;
			ptr->cells_that_gave_it_2 = corners_end[i].cells2;
			ptr->robotpose_seen_from = robot_pos;
			//END OF TEST
			
			std::cout << "Descriptors" << std::endl;
			/************************************************************/
			
			//Convert NDT MAP to image
			cv::Mat ndt_img;
			auto map_tmp = corners_end[i].getNodeLinkedPtr()->getMap();
			
			std::cout << "Descriptors" << std::endl;
			double size_image_max = 500;
			double min, max;
			//TODO: super convoluted, get rid of NDTCornerGraphElement!
			perception_oru::ndt_feature_finder::toCvMat(*map_tmp, ndt_img, size_image_max, max, min);
			
			//Use accurate CV point
			//Get old position_in_robot_frame
// 			std::cout << "Position " << ret_opencv_point_corner[i] << std::endl;
// 			cv::Point2f p_observation;
// 			p_observation.x = corners_end[i].getObservations()(0);
// 			p_observation.y = corners_end[i].getObservations()(1);
			cv::Point2i center = perception_oru::ndt_feature_finder::scalePoint(p_observation, max, min, size_image_max);
// 			std::cout << "Position " << center << "max min " << max << " " << min << std::endl;
			assert(center.x <= size_image_max);
			assert(center.y <= size_image_max);
			
// 			std::cout << "getting the angle" << std::endl;
			double angle = corners_end[i].getOrientations()[0];
			double width = corners_end[i].getAngleWidths()[0];
			double plus_a = angle + (width/2);
			double moins_a = angle - (width/2);
			
			
// 			std::cout << "angle " << angle<< std::endl;
			cv::Point2i p2;
			p2.x = center.x + (50 * std::cos(angle) );
			p2.y = center.y + (50 * std::sin(angle) );
			cv::Point2i p2_p;
			p2_p.x = center.x + (50 * std::cos(plus_a) );
			p2_p.y = center.y + (50 * std::sin(plus_a) );
			cv::Point2i p2_m;
			p2_m.x = center.x + (50 * std::cos(moins_a) );
			p2_m.y = center.y + (50 * std::sin(moins_a) );
				
// 			std::cout << "Line " << center << " "<< p2 << std::endl;;
			
// 			std::cout << "Angle" << angle.
			
// 			cv::Mat copyy;
// 			ndt_img.copyTo(copyy);
// 			cv::circle(copyy, center, 20, cv::Scalar(255, 255, 255), 5);
// 			cv::line(copyy, center, p2, cv::Scalar(255, 255, 255), 1);	
// 			cv::line(copyy, center, p2_p, cv::Scalar(255, 255, 255), 1);	
// 			cv::line(copyy, center, p2_m, cv::Scalar(255, 255, 255), 1);			
// 
// 			cv::imshow("NDT_map", copyy);
// 			cv::waitKey(0);
		
			double cx, cy, cz;
			//Should it be in meters ?
			map_tmp->getGridSizeInMeters(cx, cy, cz);
			SIFTNdtLandmark(center, ndt_img, size_image_max, cx, ptr);


			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr);
			
			
		}
		else{
			std::cout << "Point seen " << std::endl;
			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr_landmark_seen);
		}
	}
			
	
// 	/***************** ADD THE CORNERS INTO THE GRAPH***********************/
// 	
// 	for(size_t i = 0 ; i < corners_end.size() ; ++i){
// // 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;	
// 		bool seen = false;
// 		AASS::acg::VertexLandmarkNDT* ptr_landmark_seen = NULL;
// 		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// // 			if(tmp[j] == _corners_position[i]){
// 			g2o::Vector2D landmark = _nodes_landmark[j]->estimate();
// 			cv::Point2f point_land(landmark(0), landmark(1));
// 			
// 			double res = cv::norm(point_land - corners_end[i].point);
// 			
// // 			std::cout << "res : " << res << " points "  << point_land << " " << corners_end[i].point << "  cell size " << cell_size << std::endl;
// 			
// 			//If we found the landmark, we save the data
// 			if( res < cell_size * 2){
// 				seen = true;
// 				ptr_landmark_seen = _nodes_landmark[j];
// 			}
// 		}
// 		if(seen == false){
// // 			std::cout << "New point " << i << " " <<  ret_opencv_point_corner.size() << std::endl;
// 			assert(i < ret_export.size());
// 			g2o::Vector2D vec;
// 			vec << corners_end[i].point.x, corners_end[i].point.y ;
// 			AASS::acg::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_export[i].getMeanOpenCV(), 1);
// 			ptr->addAngleOrientation(corners_end[i].getAngleWidth(), corners_end[i].getOrientation());
// 			ptr->first_seen_from = robot_ptr;
// 			//TODO IMPORTANT : Directly calculate SIft here so I don't have to do it again later
// 			
// 			std::cout << "Descriptors" << std::endl;
// 			/************************************************************/
// 			
// 			//Convert NDT MAP to image
// 			cv::Mat ndt_img;
// 			auto map_tmp = corners_end[i].getNodeLinkedPtr()->getMap();
// 			
// 			std::cout << "Descriptors" << std::endl;
// 			double size_image_max = 500;
// 			double min, max;
// 			//TODO: super convoluted, get rid of NDTCornerGraphElement!
// 			perception_oru::ndt_feature_finder::toCvMat(*map_tmp, ndt_img, size_image_max, max, min);
// 			
// 			//Use accurate CV point
// 			//Get old position_in_robot_frame
// // 			std::cout << "Position " << ret_opencv_point_corner[i] << std::endl;
// 			cv::Point2i center = perception_oru::ndt_feature_finder::scalePoint(ret_export[i].getMeanOpenCV(), max, min, size_image_max);
// // 			std::cout << "Position " << center << "max min " << max << " " << min << std::endl;
// 			assert(center.x <= size_image_max);
// 			assert(center.y <= size_image_max);
// 			
// // 			std::cout << "getting the angle" << std::endl;
// 			double angle = corners_end[i].getOrientation();
// 			double width = corners_end[i].getAngleWidth();
// 			double plus_a = angle + (width/2);
// 			double moins_a = angle - (width/2);
// 			
// 			
// // 			std::cout << "angle " << angle<< std::endl;
// 			cv::Point2i p2;
// 			p2.x = center.x + (50 * std::cos(angle) );
// 			p2.y = center.y + (50 * std::sin(angle) );
// 			cv::Point2i p2_p;
// 			p2_p.x = center.x + (50 * std::cos(plus_a) );
// 			p2_p.y = center.y + (50 * std::sin(plus_a) );
// 			cv::Point2i p2_m;
// 			p2_m.x = center.x + (50 * std::cos(moins_a) );
// 			p2_m.y = center.y + (50 * std::sin(moins_a) );
// 				
// // 			std::cout << "Line " << center << " "<< p2 << std::endl;;
// 			
// // 			std::cout << "Angle" << angle.
// 			
// // 			cv::Mat copyy;
// // 			ndt_img.copyTo(copyy);
// // 			cv::circle(copyy, center, 20, cv::Scalar(255, 255, 255), 5);
// // 			cv::line(copyy, center, p2, cv::Scalar(255, 255, 255), 1);	
// // 			cv::line(copyy, center, p2_p, cv::Scalar(255, 255, 255), 1);	
// // 			cv::line(copyy, center, p2_m, cv::Scalar(255, 255, 255), 1);			
// // 
// // 			cv::imshow("NDT_map", copyy);
// // 			cv::waitKey(0);
// 		
// 			double cx, cy, cz;
// 			//Should it be in meters ?
// 			map_tmp->getGridSizeInMeters(cx, cy, cz);
// 			SIFTNdtLandmark(center, ndt_img, size_image_max, cx, ptr);
// 		
// 			/****************************************************************/
// 
// 			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr);
// 			
// 			
// 		}
// 		else{
// 			std::cout << "Point seen " << std::endl;
// 			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr_landmark_seen);
// 		}
// 	}

}


void AASS::acg::AutoCompleteGraphFeature::matchLinks()
{
	cv::Mat desc_prior;
	cv::Mat desc_ndt;
	createDescriptorNDT(desc_ndt);
	createDescriptorPrior(desc_prior);
	
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match( desc_ndt, desc_prior, matches );

}

void AASS::acg::AutoCompleteGraphFeature::createDescriptorNDT(cv::Mat& desc){
	//Convert to OpenCV
	auto it = _nodes_ndt.begin();
	cv::Mat ndt_img;
	perception_oru::ndt_feature_finder::toCvMat(*((*it)->getMap().get()), ndt_img, 500);
	for(; it != _nodes_ndt.end() ; ++it){
		cv::Mat tmp;
		cv::Mat finl;
		double max, min;
		double size_image_max = 500;
		perception_oru::ndt_feature_finder::toCvMat(*((*it)->getMap().get()), tmp, size_image_max, max, min);
		std::cout << "MAX min " << max << " " << min << std::endl;
		AASS::acg::VertexSE2RobotPose* v_ptr_ndt = *it;
		
		for(auto it_edge = _edge_landmark.begin() ; it_edge != _edge_landmark.end() ; ++it_edge){
			AASS::acg::VertexLandmarkNDT* v_ptr = dynamic_cast<AASS::acg::VertexLandmarkNDT*> ((*it_edge)->vertices()[1]);
			std::cout << "Position : " << v_ptr->position << std::endl;
// 			cv::Point2f p;
			auto vertex = v_ptr->estimate();
			//Getting the translation out of the transform : https://en.wikipedia.org/wiki/Transformation_matrix
// 			p.x = vertex(0);
// 			p.y = vertex(1);
			
			//Transforming corner into NDT_map coordinate system
			//Corner pose
			Eigen::Vector3d vec;
			vec << vertex(0), vertex(1), 0;
			
			auto vec_out_se2 = v_ptr_ndt->estimate();

			//Uses just added modified node
			g2o::SE2 se2_tmp(vec);
			//Obstacle pose - ndt_node pose
			se2_tmp = se2_tmp * vec_out_se2.inverse();
			
			Eigen::Vector3d vec_out = se2_tmp.toVector();
			
			cv::Point2f p_out(vec_out(0), vec_out(1));
			
			std::cout << "Position " << p_out << std::endl;
			cv::Point2d center = perception_oru::ndt_feature_finder::scalePoint(p_out, max, min, size_image_max);
			std::cout << "Position " << center << "max min " << max << " " << min << std::endl;
			assert(center.x <= size_image_max);
			assert(center.y <= size_image_max);
			cv::circle(tmp, center, 20, cv::Scalar(255, 255, 255), 5);			
			
		}
		
		cv::imshow("NDT_map", tmp);
		cv::waitKey(0);
		
		
		
	}

}

void AASS::acg::AutoCompleteGraphFeature::createDescriptorPrior(cv::Mat& desc)
{
	cv::Mat m = cv::Mat::ones(4, 3, CV_64F);    // 3 cols, 4 rows
	cv::Mat row = cv::Mat::ones(1, 3, CV_64F);  // 3 cols
	m.push_back(row);                           // 3 cols, 5 rows
	
	auto it = _nodes_prior.begin();
	desc = (*it)->priorattr.descriptor;
	for( ; it != _nodes_prior.end() ; ++it){
		desc.push_back((*it)->priorattr.descriptor);
	}
	assert(desc.rows == _nodes_prior.size());

}


void AASS::acg::AutoCompleteGraphFeature::SIFTNdtLandmark(const cv::Point2f centre, const cv::Mat& img, double size_image_max, double cell_size, AASS::acg::VertexLandmarkNDT* vertex)
{

	cv::Mat img_tmpp;
	img.convertTo(img_tmpp, CV_8U);
	cv::Mat img_tmp;
	cv::cvtColor( img_tmpp, img_tmp, CV_BGR2GRAY );
// 	cv::imshow("NDT_map_C", img_tmp);
// 	cv::waitKey(0);
	std::string type = type2str(img_tmp.type());
	std::cout << ">Type " << type << std::endl;
	assert(type == "8UC1");
	//Creating a SIFT descriptor for eahc corner
	cv::KeyPoint keypoint;
	keypoint.pt = centre;
	
	double sizet = 2 * size_image_max / cell_size ;
	keypoint.size = sizet ;
	keypoint.angle = -1 ;
	keypoint.octave = 1 ;
	
	std::vector<cv::KeyPoint> keypoint_v;
	keypoint_v.push_back(keypoint);
	
	cv::Mat descriptors_1;
#if CV_MAJOR_VERSION == 2
	cv::SiftDescriptorExtractor extractor;
	extractor.compute( img_tmp, keypoint_v, descriptors_1);
#else
	cv::Ptr<cv::Feature2D> extractor = cv::xfeatures2d::SURF::create();
//    cv::xfeatures2d::SiftDescriptorExtractor extractor;
	extractor->compute( img_tmp, keypoint_v, descriptors_1);
#endif

	
	std::cout << descriptors_1.rows << " " << descriptors_1.cols << std::endl;
	assert(descriptors_1.rows == 1);
	
	vertex->descriptor = descriptors_1;
	vertex->keypoint = keypoint;
}


// void AASS::acg::AutoCompleteGraphFeature::optimize(int iter){
// 			
// 	_chi2s.clear();
// 	
// 	std::cout << "BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE" << std::endl;
// 	overCheckLinks();
// 	checkRobotPoseNotMoved("before");
// 	
// 	/********** HUBER kernel ***********/
// 	
// // 			_optimizable_graph.setHuberKernel();			
// 	
// 	_flag_optimize = checkAbleToOptimize();
// 	
// 	if(_flag_optimize == true){
// 		std::cout << "OPTIMIZE" << std::endl;
// 
// 		if(_flag_use_robust_kernel){
// 			setAgeingHuberKernel();
// 		}
// 		checkRobotPoseNotMoved("set age in huber kernel");
// 		testNoNanInPrior("set age in huber kernel");
// 		testInfoNonNul("set age in huber kernel");
// 		
// // 				updatePriorEdgeCovariance();
// 		testNoNanInPrior("update prior edge cov");
// 		
// 		//Avoid overshoot of the cov
// 		for(size_t i = 0 ; i < iter ; ++i){
// 			_optimizable_graph.optimize(1);
// 			checkRobotPoseNotMoved("optimized with huber");
// 			testNoNanInPrior("optimized with huber");
// 			testInfoNonNul("optimized with huber");
// 			//Update prior edge covariance
// // 					updatePriorEdgeCovariance();
// 			testNoNanInPrior("update prior edge cov after opti huber");
// 			saveErrorStep();
// 		}
// 		
// 		/********** DCS kernel ***********/
// 		if(_flag_use_robust_kernel){
// 			setAgeingDCSKernel();
// 		}
// 		testNoNanInPrior("set age in DCS kernel");
// 		
// 		for(size_t i = 0 ; i < iter*2 ; ++i){
// 			_optimizable_graph.optimize(1);
// 			checkRobotPoseNotMoved("optimized with dcs");
// 			testNoNanInPrior("optimized with dcs");
// 			testInfoNonNul("optimized with dcs");
// 			//Update prior edge covariance
// // 					updatePriorEdgeCovariance();
// 			testNoNanInPrior("update prior edge cov after opti dcs");
// 			saveErrorStep();
// 		}
// 	
// 
// 	}
// 	else{
// 		std::cout << "No Optimization :(" << std::endl;
// 	}
// 	
// 	std::cout << "AFTER THE OPTIMIZATION CREATE" << std::endl;
// // 			int count = countLinkToMake();
// // // 			int count2 = createNewLinks();
// // 			if(count != count2){
// // 				std::cout << "Weird different detection" << std::endl;
// // 				throw std::runtime_error("ARF NOT GOOD COUNT");
// // 			}
// // 			overCheckLinks();
// 	
// 	removeBadLinks();
// 	std::cout << "AFTER THE OPTIMIZATION REMOVE" << std::endl;
// 	overCheckLinks();
// 	
// 	exportChi2s();
// 	checkRobotPoseNotMoved("after");
// 	
// // 			cv::Mat d;
// // 			createDescriptorNDT(d);
// 	
// }


// int AASS::acg::AutoCompleteGraphFeature::createNewLinks()
// {
// 	int count = 0 ;
// // 			std::vector < std::pair < AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*> > links;
// 	
// 	std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
// 	std::cout << "Prior " << _nodes_prior.size() << std::endl;
// // 	if(_nodes_prior.size() > 0){
// 	
// // 			int a;
// // 			std::cin >> a;
// 	
// 	//Update ALL links
// 	auto it = _nodes_landmark.begin();
// 	for(it ; it != _nodes_landmark.end() ; it++){
// // 		std::cout << "Working on links " << std::endl;
// 		Eigen::Vector2d pose_landmark = (*it)->estimate();
// 		auto it_prior = _nodes_prior.begin();
// 		for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){
// 			
// 			Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
// 			Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
// 			double norm_tmp = (pose_prior - pose_landmark).norm();
// 			
// 			
// 			
// 			//Don't add the same link twice
// 			if(linkAlreadyExist(*it, *it_prior) == false){
// 				
// // 				std::cout << "new" << std::endl;
// 				
// 				//Update the link
// 				if(norm_tmp <= _min_distance_for_link_in_meter){
// 					std::cout << "NORM " << norm_tmp << "min dist " << _min_distance_for_link_in_meter << std::endl;
// // 					ptr_closest = *it_prior;
// // 					norm = norm_tmp;
// 					//Pushing the link
// // 					std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
// 					
// 					if( sameOrientation(**it, **it_prior) ){
// // 								links.push_back(std::pair<AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*>(*it, *it_prior));
// 						++count;
// 						g2o::Vector2D vec;
// 						vec << 0, 0;
// 						addLinkBetweenMaps(vec, *it_prior, *it);
// 					}
// 					
// 				}	
// 			}
// 			else{
// 				
// 				std::cout << "Already exist" << std::endl;
// 			}
// 		}
// 	}
// 
// return count;
// 	
// }
