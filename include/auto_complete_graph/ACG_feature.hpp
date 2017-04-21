#ifndef AUTOCOMPLETEGRAPH_ACGFEATURE_14042017
#define AUTOCOMPLETEGRAPH_ACGFEATURE_14042017

#include "ACG.hpp"


namespace AASS {

namespace acg{	
	
	class AutoCompleteGraphFeature : public AutoCompleteGraph
	{
		public:
		AutoCompleteGraphFeature(const g2o::SE2& sensoffset, const std::string& load_file) : AutoCompleteGraph(sensoffset, load_file){};
		
		AutoCompleteGraphFeature(const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn){};
		
		AutoCompleteGraphFeature(const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn, ndt_feature::NDTFeatureGraph* ndt_graph) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn, ndt_graph){};
		
		
		virtual void updateLinks()
		{
			std::cout << "Create links in Features CAG" << std::endl;
// 			int a;
// 			std::cin >> a;
			std::cout << "Create new links" << std::endl;
// 			int count = countLinkToMake();
			int count2 = createNewLinks();
// 			if(count != count2){
// 				std::cout << "Weird different detection" << std::endl;
// 				throw std::runtime_error("ARF NOT GOOD COUNT in update link");
// 			}
			std::cout << "updateLink no small forgotten" << std::endl;
// 			checkLinkNotForgotten();
			removeBadLinks();
			std::cout << "update link not too big" << std::endl;
			checkLinkNotTooBig();
		}

		///@brief Create new link based on the distance AND the angle of the corners
		virtual int createNewLinks()
		{
			int count = 0 ;
// 			std::vector < std::pair < AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*> > links;
			
			std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
			std::cout << "Prior " << _nodes_prior.size() << std::endl;
		// 	if(_nodes_prior.size() > 0){
			
// 			int a;
// 			std::cin >> a;
			
			//Update ALL links
			auto it = _nodes_landmark.begin();
			for(it ; it != _nodes_landmark.end() ; it++){
		// 		std::cout << "Working on links " << std::endl;
				Eigen::Vector2d pose_landmark = (*it)->estimate();
				auto it_prior = _nodes_prior.begin();
				for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){
					
					Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
					Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
					double norm_tmp = (pose_prior - pose_landmark).norm();
					
					
					
					//Don't add the same link twice
					if(linkAlreadyExist(*it, *it_prior) == false){
						
		// 				std::cout << "new" << std::endl;
						
						//Update the link
						if(norm_tmp <= _min_distance_for_link_in_meter){
							std::cout << "NORM " << norm_tmp << "min dist " << _min_distance_for_link_in_meter << std::endl;
		// 					ptr_closest = *it_prior;
		// 					norm = norm_tmp;
							//Pushing the link
		// 					std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
							
							if( sameOrientation(**it, **it_prior) ){
// 								links.push_back(std::pair<AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*>(*it, *it_prior));
								++count;
								g2o::Vector2D vec;
								vec << 0, 0;
								addLinkBetweenMaps(vec, *it_prior, *it);
							}
							
						}	
					}
					else{
						
						std::cout << "Already exist" << std::endl;
					}
				}
			}

		return count;
			
		}
		
		
		bool sameOrientation(const AASS::acg::VertexLandmarkNDT& landmark, const AASS::acg::VertexSE2Prior& prior_corner){
			double landmark_angle = landmark.getAngleWidth();
			double landmark_direction = landmark.getDirectionGlobal();
			
			std::cout << "Orientation calc" << std::endl;
			auto angledirection = prior_corner.getAngleDirection();
			for(auto it = angledirection.begin() ; it != angledirection.end() ; ++it){
				
				double angle_width = it->first;
				double direction = it->second;
				assert(angle_width >= 0.08);
				std::cout << " Angles " << angle_width << " " << direction << " and " << landmark_angle << " " << landmark_direction << std::endl;
				std::cout <<  direction << " <= " << landmark_direction + 0.785 << " && " << direction << " >= " << landmark_direction - 0.785 << " && " <<	angle_width << " <= " << landmark_angle + 0.785 << " && " << angle_width << " >= " << landmark_angle - 0.349 << std::endl;
				
				double landmark_direction_over = landmark_direction + 0.785;
				if (landmark_direction_over < 0) landmark_direction_over += 2 * M_PI;
				double landmark_direction_under = landmark_direction - 0.785;
				if (landmark_direction_under < 0) landmark_direction_under += 2 * M_PI;
				double landmark_angle_under = landmark_angle - 0.349;
				if (landmark_angle_under < 0) landmark_angle_under += 2 * M_PI;
				double landmark_angle_over = landmark_angle + 0.785;
				if (landmark_angle_over >= 2 * M_PI) landmark_angle_over -= 2 * M_PI;
				
				if( direction <= landmark_direction_over && direction >= landmark_direction_under &&
					angle_width <= landmark_angle_over && angle_width >= landmark_angle_under
				){
// 				if( direction <= landmark_direction + 0.785 && direction >= landmark_direction - 0.785
// 				){
					std::cout << "Good angle" << std::endl;
// 					int a;
// 					std::cin>>a;
					return true;
				}
			}
// 			std::cout << "Not good " << std::endl;
// 			int a;
// 			std::cin>>a;
			return false;
		}
		
		virtual /**
		 * @brief actual optimization loop. Make sure setFirst and prepare are called before. Use Huber first and then DCS.
		 * 
		 */
		void optimize(int iter = 10){
			
			_chi2s.clear();
			
			std::cout << "BEFORE THE OPTIMIZATION BUT AFTER ADDING A NODE" << std::endl;
			overCheckLinks();
			checkRobotPoseNotMoved("before");
			
			/********** HUBER kernel ***********/
			
// 			_optimizable_graph.setHuberKernel();			
			
			_flag_optimize = checkAbleToOptimize();
			
			if(_flag_optimize == true){
				std::cout << "OPTIMIZE" << std::endl;

				if(_flag_use_robust_kernel){
					setAgeingHuberKernel();
				}
				checkRobotPoseNotMoved("set age in huber kernel");
				testNoNanInPrior("set age in huber kernel");
				testInfoNonNul("set age in huber kernel");
				
// 				updatePriorEdgeCovariance();
				testNoNanInPrior("update prior edge cov");
				
				//Avoid overshoot of the cov
				for(size_t i = 0 ; i < iter ; ++i){
					_optimizable_graph.optimize(1);
					checkRobotPoseNotMoved("optimized with huber");
					testNoNanInPrior("optimized with huber");
					testInfoNonNul("optimized with huber");
					//Update prior edge covariance
// 					updatePriorEdgeCovariance();
					testNoNanInPrior("update prior edge cov after opti huber");
					saveErrorStep();
				}
				
				/********** DCS kernel ***********/
				if(_flag_use_robust_kernel){
					setAgeingDCSKernel();
				}
				testNoNanInPrior("set age in DCS kernel");
				
				for(size_t i = 0 ; i < iter*2 ; ++i){
					_optimizable_graph.optimize(1);
					checkRobotPoseNotMoved("optimized with dcs");
					testNoNanInPrior("optimized with dcs");
					testInfoNonNul("optimized with dcs");
					//Update prior edge covariance
// 					updatePriorEdgeCovariance();
					testNoNanInPrior("update prior edge cov after opti dcs");
					saveErrorStep();
				}
			

			}
			else{
				std::cout << "No Optimization :(" << std::endl;
			}
			
			std::cout << "AFTER THE OPTIMIZATION CREATE" << std::endl;
// 			int count = countLinkToMake();
// // 			int count2 = createNewLinks();
// 			if(count != count2){
// 				std::cout << "Weird different detection" << std::endl;
// 				throw std::runtime_error("ARF NOT GOOD COUNT");
// 			}
// 			overCheckLinks();
			
			removeBadLinks();
			std::cout << "AFTER THE OPTIMIZATION REMOVE" << std::endl;
			overCheckLinks();
			
			exportChi2s();
			checkRobotPoseNotMoved("after");
			
// 			cv::Mat d;
// 			createDescriptorNDT(d);
			
		}
		
		virtual void overCheckLinks(){
			//Because not all link will be created
// 			checkLinkNotForgotten();
			checkLinkNotTooBig();
		}
		
		///@brief return true if ACG got more than or equal to 5 links. Make this better.
		virtual bool checkAbleToOptimize(){
			std::cout << "edge link size: " << _edge_link.size() << std::endl;
			if(_edge_link.size() >= 6){
				return true;
			}
			return false;
		}
		

	};
	
}

}

#endif
