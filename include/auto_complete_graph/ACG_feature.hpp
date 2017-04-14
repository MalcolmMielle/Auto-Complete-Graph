#ifndef AUTOCOMPLETEGRAPH_ACGFEATURE_14042017
#define AUTOCOMPLETEGRAPH_ACGFEATURE_14042017

#include "ACG.hpp"


namespace AASS {

namespace acg{	
	
	 class AutoCompleteGraph_Feature : public AutoCompleteGraph
	{
		public:
		AutoCompleteGraph_Feature(const g2o::SE2& sensoffset, const std::__cxx11::string& load_file);
		
		AutoCompleteGraph_Feature(const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn);
		
		AutoCompleteGraph_Feature(const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn, ndt_feature::NDTFeatureGraph* ndt_graph);
		
		
		void updateLinks()
		{
			std::cout << "Create new links" << std::endl;
			int count = countLinkToMake();
			int count2 = createNewLinks();
			if(count != count2){
				std::cout << "Weird different detection" << std::endl;
				throw std::runtime_error("ARF NOT GOOD COUNT in update link");
			}
			std::cout << "updateLink no small forgotten" << std::endl;
			checkLinkNotForgotten();
			removeBadLinks();
			std::cout << "update link not too big" << std::endl;
			checkLinkNotTooBig();
		}

		///@brief Create new link based on the distance AND the angle of the corners
		int createNewLinks()
		{
			int count = 0 ;
// 			std::vector < std::pair < AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*> > links;
			
			std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
			std::cout << "Prior " << _nodes_prior.size() << std::endl;
		// 	if(_nodes_prior.size() > 0){
				
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
								g2o::Vector2D vec;
								vec << 0, 0;
								addLinkBetweenMaps(vec, *it_prior, *it);
							}
							++count;
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
			double landmark_angle = landmark.getAngleGlobal();
			for(auto it = prior_corner.getAngleDirection().begin() ; it != prior_corner.getAngleDirection().end() ; ++it){
				double angle_width = it->first;
				double direction = it->second;
			}
			
		}
		
		

	};
	
}

}

#endif