#ifndef AUTOCOMPLETEGRAPH_GOODMATCHINGS_03022017
#define AUTOCOMPLETEGRAPH_GOODMATCHINGS_03022017

#include "opencv2/opencv.hpp"

#include "RvizPoints.hpp"

namespace AASS {

namespace acg{	
	
	
	/*
	 * Get Points from Rviz. THose point are used to marked good links and the class can use them to calculate a number of outliers
	 */
	class GoodMatchings : public RvizPoints {
		
	public:
		GoodMatchings(ros::NodeHandle nh, AutoCompleteGraph* acg) : RvizPoints(nh, acg){			
		}
		
		/**
		 * @brief return the number of outliers link-edges.
		 */
		void getOutliers(){
			int good = 0;
			int all = _acg->getLinkEdges().size();
			for(auto it = _points.begin(); it != _points.end(); ++it){
				auto links = _acg->getLinkEdges();
				for(auto it_acg = links.begin(); it_acg != links.end(); ++it_acg){
					
					g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>((*it_acg)->vertices()[0]);
					if(ptr == NULL){
						std::cout << ptr << " and " << (*it_acg)->vertices()[0] << std::endl;
						throw std::runtime_error("Links do not have the good vertex type. Prior lae");
					}
					g2o::VertexPointXY* ptr2 = dynamic_cast<g2o::VertexPointXY*>((*it_acg)->vertices()[1]);
					if(ptr2 == NULL){
						throw std::runtime_error("Links do not have the good vertex type. Landmark");
					}
					
					if(it->getLandmarkNode() == ptr2 && it->getPriorNode() == ptr){
						++good;
					}
					
				}
			}
			
			std::cout << "/******************************************************************/" << std::endl;
			std::cout << "outliers : " << all - good << " with " << good << " good links" << std::endl;
			std::cout << "/******************************************************************/" << std::endl;
			
		}
		
	};
}

}

#endif