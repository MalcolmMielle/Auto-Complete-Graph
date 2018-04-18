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
		 * @brief register a point from either the prior or the robot map. Start with the prior
		 */
		virtual void clicked(const geometry_msgs::PointStamped::ConstPtr& msg){
			std::cout << "Clicked !!! " << _flag_go << std::endl;
			cv::Point2f point(msg->point.x, msg->point.y);
			
			if(_flag_go == false){
				_tmp_point = point;
			}
			else{
				std::cout << "add point" << std::endl;
				Match match(_tmp_point, point);
				match.getNodes(*_acg);
				
				if(checkMatchExist(match) == false)
				{
					_points.push_back(match);
				}
			}
			
			if(_flag_go == true){
				_flag_go = false;
			}
			else{
				_flag_go = true;
			}
			
			publishAll();

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
					g2o::VertexPointXYACG* ptr2 = dynamic_cast<g2o::VertexPointXYACG*>((*it_acg)->vertices()[1]);
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
		
		void publishAll(){
			
			std::cout << "Publishing " << _points.size() << std::endl;
			
			_link_markers.points.clear();
			_link_markers.header.stamp = ros::Time::now();
			
			for(auto it = _points.begin(); it != _points.end(); ++it){
				geometry_msgs::Point p;
				
				auto vertex = it->getPriorNode()->estimate().toVector();
				p.x = vertex(0);
				p.y = vertex(1);
				p.z = 0;
				
				std::cout << "first point " << p.x << " " << p.y << std::endl;
				
				_link_markers.points.push_back(p);
				
				auto vertex2 = it->getLandmarkNode()->estimate();
				p.x = vertex2(0);
				p.y = vertex2(1);
				p.z = 0;
				
				std::cout << "second point " << p.x << " " << p.y << std::endl;
				
				_link_markers.points.push_back(p);
				
			}
			
			_link_pub.publish(_link_markers);
			
		}
		
		private:
		bool checkMatchExist(Match& match){
		
			for(auto it = _points.begin(); it != _points.end(); ++it){
				if(match.getPriorNode() == it->getPriorNode() && match.getLandmarkNode() == it->getLandmarkNode() ){
					return true;
				}
			}
			
			return false;
			
		}
		
		
	};
}

}

#endif