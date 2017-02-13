#ifndef AUTOCOMPLETEGRAPH_GOODMATCHINGS_03022017
#define AUTOCOMPLETEGRAPH_GOODMATCHINGS_03022017

#include "opencv2/opencv.hpp"

#include "ACG.hpp"

namespace AASS {

namespace acg{	
	
	class Match{
	protected:
		cv::Point2f _prior;
		cv::Point2f _landmark;
		g2o::VertexSE2Prior* _node_prior;
		///@brief vector storing all node from the landarks 
		g2o::VertexPointXY* _node_landmark;
		
	public:
		Match(const cv::Point2f& p, cv::Point2f& r): _prior(p), _landmark(r){}
		
		cv::Point2f getPriorPoint(){return _prior;}
		cv::Point2f getRobotPoint(){return _landmark;}
		g2o::VertexSE2Prior* getPriorNode(){return _node_prior;}
		g2o::VertexPointXY* getLandmarkNode(){return _node_landmark;}
		
		void getNodes(const AutoCompleteGraph& acg){
			
			Eigen::Vector2d pose2d_p; pose2d_p << _prior.x, _prior.y;
			std::vector<g2o::VertexSE2Prior*> prior_nodes = acg.getPriorNodes();
			
			double norm_p = -1;
			
			for(std::vector<g2o::VertexSE2Prior*>::const_iterator prior_node_it_const = prior_nodes.begin();prior_node_it_const != prior_nodes.end(); prior_node_it_const++){
				
				Eigen::Vector3d pose = (*prior_node_it_const)->estimate().toVector();
				Eigen::Vector2d pose2d; pose2d << pose(0), pose(1);
				double norm = (pose2d - pose2d_p).norm();
				
				if(norm_p == -1 || norm <= norm_p){
					norm_p = norm;
					_node_prior = *prior_node_it_const;
				}				
			}
			
			std::vector<g2o::VertexPointXY*> landmark_nodes = acg.getLandmarkNodes();
			
			double norm_l = -1;
			pose2d_p; pose2d_p << _landmark.x, _landmark.y;
			
			for(std::vector<g2o::VertexPointXY*>::const_iterator landmark_node_it_const = landmark_nodes.begin();landmark_node_it_const != landmark_nodes.end(); landmark_node_it_const++){
				
				Eigen::Vector2d pose2d = (*landmark_node_it_const)->estimate();
				double norm = (pose2d - pose2d_p).norm();
				
				if(norm_l == -1 || norm <= norm_l){
					norm_l = norm;
					_node_landmark = *landmark_node_it_const;
				}				
			}
			
		}
		
	};
	
	/*
	 * Working directly on g2o graph
	 */
	class GoodMatchings {
		
	protected:
		bool _flag_go;
		cv::Point2f _tmp_point;
		ros::NodeHandle _nh;
		ros::Subscriber _point_clicked;
		std::vector<Match> _points;
		AutoCompleteGraph* _acg;
		ros::Publisher _link_pub;
		visualization_msgs::Marker _link_markers;
		bool which;
		
	public:
		GoodMatchings(ros::NodeHandle nh, AutoCompleteGraph* acg) : _flag_go(false), _nh(nh), _acg(acg){
			_point_clicked = _nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, boost::bind(&GoodMatchings::clicked, this, _1));
			
			_link_pub = _nh.advertise<visualization_msgs::Marker>("correct_link", 10);
			
			_link_markers.type = visualization_msgs::Marker::LINE_LIST;
			_link_markers.header.frame_id = "/world";
			_link_markers.ns = "acg";
			_link_markers.id = 3;
			_link_markers.scale.x = 0.2;
			_link_markers.scale.y = 0.2;
			_link_markers.color.g = 0.5f;
			_link_markers.color.r = 0.5f;
			_link_markers.color.a = 0.5;
			
		}
		
		void clicked(const geometry_msgs::PointStamped::ConstPtr& msg){
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
				
				std::cout << "first point " << p.x << "" << p.y << std::endl;
				
				_link_markers.points.push_back(p);
				
				auto vertex2 = it->getLandmarkNode()->estimate();
				p.x = vertex2(0);
				p.y = vertex2(1);
				p.z = 0;
				
				std::cout << "second point " << p.x << "" << p.y << std::endl;
				
				_link_markers.points.push_back(p);
				
			}
			
			_link_pub.publish(_link_markers);
			
		}
		
		
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