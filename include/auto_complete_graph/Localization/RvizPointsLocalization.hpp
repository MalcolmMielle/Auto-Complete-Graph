#ifndef AUTOCOMPLETEGRAPH_RVIZPOINTLOCALIZATION_18042018
#define AUTOCOMPLETEGRAPH_RVIZPOINTLOCALIZATION_18042018

#include "opencv2/opencv.hpp"

#include "MatchLocalization.hpp"

namespace AASS {

	namespace acg{

		/**
		 * @brief base class that get points from Rviz, find the closest prior or then robot landmark and associate them together.
		 */
		class RvizPointsLocalization {

		protected:
			bool _flag_go;
			cv::Point2f _tmp_point;
			ros::NodeHandle _nh;
			ros::Subscriber _point_clicked;
			std::vector<MatchLocalization> _points;
			AutoCompleteGraphLocalization* _acg;
			ros::Publisher _link_pub;
			visualization_msgs::Marker _link_markers;
			bool which;

		public:
			RvizPointsLocalization(ros::NodeHandle nh, AutoCompleteGraphLocalization* acg) : _flag_go(false), _nh(nh), _acg(acg){
				_point_clicked = _nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, boost::bind(&RvizPointsLocalization::clicked, this, _1));

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

			void clear(){_points.clear(); _flag_go = false;}

			std::vector<AASS::acg::MatchLocalization>& getMatches(){return _points;}
			const std::vector<MatchLocalization>& getMatches() const {return _points;}
			std::size_t size() const {return _points.size();}

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
					MatchLocalization match(_tmp_point, point);
// 				match.getNodes(*_acg);

// 				if(checkMatchExist(match) == false)
// 				{
					_points.push_back(match);
// 				}
				}

				if(_flag_go == true){
					_flag_go = false;
				}
				else{
					_flag_go = true;
				}

				publishAll();

			}

			virtual void publishAll(){

				std::cout << "Publishing " << _points.size() << std::endl;

				_link_markers.points.clear();
				_link_markers.header.stamp = ros::Time::now();

				for(auto it = _points.begin(); it != _points.end(); ++it){
					geometry_msgs::Point p;

// 				auto vertex = it->getPriorNode()->estimate().toVector();
					auto vertex = it->getPriorPoint();
					p.x = vertex.x;
					p.y = vertex.y;
					p.z = 0;

					std::cout << "first point " << p.x << " " << p.y << std::endl;

					_link_markers.points.push_back(p);

// 				auto vertex2 = it->getLandmarkNode()->estimate();
					auto vertex2 = it->getLandmarkPoint();
					p.x = vertex2.x;
					p.y = vertex2.y;
					p.z = 0;

					std::cout << "second point " << p.x << " " << p.y << std::endl;

					_link_markers.points.push_back(p);

				}

				_link_pub.publish(_link_markers);

			}

		};
	}

}

#endif