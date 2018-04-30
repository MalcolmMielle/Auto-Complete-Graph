#ifndef AUTOCOMPLETEGRAPH_MATCH_29032019
#define AUTOCOMPLETEGRAPH_MATCH_29032019

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
		g2o::VertexLandmarkNDT* _node_landmark;

	public:
		Match(const cv::Point2f& p, cv::Point2f& r): _prior(p), _landmark(r){}

		cv::Point2f getPriorPoint(){return _prior;}
		cv::Point2f getLandmarkPoint(){return _landmark;}
		g2o::VertexSE2Prior* getPriorNode(){return _node_prior;}
		g2o::VertexLandmarkNDT* getLandmarkNode(){return _node_landmark;}

		void getNodes(const AutoCompleteGraph& acg){

			Eigen::Vector2d pose2d_p; pose2d_p << _prior.x, _prior.y;
			auto prior_nodes = acg.getPrior()->getNodes();

			double norm_p = -1;

			for(auto prior_node_it_const = prior_nodes.begin();prior_node_it_const != prior_nodes.end(); prior_node_it_const++){

				Eigen::Vector3d pose = (*prior_node_it_const)->estimate().toVector();
				Eigen::Vector2d pose2d; pose2d << pose(0), pose(1);
				double norm = (pose2d - pose2d_p).norm();

				if(norm_p == -1 || norm <= norm_p){
					norm_p = norm;
					_node_prior = *prior_node_it_const;
				}
			}

			std::vector<g2o::VertexLandmarkNDT*> landmark_nodes = acg.getLandmarkNodes();

			double norm_l = -1;
			pose2d_p; pose2d_p << _landmark.x, _landmark.y;

			for(std::vector<g2o::VertexLandmarkNDT*>::const_iterator landmark_node_it_const = landmark_nodes.begin();landmark_node_it_const != landmark_nodes.end(); landmark_node_it_const++){

				Eigen::Vector2d pose2d = (*landmark_node_it_const)->estimate();
				double norm = (pose2d - pose2d_p).norm();

				if(norm_l == -1 || norm <= norm_l){
					norm_l = norm;
					_node_landmark = *landmark_node_it_const;
				}
			}

		}

	};
}

}

#endif