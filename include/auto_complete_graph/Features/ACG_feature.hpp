#ifndef AUTOCOMPLETEGRAPH_ACGFEATURE_14042017
#define AUTOCOMPLETEGRAPH_ACGFEATURE_14042017

#include "ACG.hpp"

namespace AASS
{

	namespace acg
	{

		class AutoCompleteGraphFeature : public AutoCompleteGraph
		{
		public:
			AutoCompleteGraphFeature(const g2o::SE2 &sensoffset, const std::string &load_file) : AutoCompleteGraph(sensoffset, load_file){};

			AutoCompleteGraphFeature(const g2o::SE2 &sensoffset, const Eigen::Vector2d &tn, double rn, const Eigen::Vector2d &ln, const Eigen::Vector2d &pn, double rp, const Eigen::Vector2d &linkn) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn){};

			AutoCompleteGraphFeature(const g2o::SE2 &sensoffset, const Eigen::Vector2d &tn, double rn, const Eigen::Vector2d &ln, const Eigen::Vector2d &pn, double rp, const Eigen::Vector2d &linkn, ndt_feature::NDTFeatureGraph *ndt_graph) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn, ndt_graph){};

			virtual void updateLinks()
			{
				int count2 = createNewLinks();
				removeBadLinks();
				checkLinkNotTooBig();
			}

			virtual void overCheckLinks()
			{
				checkLinkNotTooBig();
			}

			///@brief return true if ACG got more than or equal to 5 links. Make this better.
			virtual bool checkAbleToOptimize()
			{
				std::cout << "edge link size: " << _edge_link.size() << std::endl;
				if (_edge_link.size() >= 6)
				{
					return true;
				}
				return false;
			}

			void SIFTNdtLandmark(const cv::Point2f centre, const cv::Mat &img, double size_image_max, double cell_size, AASS::acg::VertexLandmarkNDT *vertex);

			void createDescriptorPrior(cv::Mat &desc);

			//UNUSUED
			void createDescriptorNDT(cv::Mat &desc);

			void matchLinks();

			void extractCornerNDTMap(const std::shared_ptr<perception_oru::NDTMap> &map, AASS::acg::VertexSE2RobotPose *robot_ptr, const g2o::SE2 &robot_pos);
		};

	}

}

#endif
