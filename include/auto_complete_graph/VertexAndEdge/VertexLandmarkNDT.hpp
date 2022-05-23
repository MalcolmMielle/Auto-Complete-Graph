#ifndef AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017
#define AUTOCOMPLETEGRAPH_VERTEXLANDMARKNDT_10112017

#include <auto_complete_graph/Localization/Localization.hpp>
#include "ndt_map/ndt_cell.h"
#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "VertexSE2RobotPose.hpp"
#include "opencv2/opencv.hpp"
#include "VertexSE2Prior.hpp"
#include "VertexSE2RobotLocalization.hpp"
#include "auto_complete_graph/utils.hpp"
#include "VertexPointXYACG.hpp"
#include "registration.hpp"
#include "EdgeLandmark.hpp"

#include "LocalizationPointer.hpp"
namespace g2o
{

	class VertexLandmarkNDT : public g2o::VertexPointXYACG
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer>> associated_localization;

		std::vector<boost::shared_ptr<perception_oru::NDTCell>> cells_that_gave_it_1;
		std::vector<boost::shared_ptr<perception_oru::NDTCell>> cells_that_gave_it_2;
		g2o::SE2 robotpose_seen_from;

		AASS::acg::EdgeInterfaceMalcolm interface;

		cv::Point2f position;
		cv::KeyPoint keypoint;
		cv::Mat descriptor;
		std::vector<std::pair<double, double>> angle_orientation;
		g2o::VertexSE2RobotPose *first_seen_from;

		VertexLandmarkNDT() : first_seen_from(NULL), g2o::VertexPointXYACG(){};

		void addLocalization(VertexSE2RobotLocalization *vertex, g2o::EdgeLandmark_malcolm *edge_observation_from_mcl_pose, const Eigen::Vector3d &mean, const Eigen::Matrix3d &cov, const Eigen::Vector2d &observation, const Eigen::Vector2d &original_pose_in_robot_frame, int index)
		{

			std::shared_ptr<AASS::acg::LocalizationPointer> lp(new AASS::acg::LocalizationPointer());
			lp->vertex_mcl_pose = vertex;
			lp->edge_observation_from_mcl_pose = edge_observation_from_mcl_pose;
			lp->mean = mean;
			lp->cov = cov;
			lp->index = index;
			lp->observation = observation;
			lp->original_pose_in_robot_frame = original_pose_in_robot_frame;
			cov.computeInverseAndDetWithCheck(lp->cov_inverse, lp->determinant, lp->inverse_cov_exist);
			if (!lp->inverse_cov_exist)
			{
				std::cout << "Covariance " << cov << std::endl;
				std::cout << "determinant " << lp->determinant << std::endl;
				//				throw std::runtime_error("Inverse covariance not findable");
				std::cout << "Inverse covariance not findable switching ot a small covariance instead" << std::endl;
				Eigen::Matrix3d temp_cov;
				temp_cov << 0.1, 0, 0,
					0, 0.1, 0,
					0, 0, 0.1;
				lp->cov = temp_cov;
				cov.computeInverseAndDetWithCheck(lp->cov_inverse, lp->determinant, lp->inverse_cov_exist);
				std::cout << "Did we find the inverse covariance now : " << lp->cov_inverse << std::endl;
				assert(lp->inverse_cov_exist == true);
			}
			associated_localization.insert(lp);
		}

		const Eigen::Matrix3d &getCovarianceObservation(const VertexSE2RobotLocalization *vertex) const
		{
			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
									   [vertex](const std::shared_ptr<AASS::acg::LocalizationPointer> &a) -> bool
									   { return a->vertex_mcl_pose == vertex; });
			return (*result)->cov;
		}

		const Eigen::Matrix3d &getInverseCovarianceObservation(const VertexSE2RobotLocalization *vertex) const
		{
			auto result = std::find_if(associated_localization.begin(), associated_localization.end(),
									   [vertex](const std::shared_ptr<AASS::acg::LocalizationPointer> &a) -> bool
									   { return a->vertex_mcl_pose == vertex; });
			return (*result)->cov_inverse;
		}

		const std::unordered_set<std::shared_ptr<AASS::acg::LocalizationPointer>> &getLocalization() const { return associated_localization; }

		const std::vector<std::pair<double, double>> &getAnglesAndOrientations() const { return angle_orientation; }
		double getAngleWidth(int i) const { return angle_orientation[i].first; }
		double getOrientation(int i) const { return angle_orientation[i].second; }
		// 		double getOrientation() const {return angle_orientation.second;}
		void addAngleOrientation(double a, double d)
		{
			if (a < 0)
				a += 2 * M_PI;
			if (d < 0)
				d += 2 * M_PI;
			angle_orientation.push_back(std::pair<double, double>(a, d));
		};
		void addAnglesOrientations(const std::vector<double> &angles, const std::vector<double> &orientations)
		{
			assert(angles.size() == orientations.size());
			for (int i = 0; i < angles.size(); ++i)
			{
				angle_orientation.push_back(std::pair<double, double>(angles[i], orientations[i]));
			}
		}
		double getOrientationGlobal(int i) const
		{
			assert(first_seen_from != NULL);
			auto vec = first_seen_from->estimate().toVector();
			// 			std::cout << "Got estimate " << std::endl;
			double angle = vec(2) + angle_orientation[i].second;
			if (angle >= 2 * M_PI)
			{
				angle = angle - 2 * M_PI;
			}
			else if (angle < 0)
			{
				angle = angle + 2 * M_PI;
			}
			return angle;
		}

		bool sameOrientation(std::vector<std::pair<double, double>> angles_orientations_v) const
		{

			for (auto it_ao_v = angles_orientations_v.begin(); it_ao_v != angles_orientations_v.end(); ++it_ao_v)
			{
				double angle_width_v = it_ao_v->first;
				double orientation_v = it_ao_v->second;
				assert(angle_width_v >= 0.08);
				for (int i_ao = 0; i_ao < getAnglesAndOrientations().size(); ++i_ao)
				{
					double landmark_angle = getAngleWidth(i_ao);
					double landmark_orientation = getOrientationGlobal(i_ao);
					assert(landmark_orientation >= 0);
					assert(orientation_v >= 0);
					assert(landmark_orientation < 2 * M_PI);
					assert(orientation_v < 2 * M_PI);

					//Smallest angle difference
					double orientation_diff = orientation_v - landmark_orientation;
					orientation_diff = fmod((orientation_diff + M_PI), (2 * M_PI)) - M_PI;

					double angle_diff = std::abs(angle_width_v - landmark_angle);

					//They are close enough in orientation
					if (orientation_diff <= M_PI / 4)
					{
						if (angle_diff <= M_PI / 4)
						{
							return true;
						}
					}
				}
			}
			return false;
		}
	};
}
#endif
