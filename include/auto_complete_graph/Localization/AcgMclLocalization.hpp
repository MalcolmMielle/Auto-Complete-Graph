#ifndef AUTOCOMPLETEGRAPH_ACGMCLLOCALIZATION_26072018
#define AUTOCOMPLETEGRAPH_ACGMCLLOCALIZATION_26072018

#include <pcl/common/transforms.h>
#include <ndt_localization/particle_filter.hpp>

#include "auto_complete_graph/Localization/Localization.hpp"
#include "auto_complete_graph/GraphMapLocalizationMsg.h"
#include "auto_complete_graph/Localization/LocalizationConvertion.hpp"

#include "g2o/types/slam2d/se2.h"
#include "auto_complete_graph/conversion.hpp"

namespace AASS {

namespace acg{

	/**
	 * A class doing the localization published in graph_map_publisher_localization.cpp
	 */
	class ACGMCLLocalization : public perception_oru::particle_filter{

	protected:
		
// 		boost::shared_ptr<perception_oru::particle_filter> _ndtmcl;
		bool mcl_loaded_;

		Eigen::Affine3d _sensor_pose;
		
		std::vector<Localization> _localization;


	public:
		// map publishing function
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW


//		ACGMCLLocalization(std::string mapFile_, int particleCount_) : mcl_loaded_(false), perception_oru::particle_filter(mapFile_, particleCount_){};
		
		ACGMCLLocalization(perception_oru::NDTMap *ndtMap_, int particleCount_/*, init_type initializationType_*/, bool be2D_=true, bool forceSIR_=true,double varLimit_=0, int sirCount_=0) : mcl_loaded_(false), perception_oru::particle_filter(ndtMap_, particleCount_, be2D_, forceSIR_, varLimit_, sirCount_){
			std::cout << "FORCE SIR << forceSir_--------------->< " << forceSIR_ << std::endl;
		}
		
		void init(double xx, double yy, double yaw, double initVar, double cov_x_mcl, double cov_y_mcl, double cov_yaw_mcl, double scale_gaussian_mcl, double numPart, bool forceSIR){

//			std::cout << "INIT MCL ACG" << std::endl;
			
// 			perception_oru::particle_filter* pMCL = new perception_oru::particle_filter(map, numPart, true, forceSIR);
// 			_ndtmcl = boost::shared_ptr<perception_oru::particle_filter>(pMCL);
			
			setMotionModelCovX(cov_x_mcl);
			setMotionModelCovY(cov_y_mcl);
			setMotionModelCovYaw(cov_yaw_mcl);
			setScalingFactorGaussian(scale_gaussian_mcl);
//
//	 		std::cout << "Init at " << xx << " " << yy << " " << yaw << std::endl;
			InitializeNormalConstantAngle(xx, yy, yaw, initVar);
//			std::cout << "Normal init" << std::endl;
			mcl_loaded_ = true;
//			std::cout << "MAP LOADED SUCCESSFULLY :)" << std::endl;
			
		}
		
		
		///Laser scan to PointCloud expressed in the base frame
		std::tuple<Eigen::Vector3d, Eigen::Matrix3d> localization( const Eigen::Affine3d& Tmotion, const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Affine3d& sensorpose, double fraction, double cutoff){
			
			if(mcl_loaded_ = true){
				
				UpdateAndPredictEff(Tmotion, cloud, sensorpose, fraction, cutoff);

				Eigen::Matrix3d cov;
				Eigen::Vector3d mean;

				GetPoseMeanAndVariance2D(mean, cov);
				
				return std::make_tuple(mean, cov);
			// 	
			}
			else{
				std::cout << "You need to init MCL to start MCL localization" << std::endl;
				Eigen::Matrix3d cov;
				Eigen::Vector3d mean;
				return std::make_tuple(mean, cov);
			}
		}

		void setSensorPose(const Eigen::Affine3d& sen){_sensor_pose = sen;}
		
		
		void savePos(int index = -1){
			Eigen::Matrix3d cov;
			Eigen::Vector3d mean;
			GetPoseMeanAndVariance2D(mean, cov);

			Eigen::Affine3d mean_tmp = Eigen::Translation<double, 3>(mean(0), mean(1), 0) *
                                   Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxis<double>(mean(2), Eigen::Vector3d::UnitZ());

			mean_tmp = mean_tmp * _sensor_pose;
			Eigen::Isometry2d iso = Affine3d2Isometry2d(mean_tmp);
			g2o::SE2 se(iso);
			Eigen::Vector3d mean_sensor = se.toVector();

			Localization loc;
			loc.mean = mean_sensor;
			loc.cov = cov;
			loc.index = index;
			_localization.push_back(loc);

//			_mean_saved.push_back(mean);
//			_cov_saved.push_back(cov);
//			_indexes.push_back(index);
			
		}

		void saveCov(int index = -1){
			Eigen::Matrix3d cov;
			Eigen::Vector3d mean;
			GetPoseMeanAndVariance2D(mean, cov);

//			std::cout << "index " << index << " LOCALIZATION " << _localization[index].cov <<  " = " << cov << std::endl;

			_localization[index].cov = cov;

//			std::cout << "MAking sure " << _localization[index].cov << std::endl;
//			loc.mean = mean;
//			loc.cov = cov;
//			loc.index = index;
//			_localization.push_back(loc);
		}
		
//		const std::vector<Eigen::Vector3d>& getMeans() const {return _mean_saved;}
//		const std::vector<Eigen::Matrix3d>& getCovs() const {return _cov_saved;}

		const std::vector<Localization>& getLocalizations() const {return _localization;}

		
		void toMessage(auto_complete_graph::GraphMapLocalizationMsg& msg){

			for(auto const& localization : _localization) {
//				assert(localization.cov(0, 0) != -1);
				auto_complete_graph::LocalizationMsg loc_msg = AASS::acg::toMessage(localization);
				msg.localizations.push_back(loc_msg);
			}
			

		}

		/**
		 * Update the internal map and copy it
		 * @param ndtMap new map
		 */
		void setMap(const perception_oru::NDTMap& ndtMap){
			std::shared_ptr<perception_oru::NDTMap> ndtmapcopy (new perception_oru::NDTMap(ndtMap));
			perception_oru::particle_filter::setMap(ndtmapcopy);
		}
		
	};
	
	
	

}

}

#endif