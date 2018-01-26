#ifndef AUTOCOMPLETEGRAPH_ACGMCLLOCALIZATION_26072018
#define AUTOCOMPLETEGRAPH_ACGMCLLOCALIZATION_26072018

#include <pcl/common/transforms.h>
#include <ndt_mcl/particle_filter.hpp>

namespace AASS {

namespace acg{
	
	class ACGMCLLocalization : public perception_oru::particle_filter{
	protected:
		
// 		boost::shared_ptr<perception_oru::particle_filter> _ndtmcl;
		bool mcl_loaded_;
		
	public:
		ACGMCLLocalization(std::string mapFile_, int particleCount_) : mcl_loaded_(false), perception_oru::particle_filter(mapFile_, particleCount_){};
		
		ACGMCLLocalization(perception_oru::NDTMap *ndtMap_, int particleCount_/*, init_type initializationType_*/, bool be2D_=true, bool forceSIR_=true,double varLimit_=0, int sirCount_=0) : mcl_loaded_(false), perception_oru::particle_filter(ndtMap_, particleCount_, be2D_, forceSIR_, varLimit_, sirCount_){}
		
		void init(perception_oru::NDTMap* map, double xx, double yy, double yaw, double initVar, double cov_x_mcl, double cov_y_mcl, double cov_yaw_mcl, double scale_gaussian_mcl, double numPart, bool forceSIR){
			
// 			perception_oru::particle_filter* pMCL = new perception_oru::particle_filter(map, numPart, true, forceSIR);
// 			_ndtmcl = boost::shared_ptr<perception_oru::particle_filter>(pMCL);
			
			setMotionModelCovX(cov_x_mcl);
			setMotionModelCovY(cov_y_mcl);
			setMotionModelCovYaw(cov_yaw_mcl);
			setScalingFactorGaussian(scale_gaussian_mcl);
			
	// 		std::cout << "Init at " << xx << " " << yy << " " << zz << std::endl; exit(0);
			InitializeNormal(xx, yy, yaw, initVar);
			mcl_loaded_ = true;
			std::cout << "MAP LOADED SUCCESSFULLY :)" << std::endl;
			
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
			}
		}
		
	};
	
	
	

}

}

#endif