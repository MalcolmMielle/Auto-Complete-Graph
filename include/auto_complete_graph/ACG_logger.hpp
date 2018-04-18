#ifndef AUTOCOMPLETEGRAPH_ACGLOGGER_01062017
#define AUTOCOMPLETEGRAPH_ACGLOGGER_01062017

#include "ACGBase.hpp"

namespace AASS{
namespace acg{
	
	class AutoCompleteGraphLogger : public AutoCompleteGraph{
		
	protected:
		std::string _file;
		
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		AutoCompleteGraphLogger(const std::string& logfile, const g2o::SE2& sensoffset, const std::string& load_file) : AutoCompleteGraph(sensoffset, load_file), _file(logfile){}
		
		AutoCompleteGraphLogger(const std::string& logfile, const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn, ndt_feature::NDTFeatureGraph* ndt_graph) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn, ndt_graph), _file(logfile){}
		
		AutoCompleteGraphLogger(const std::string& logfile, const g2o::SE2& sensoffset, const Eigen::Vector2d& tn, double rn, const Eigen::Vector2d& ln, const Eigen::Vector2d& pn, double rp, const Eigen::Vector2d& linkn) : AutoCompleteGraph(sensoffset, tn, rn, ln, pn, rp, linkn), _file(logfile){}
		
		
		void log(){
			
			std::cout <<"Log in " << _file << std::endl;
			std::ofstream out(_file.c_str(), std::ios::in | std::ios::out | std::ios::ate);
// 				out.open (_file_out_logger.c_str());
			out << "New step" << "\n\n";
			out.close();
						
			for(auto it = _nodes_ndt.begin() ; it != _nodes_ndt.end() ; ++it){
				auto vec2d = (*it)->estimate().toVector();
				ros::Time time((*it)->getTime());
				logT(vec2d, time);
			}
			
		}
	
		void logT(const Eigen::Vector3d& T_out, ros::Time time){
			
			std::cout <<"Log T " << _file << std::endl;
			std::ofstream out(_file.c_str(), std::ios::in | std::ios::out | std::ios::ate);
// 				out.open (_file_out_logger.c_str());
			
			std::cout << T_out(0) << " " << T_out(1) << " " << T_out(2) << " " << time << std::endl;
			out << T_out(0) << " " << T_out(1) << " " << T_out(2) << " " << time << "\n";
			out.close();
			
		}
		
// 		private:
// 		Eigen::Affine2d eigenAffine3dTo2d(const Eigen::Affine3d &a3d) {
// 			return Eigen::Translation2d(a3d.translation().topRows<2>()) *
// 				Eigen::Rotation2D<double>(getRobustYawFromAffine3d(a3d));//a3d.linear().topLeftCorner<2,2>();
// 		}
// 		
// 		double getRobustYawFromAffine3d(const Eigen::Affine3d &a) {
// 			// To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
// 			Eigen::Vector3d v1(1,0,0);
// 			Eigen::Vector3d v2 = a.rotation()*v1;
// 			double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
// 			double angle = acos(dot);
// 			// Need to find the sign
// 			if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
// 				return angle;
// 			return -angle;
// 		}
		
	};
}
}

#endif