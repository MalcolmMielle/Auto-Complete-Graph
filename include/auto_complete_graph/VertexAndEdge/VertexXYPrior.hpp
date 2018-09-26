#ifndef AUTOCOMPLETEGRAPH_VERTEXXYPRIOR_17042018
#define AUTOCOMPLETEGRAPH_VERTEXXYPRIOR_17042018

#include "VertexPointXYACG.hpp"
#include "EdgeInterfaceMalcolm.hpp"
#include "EdgeXYPrior.hpp"
#include "auto_complete_graph/PriorLoaderInterface.hpp"
//#include "VertexLandmarkNDT.hpp"

namespace g2o{

	class VertexXYPrior : public g2o::VertexPointXYACG
	{

//		struct anglesPrior{
//			g2o::EdgeXYPriorACG* from;
//			g2o::EdgeXYPriorACG* toward;
//			VertexXYPrior* base;
//			double first_angle_in_rad;
//
//			double diff(){
//
//			}
//
//		};


	protected:

//		std::set<anglesPrior> _angles_neighbors;



	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		VertexXYPrior() : g2o::VertexPointXYACG(){}
		AASS::acg::PriorAttr priorattr;


		std::vector<g2o::EdgeXYPriorACG*> getPriorEgdes() const {

			auto edges = this->edges();
			std::vector<g2o::EdgeXYPriorACG*> edges_prior;
			//Get only prior edges
			for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
//					std::cout << "pointer " << dynamic_cast<g2o::EdgeXYPriorACG*>(*ite) << std::endl;
				g2o::EdgeXYPriorACG* ptr = dynamic_cast<g2o::EdgeXYPriorACG*>(*ite);
				if(ptr != NULL){
//						std::cout << "Ptr non nul :)" << std::endl;
					//Make sure not pushed twice
					for(auto ite2 = edges_prior.begin(); ite2 != edges_prior.end(); ++ite2 ){
						assert(ptr != *ite2);
					}
// 						std::cout << "pushing " << ptr << std::endl;
// 						std::cout << " pushed edges " << std::endl;
					edges_prior.push_back(ptr);
// 						std::cout << "pushed edges done " << std::endl;
				}
			}
			return edges_prior;
		}

//		void initAngles(){
//			std::cout << "New vert"<<std::endl;
//
//			auto edges_prior = getPriorEgdes();
//
//			if(edges_prior.size() > 1) {
////					std::cout << "This should always be right :) : edges_prior.size() : " << edges_prior.size() << std::endl;
//				auto comp = [this](g2o::EdgeXYPriorACG *a, g2o::EdgeXYPriorACG *b) {
//					auto from_vec2d = a->getOrientation2D(*this);
//					auto to_vec2d = b->getOrientation2D(*this);
//					//Rotate
//
//					double angle_from = atan2(from_vec2d(1), from_vec2d(0)) - atan2(0, 1);
//					if (angle_from < 0) angle_from += 2 * M_PI;
//					double angle_to = atan2(to_vec2d(1), to_vec2d(0)) - atan2(0, 1);
//					if (angle_to < 0) angle_to += 2 * M_PI;
//
//					return angle_from < angle_to;
//
//				};
//
//				std::sort(edges_prior.begin(), edges_prior.end(), comp);
//
//// 					std::cout << "angle first " << edges.size() << std::endl;
//				auto ite = edges_prior.begin();
//				auto ite_end = edges_prior.back();
//
//				anglesPrior ap;
//				ap.from = ite_end;
//				ap.toward = *ite;
//				ap.base = this;
//				ap.first_angle_in_rad = angle((*ite_end), *(*ite)).first;
//				_angles_neighbors.insert(ap);
//
//			}
//		}


//		double angleDiff(g2o::EdgeXYPriorACG* of_this_edge){
//			double total_diff = 0;
//			for(auto angles : _angles_neighbors){
//				if(angles.from == of_this_edge || angles.toward == of_this_edge){
//					double angles_updated = angle(*angles.from, *angles.toward).first;
//					double diff = std::abs(angles_updated - angles.first_angle_in_rad);
//					total_diff += diff;
//				}
//			}
//			return total_diff;
//		}

		//List of associated landmark. For visualization purpose
//		std::set<const g2o::VertexLandmarkNDT*> landmarks;

		//Repeating code between VertexSE2Prior and VertexXYPrior...
		std::vector<std::pair<double, double> > getAnglesAndOrientations() const {

// 			std::cout << "New vert"<<std::endl;

			std::vector<std::pair<double, double> > out;
// 			std::cout << "edges " << std::endl;
//			auto edges = this->edges();
// 			std::cout << "edges done " << std::endl;
//			std::vector<g2o::EdgeXYPriorACG*> edges_prior;
			auto edges_prior = getPriorEgdes();
			//Get only prior edges
//			if(edges.size() >= 1){
//				for ( auto ite = edges.begin(); ite != edges.end(); ++ite ){
////					std::cout << "pointer " << dynamic_cast<g2o::EdgeXYPriorACG*>(*ite) << std::endl;
//					g2o::EdgeXYPriorACG* ptr = dynamic_cast<g2o::EdgeXYPriorACG*>(*ite);
//					if(ptr != NULL){
////						std::cout << "Ptr non nul :)" << std::endl;
//						//Make sure not pushed twice
//						for(auto ite2 = edges_prior.begin(); ite2 != edges_prior.end(); ++ite2 ){
//							assert(ptr != *ite2);
//						}
//// 						std::cout << "pushing " << ptr << std::endl;
//// 						std::cout << " pushed edges " << std::endl;
//						edges_prior.push_back(ptr);
//// 						std::cout << "pushed edges done " << std::endl;
//					}
//				}

			if(edges_prior.size() > 1){
//					std::cout << "This should always be right :) : edges_prior.size() : " << edges_prior.size() << std::endl;
				auto comp = [this](g2o::EdgeXYPriorACG* a, g2o::EdgeXYPriorACG* b)
				{
					auto from_vec2d = a->getOrientation2D(*this);
					auto to_vec2d = b->getOrientation2D(*this);
					//Rotate

					double angle_from = atan2(from_vec2d(1), from_vec2d(0)) - atan2(0, 1);
					if (angle_from < 0) angle_from += 2 * M_PI;
					double angle_to = atan2(to_vec2d(1), to_vec2d(0)) - atan2(0, 1);
					if (angle_to < 0) angle_to += 2 * M_PI;

					return angle_from < angle_to;

				};

				std::sort( edges_prior.begin(), edges_prior.end(), comp );

// 					std::cout << "angle first " << edges.size() << std::endl;
				auto ite = edges_prior.begin();
				auto ite_end = edges_prior.back();
				out.push_back( angle( (*ite_end), *(*ite) ) );
				assert(out.back().first >= 0);
				assert(out.back().second >= 0);
				assert(out.back().first <= 2 * M_PI);
				assert(out.back().second <= 2 * M_PI);

				for ( auto ite = edges_prior.begin(); ite != edges_prior.end() - 1 ; ++ite ){
// 						std::cout << "angle more " << edges.size() << std::endl;
					out.push_back( angle( **ite, **(ite + 1) ) );
					assert(out.back().first >= 0);
					assert(out.back().second >= 0);
					assert(out.back().first <= 2 * M_PI);
					assert(out.back().second <= 2 * M_PI);
				}
			}
//				else{
//					throw std::runtime_error("No edge prior :( ??");
//				}

//			}
			else{
				if(edges_prior.size() == 0) {
					throw std::runtime_error("No edges at all :(");
				}
			}
			return out;

		}

	private:
		std::pair<double, double> angle(const g2o::EdgeXYPriorACG& from, const g2o::EdgeXYPriorACG& to) const {


			auto from_vec2d = from.getOrientation2D(*this);
			auto to_vec2d = to.getOrientation2D(*this);

// 			std::cout << "from " << from_vec2d << " , " << &from << " to " << to_vec2d << ", " << &to << std::endl;
			//Rotate
// 			angle = atan2(vector2.y, vector2.x) - atan2(vector1.y, vector1.x);
			double angle_between = atan2(to_vec2d(1), to_vec2d(0)) - atan2(from_vec2d(1), from_vec2d(0));
// 			std::cout << "Angle between " << angle_between << std::endl;
			if (angle_between < 0) angle_between += 2 * M_PI;
// 			std::cout << "Angle between " << angle_between << std::endl;

			double angle_from = atan2(from_vec2d(1), from_vec2d(0)) - atan2(0, 1);
			if (angle_from < 0) angle_from += 2 * M_PI;

			double angle_to = atan2(to_vec2d(1), to_vec2d(0)) - atan2(0, 1);
			if (angle_to < 0) angle_to += 2 * M_PI;

			double direction = (angle_to + angle_from) / 2;
			assert(direction <= 2 * M_PI);

			if(angle_from > angle_to){direction = direction + M_PI;}
			while (direction >= 2* M_PI){direction = direction - (2 * M_PI);}

			double width = std::abs(angle_to - angle_from);

// 			std::cout << "Angle between " << angle_between << std::endl;
//			assert(angle_between >= 0.08);
//			assert(direction >= 0);
//			assert(angle_between >= 0);
//			assert(direction <= 2 * M_PI);
//			assert(angle_between <= 2 * M_PI);

			return std::pair<double, double>(angle_between, direction);
		}


	};


}
#endif