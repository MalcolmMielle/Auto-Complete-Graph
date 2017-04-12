#ifndef AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016
#define AUTOCOMPLETEGRAPH_PRIORLOADERINTERFACE_16112016

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include "das/CornerDetector.hpp"
#include "KeypointPriorDetector.hpp"

namespace AASS{
	namespace acg{
		
		
		class PriorAttr : public AASS::vodigrex::SimpleNode{
		public:
			cv::KeyPoint keypoint;
			cv::Point2d position;
			cv::Mat descriptor;
			PriorAttr() : AASS::vodigrex::SimpleNode(){};
			PriorAttr(const AASS::vodigrex::SimpleNode& inp) : AASS::vodigrex::SimpleNode(inp){}
		};
		

		/**
		 * @brief The class with all the hardcoded first guess things
		 */
		class PriorLoaderInterface{
		public : 
			typedef typename bettergraph::PseudoGraph<PriorAttr, AASS::vodigrex::SimpleEdge> PriorGraph;
			typedef typename bettergraph::PseudoGraph<PriorAttr, AASS::vodigrex::SimpleEdge>::GraphType PriorGraphType;
			typedef typename boost::graph_traits<PriorGraphType>::vertex_iterator PriorVertexIterator;
			typedef typename boost::graph_traits<PriorGraphType>::vertex_descriptor PriorVertex;
			typedef typename boost::graph_traits<PriorGraphType>::edge_descriptor PriorEdge;
			typedef typename boost::graph_traits<PriorGraphType>::out_edge_iterator PriorEdgeIterator;
			
		protected:
			cv::Mat _img_gray;
			std::vector<cv::Point2f> _corner_prior;
			std::vector<cv::Point2f> _same_point_prior;
			std::vector<cv::Point2f> _same_point_slam;
			std::vector<cv::Point2f> _corner_prior_matched;
			bettergraph::PseudoGraph<PriorAttr, AASS::vodigrex::SimpleEdge> _prior_graph;
			cv::Mat _scale_transform_prior2ndt;
			std::string _file;
			AASS::das::CornerDetector _cornerDetect;
			double _deviation;
			double _angle;
			double _scale;
			cv::Point2f _center;
			
			KeypointPriorDetector _keypoints;
			
		public: 
			PriorLoaderInterface(const std::string& file) : _file(file){		
			}
			
			const PriorGraph getGraph() const {return _prior_graph;}
			PriorGraph getGraph(){return _prior_graph;}
			
			double getDeviation(){return _deviation;}
			double getAngle(){return _angle;}
			double getScale(){return _scale;}
			cv::Point2f getCenter(){return _center;}
			
			/**
			 * @brief Extract the corners and do the transformation onto the SLAM
			 */
			void prepare(){
				extractCornerPrior();
				transformOntoSLAM();
			}
			
			/**
			* @brief Extracting the corner from the prior
			*/
			void extractCornerPrior(){
		// 		AASS::das::BasementPriorLine basement;
				
				_corner_prior.clear();
				_prior_graph.clear();
				
				_cornerDetect.clear();
				
				cv::Mat src = cv::imread( _file, 1 );
				cv::cvtColor( src, _img_gray, CV_BGR2GRAY );
				
// 				cv::imshow("TEST IMG", _img_gray);
// 				cv::waitKey(0);
				
				_cornerDetect.getFeaturesGraph(_img_gray);
				
// 				cornerDetect.removeClosePoints(20);
				_corner_prior = _cornerDetect.getGraphPoint();
				auto prior_graph = _cornerDetect.getGraph();
				
				extractKeypoints(prior_graph);
				
// 				//PRINT
// 				cv::Mat src = cv::imread( _file, CV_LOAD_IMAGE_COLOR ), src_gray;
// 				cv::cvtColor(src, src_gray, CV_RGB2GRAY );
// // 				
// 				cv::threshold(src_gray, src_gray, 100, 255, src_gray.type());
// 				cv::Mat maa_3 = src_gray.clone();
// 				maa_3.setTo(cv::Scalar(0));
// 				AASS::vodigrex::draw<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>(_prior_graph, maa_3);
// 				std::cout << "Number of nodes " << _prior_graph.getNumVertices() << std::endl;
// 				cv::imshow("graph", maa_3);
// 				cv::imshow("map", src_gray);
// 				cv::waitKey(0);
				
			}
			
			/**
			 * @brief scale the corner and the graph depending on the _scale_transform_prior2ndt attribute
			 */
			void transformOntoSLAM(){
				
				
// 				std::cout << "Transform onto slam from scale : " << std::endl << _scale_transform_prior2ndt << std::endl;
				//Scale transform here:		
				//Then create association
// 				cv::perspectiveTransform( _corner_prior, _corner_prior_matched, _scale_transform_prior2ndt);
				
				assert(_same_point_prior.size() >= 2);
				
				cv::Point2f srcTri[2];
				cv::Point2f dstTri[2];
				srcTri[0] = _same_point_prior[0];
				srcTri[1] = _same_point_prior[1];
// 				srcTri[2] = _same_point_prior[2];
				
				dstTri[0] = _same_point_slam[0];
				dstTri[1] = _same_point_slam[1];

				std::cout << "Point for transfo" << std::endl;
				std::cout << _same_point_prior[0] << " " << _same_point_slam[0] << std::endl << _same_point_prior[1] << " " << _same_point_slam[1] << std::endl;
								
				//Translate to origin
				cv::Point2d translation = - srcTri[0];
				double angle = 0;
				cv::Mat warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);
				
				//Rotate
				Eigen::Vector2d vece1s; vece1s << srcTri[1].x - srcTri[0].x, srcTri[1].y - srcTri[0].y;
				Eigen::Vector2d vece2s; vece2s << dstTri[1].x - dstTri[0].x, dstTri[1].y - dstTri[0].y;
				
				angle = atan2(vece2s(1),vece2s(0)) - atan2(vece1s(1), vece1s(0));
				if (angle < 0) angle += 2 * M_PI;
				
// 				std::cout << "Angle : "<< angle << std::endl;
// 				angle = angle * 2 * 3.14157 / 360;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1);
// 				std::cout << "mat " << warp_mat << std::endl;
				AffineTransformGraph(warp_mat);
				
				//Scale
				cv::Point2f vec1 = srcTri[1] - srcTri[0];
				cv::Point2f vec2 = dstTri[1] - dstTri[0];
				cv::Mat mat_transfo =  (cv::Mat_<float>(2,2) << vec2.x / vec1.x, 0, 0, vec2.y / vec1.y);
				double l1 = std::sqrt( ( vec1.x * vec1.x ) + ( vec1.y * vec1.y ) );
				double l2 = std::sqrt( ( vec2.x * vec2.x ) + ( vec2.y * vec2.y ) );
				double ratio = l2 / l1;
				
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
// 				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					cv::Point2d point_out = point * ratio;

					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
				}
// 				//Translate back
				translation = srcTri[0];
				angle = 0;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);
// 				
// 				//Translate to point
				translation = dstTri[0] - srcTri[0];
				angle = 0;
				warp_mat = (cv::Mat_<double>(3,3) << std::cos(angle), -std::sin(angle), translation.x, std::sin(angle), std::cos(angle), translation.y, 0, 0, 1);
				AffineTransformGraph(warp_mat);

				
			}
			
			/**
			 * @brief initialze the structure
			 * @param[in] pt_slam : points from the robot slam map
			 * @param[in] pt_prior : equivalent points from the robot slam map
			 */
			void initialize(const std::vector<cv::Point2f>& pt_slam, const std::vector<cv::Point2f>& pt_prior, double deviation, double angle, double scale, cv::Point2f center){
				
				_deviation = deviation;
				_angle = angle;
				_scale = scale;
				_center = center;
				
				initialize(pt_slam, pt_prior);
				
			}
			
			/**
			 * @brief initialze the structure
			 * @param[in] pt_slam : points from the robot slam map
			 * @param[in] pt_prior : equivalent points from the robot slam map
			 */
			void initialize(const std::vector<cv::Point2f>& pt_slam, const std::vector<cv::Point2f>& pt_prior){
				
				assert(pt_slam.size() >= 2);
				assert(pt_prior.size() == pt_slam.size());
				
				std::cout << pt_prior[0] << " " << pt_slam[0] << std::endl << pt_prior[1] << " " << pt_slam[1] << std::endl;
				
				_same_point_prior.clear();
				_same_point_slam.clear();
				
				this->_cornerDetect.setMinimumDeviationCorner( (85 * 3.14159) / 180 );
				
				auto randomNoise = [](double mean, double deviationt) -> double {
					std::default_random_engine engine{std::random_device()() };
					std::normal_distribution<double> dist(mean, deviationt);
					return dist(engine);
				};
				
// 				cv::Point center = cv::Point(7, 08);
				
				cv::Mat rot_mat = cv::getRotationMatrix2D(_center, _angle, _scale);
				std::cout << rot_mat << std::endl;
// 				exit(0);
				
				auto rotatef = [](const cv::Mat& rot_mat, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y, 1);
					cv::Mat mat_out = rot_mat * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					return point_out;
				};
				
				auto noise_x = randomNoise(0, _deviation);
				auto noise_y = randomNoise(0, _deviation);
				
				std::cout << "Noise X Y " << noise_x << " " << noise_y << std::endl;
// 				exit(0);
				
				cv::Point2f out = cv::Point2f(pt_slam[0].x + noise_x, pt_slam[0].y + noise_y);
				cv::Point2f slam_point = rotatef(rot_mat, out);
				
				_same_point_prior.push_back(pt_prior[0]);
				_same_point_slam.push_back(slam_point);
				
				noise_x = randomNoise(0, _deviation);
				noise_y = randomNoise(0, _deviation);
				out = cv::Point2f(pt_slam[1].x + noise_x, pt_slam[1].y + noise_y);
				slam_point = rotatef(rot_mat, out);
				
				_same_point_prior.push_back(pt_prior[1]);                   
				_same_point_slam.push_back(slam_point);
				
// 				noise_x = randomNoise(0, _deviation);
// 				noise_y = randomNoise(0, _deviation);
// 				out = cv::Point2f(pt_slam[2].x + noise_x, pt_slam[2].y + noise_y);
// 				slam_point = rotatef(rot_mat, out);
// 				
// 				//ATTENTION : next line or for used the points only
// 				
// 				_same_point_prior.push_back(pt_prior[2]);
// 				_same_point_slam.push_back(slam_point);
// 				
// 				noise_x = randomNoise(0, _deviation);
// 				noise_y = randomNoise(0, _deviation);
// 				out = cv::Point2f(pt_slam[3].x + noise_x, pt_slam[3].y + noise_y);
// 				slam_point = rotatef(rot_mat, out);
// 				
// 				_same_point_prior.push_back(pt_prior[3]);              
// 				_same_point_slam.push_back(slam_point);
				
// 				_same_point_prior.push_back(cv::Point2f(786, 373));
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				_same_point_prior.push_back(cv::Point2f(788, 311));                   
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				//ATTENTION : next line or for used the points only
// 				
// 				_same_point_prior.push_back(cv::Point2f(614, 306));
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
// 				
// 				_same_point_prior.push_back(cv::Point2f(637, 529));              
// 				_same_point_slam.push_back(cv::Point2f(786, 373));
				
// 				_scale_transform_prior2ndt = cv::findHomography(_same_point_prior, _same_point_slam, CV_RANSAC, 3, cv::noArray());
				
			}
			
			
		protected:
			
		
			
			
			void extractKeypoints(const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>& graph){
				
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				std::vector<AASS::das::CornerDetector::CornerVertex> das_v;
				std::vector<PriorVertex> prior_v;
				
				
				std::cout << "Keypoints" << std::endl;
				//Adding all the vertices
				std::pair< AASS::das::CornerDetector::CornerVertexIterator, AASS::das::CornerDetector::CornerVertexIterator > vp;
				for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					PriorVertex vertex_out;
					PriorAttr nodeAttribute(graph[v]);
					
					
					//Creating a SIFT descriptor for eahc corner
					cv::KeyPoint keypoint;
					keypoint.pt = cv::Point2f( nodeAttribute.getX(), nodeAttribute.getY() );
					
					//Calculate smallest edge size
					std::deque<AASS::das::CornerDetector::CornerVertex> out;
					graph.getAllVertexLinked(v, out); 
					double smallest = -1;
					for(auto it = out.begin() ; it != out.end() ; ++it){
						if(v != *it){
							double tmp_size_x = (graph[v].getX() - graph[*it].getX());
							double tmp_size_y = (graph[v].getY() - graph[*it].getY());
							tmp_size_x = std::abs(tmp_size_x);
							tmp_size_y = std::abs(tmp_size_y);
							double tmp_size = (tmp_size_x * tmp_size_x) + (tmp_size_y * tmp_size_y);
							if(smallest == -1 || smallest > tmp_size){
								if(tmp_size >  0){
// 									std::cout << "Smallest size " << tmp_size << std::endl;
									smallest = tmp_size;
								}
								else{
// 									std::cout << "Weird zero distance between points" <<std::endl;
								}
							}
						}
					}
					
					keypoint.size = std::sqrt(smallest) ;
					keypoint.angle = -1 ;
					keypoint.octave = 1 ;
					
// 					cv::Mat copy;
// 					_img_gray.copyTo(copy);
// 					cv::circle(copy, cv::Point2f( nodeAttribute.getX(), nodeAttribute.getY() ), keypoint.size, cv::Scalar(255), 1);
// 					
// 					cv::imshow("Keypoint", copy);
// 					cv::waitKey(0);
					
					std::vector<cv::KeyPoint> keypoint_v;
					keypoint_v.push_back(keypoint);
					
					cv::Mat descriptors_1;
					cv::SiftDescriptorExtractor extractor;
					extractor.compute( _img_gray, keypoint_v, descriptors_1);
					
// 					std::cout << descriptors_1.rows << " " << descriptors_1.cols << std::endl;
					assert(descriptors_1.rows == 1);
					
					nodeAttribute.keypoint = keypoint;
					nodeAttribute.position = keypoint.pt;
					nodeAttribute.descriptor = descriptors_1;
					
					_prior_graph.addVertex(vertex_out, nodeAttribute);
					das_v.push_back(v);
					prior_v.push_back(vertex_out);
					
				}
				
				std::cout << "Done" << std::endl;
				
				//Adding all the edges
				auto es = boost::edges(graph);
				for (auto eit = es.first; eit != es.second; ++eit) {
				//Since we fuse the old zone in biggest we only need to link them to biggest
					auto sour = boost::source(*eit, graph);			
					auto targ = boost::target(*eit, graph);
					PriorVertex source_p;
					PriorVertex target_p;
					bool flag_found_src = false;
					bool flag_found_targ = false;
					for(int i = 0 ; i < das_v.size() ; ++i){
						if(das_v[i] == sour){
							source_p = prior_v[i];
							assert(flag_found_src == false);
							flag_found_src = true;
						}
						if(das_v[i] == targ){
							target_p = prior_v[i];
							assert(flag_found_targ == false);
							flag_found_targ = true;
						}
					}
					assert(flag_found_src == true);
					assert(flag_found_targ == true);
					
					PriorEdge out_edge;
					_prior_graph.addEdge(out_edge, source_p, target_p, graph[*eit]);
				}
							
			}
			
			
			void rotateGraph(const cv::Mat& rot_mat ){
				
				
				auto transf = [](const cv::Mat& rot_mat, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y);
					cv::Mat mat_out = rot_mat * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					return point_out;
				};
				
				
				
				
				std::pair< PriorVertexIterator, PriorVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					auto point_out = transf(rot_mat, point);
					
					//Matrix multiplication
					
					std::cout << "Point out " << point_out << std::endl;
					
					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
					_corner_prior_matched.push_back(point_out);
					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
			}
			
			void AffineTransformGraph(const cv::Mat& warp_transfo ){
				
				
				auto transf = [](const cv::Mat& warp_transfo, const cv::Point2d point) -> cv::Point2d {
					//Matrix multiplication
					cv::Mat point_m = (cv::Mat_<double>(3,1) << point.x, point.y, 1);
					cv::Mat mat_out = warp_transfo * point_m;
// 					std::cout << "Mat out " << mat_out << std::endl;
					cv::Point2d point_out;
					point_out.x = mat_out.at<double>(0);
					point_out.y = mat_out.at<double>(1);
					std::cout << "reutnr point " << point_out.x << " " << point_out.y << std::endl;
					return point_out;
				};
				
				
				
				
				std::pair< PriorVertexIterator, PriorVertexIterator > vp;
				//vertices access all the vertix
				//Classify them in order
	// 				std::cout << "Gph size : " << _graph.getNumVertices() << std::endl;
				int i = 0 ;
				for (vp = boost::vertices(_prior_graph); vp.first != vp.second; ++vp.first) {
	// 					std::cout << "going throught grph " << i << std::endl; ++i;
					AASS::das::CornerDetector::CornerVertex v = *vp.first;
					//ATTENTION !
					cv::Point2d point;
					point.x = _prior_graph[v].getX();
					point.y = _prior_graph[v].getY();
					
					cv::Point2d point_out = transf(warp_transfo, point);
					
					//Matrix multiplication
					
// 					std::cout << "Point out " << point_out << std::endl;
					
// 					std::cout << "OLD value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					_prior_graph[v].setX(point_out.x);
					_prior_graph[v].setY(point_out.y);
					_corner_prior_matched.push_back(point_out);
// 					std::cout << "New value " << _corner_prior_matched[i].x << " " << _corner_prior_matched[i].y << std::endl;
// 					std::cout << "New value " << _prior_graph[v].getX() << " " << _prior_graph[v].getY() << std::endl;
					++i;
				}
				
				
			}
			
		};
		
	}
}

#endif