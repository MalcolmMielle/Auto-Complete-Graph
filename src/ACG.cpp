#include "auto_complete_graph/ACG.hpp"

g2o::VertexSE2* AASS::acg::AutoCompleteGraph::addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map){
	
	std::cout << "Adding the robot pose " << std::endl;
	g2o::VertexSE2* robot =  new g2o::VertexSE2;
	robot->setEstimate(se2);
	robot->setId(_optimizable_graph.vertices().size());
	_optimizable_graph.addVertex(robot);
	
	NDTNodeAndMap nodeAndMap(robot, map, affine);
	
	_nodes_ndt.push_back(nodeAndMap);
	return robot;
}
g2o::VertexSE2* AASS::acg::AutoCompleteGraph::addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map){
	g2o::SE2 se2(rob(0), rob(1), rob(2));
	return addRobotPose(se2, affine, map);
}
g2o::VertexSE2* AASS::acg::AutoCompleteGraph::addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, lslgeneric::NDTMap* map){
	Eigen::Vector3d robot1;
	robot1 << x, y, theta;
	return addRobotPose(robot1, affine, map);
}

g2o::VertexPointXY* AASS::acg::AutoCompleteGraph::addLandmarkPose(const g2o::Vector2D& pos, int strength){
	g2o::VertexPointXY* landmark = new g2o::VertexPointXY;
	landmark->setId(_optimizable_graph.vertices().size());
	std::cout << "Setting the id " << _optimizable_graph.vertices().size() << std::endl;
	landmark->setEstimate(pos);
	_optimizable_graph.addVertex(landmark);
	_nodes_landmark.push_back(landmark);
	return landmark;
}
g2o::VertexPointXY* AASS::acg::AutoCompleteGraph::addLandmarkPose(double x, double y, int strength){
	g2o::Vector2D lan;
	lan << x, y;
	return addLandmarkPose(lan, strength);
}

g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(const g2o::SE2& se2){
	g2o::VertexSE2Prior* priorlandmark = new g2o::VertexSE2Prior;
	priorlandmark->setId(_optimizable_graph.vertices().size());
	priorlandmark->setEstimate(se2);
	_optimizable_graph.addVertex(priorlandmark);
	_nodes_prior.push_back(priorlandmark);
	return priorlandmark;
}
g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(const Eigen::Vector3d& lan){
	g2o::SE2 se2(lan(0), lan(1), lan(2));
	return addPriorLandmarkPose(se2);
}
g2o::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(double x, double y, double theta){
	Eigen::Vector3d lan;
	lan << x, y, theta;
	return addPriorLandmarkPose(lan);
}


g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information){
	
	g2o::EdgeOdometry_malcolm* odometry = new g2o::EdgeOdometry_malcolm;
	odometry->vertices()[0] = v1 ;
	odometry->vertices()[1] = v2 ;
	odometry->setMeasurement(se2);
	odometry->setInformation(information);
	_optimizable_graph.addEdge(odometry);
	_edge_odometry.push_back(odometry);
	return odometry;
}

g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(const g2o::SE2& observ, int from_id, int toward_id, const Eigen::Matrix3d& information){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addOdometry(observ, from_ptr, toward_ptr, information);
}
g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(double x, double y, double theta, int from_id, int toward_id, const Eigen::Matrix3d& information){
	g2o::SE2 se2(x, y, theta);
	return addOdometry(se2, from_id, toward_id, information);
}

g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
			
	Eigen::Matrix3d covariance;
	covariance.fill(0.);
	covariance(0, 0) = _transNoise[0]*_transNoise[0];
	covariance(1, 1) = _transNoise[1]*_transNoise[1];
	covariance(2, 2) = _rotNoise*_rotNoise;
	Eigen::Matrix3d information = covariance.inverse();
	
	return addOdometry(se2, v1, v2, information);
}



g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(const g2o::SE2& observ, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addOdometry(observ, from_ptr, toward_ptr);
}
g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(double x, double y, double theta, int from_id, int toward_id){
	g2o::SE2 se2(x, y, theta);
	return addOdometry(se2, from_id, toward_id);
}

g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraph::addLandmarkObservation(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	Eigen::Matrix2d covariance_landmark; 
	covariance_landmark.fill(0.);
	covariance_landmark(0, 0) = _landmarkNoise[0]*_landmarkNoise[0];
	covariance_landmark(1, 1) = _landmarkNoise[1]*_landmarkNoise[1];
// 			covariance_landmark(2, 2) = 13;//<- Rotation covariance landmark is more than 4PI
	Eigen::Matrix2d information_landmark = covariance_landmark.inverse();
	
	g2o::EdgeLandmark_malcolm* landmarkObservation =  new g2o::EdgeLandmark_malcolm;
	landmarkObservation->vertices()[0] = v1;
	landmarkObservation->vertices()[1] = v2;
	landmarkObservation->setMeasurement(pos);
	landmarkObservation->setInformation(information_landmark);
	landmarkObservation->setParameterId(0, _sensorOffset->id());
	_optimizable_graph.addEdge(landmarkObservation);
	_edge_landmark.push_back(landmarkObservation);
	
	return landmarkObservation;
}
g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraph::addLandmarkObservation(const g2o::Vector2D& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addLandmarkObservation(pos, from_ptr, toward_ptr);
}

g2o::EdgeSE2Prior_malcolm* AASS::acg::AutoCompleteGraph::addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	
	
	//Get Eigen vector
	g2o::VertexSE2Prior* v_ptr = dynamic_cast<g2o::VertexSE2Prior*>(v1);
	g2o::VertexSE2Prior* v_ptr2 = dynamic_cast<g2o::VertexSE2Prior*>(v2);
	Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
	Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
	
// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;
	
	Eigen::Vector2d eigenvec; 
	eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
	std::pair<double, double> eigenval(_priorNoise(0), _priorNoise(1));
	
	Eigen::Matrix2d cov = getCovarianceVec(eigenvec, eigenval);
	
// 			std::cout << "Covariance prior " << std::endl << cov.format(cleanFmt) << std::endl;
	
	Eigen::Matrix3d covariance_prior;
	covariance_prior.fill(0.);
	covariance_prior(0, 0) = cov(0, 0);
	covariance_prior(0, 1) = cov(0, 1);
	covariance_prior(1, 0) = cov(1, 0);
	covariance_prior(1, 1) = cov(1, 1);
// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
	covariance_prior(2, 2) = _prior_rot * _prior_rot;
	Eigen::Matrix3d information_prior = covariance_prior.inverse();
// 			std::cout << "Information prior " << std::endl << cov.format(cleanFmt) << std::endl;
	
	
	
	g2o::EdgeSE2Prior_malcolm* priorObservation =  new g2o::EdgeSE2Prior_malcolm;
	priorObservation->vertices()[0] = v1;
	priorObservation->vertices()[1] = v2;
	priorObservation->setMeasurement(se2);
	priorObservation->setInformation(information_prior);
	priorObservation->setParameterId(0, _sensorOffset->id());
	priorObservation->interface.setOriginalValue(se2);
	
	_optimizable_graph.addEdge(priorObservation);
	
// 	EdgePriorAndInitialValue epiv(priorObservation, se2);
	_edge_prior.push_back(priorObservation);
	return priorObservation;
}

// void AASS::acg::AutoCompleteGraph::addEdgePrior(g2o::SE2 observ, int from, int toward){
// 	std::tuple<g2o::SE2, int, int> obs1(observ, from, toward);
// 	addEdgePrior(obs1);
// }
// void AASS::acg::AutoCompleteGraph::addEdgePrior(double x, double y, double theta, int from, int toward){
// 	g2o::SE2 se2(x, y, theta);
// 	addEdgePrior(se2, from, toward);
// 	
// }

g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraph::addLinkBetweenMaps(const g2o::Vector2D& pos, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	
	std::cout << "Adding link" << std::endl;
	//Making sure the two node are the good type
	g2o::HyperGraph::Vertex* from;
	g2o::HyperGraph::Vertex* toward;
	g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>(v1);
	g2o::VertexPointXY* ptr2;
	if(ptr != NULL){
		g2o::VertexPointXY* ptr2 = dynamic_cast<g2o::VertexPointXY*>(v2);
		if(ptr2 == NULL){
			throw std::runtime_error("Pointers are not of compatible type. First pointer is a SE2 while the second is not a PointXY");
		}
		else{
			from = v1;
			toward = v2;
		}
	}
	else{
		ptr = dynamic_cast<g2o::VertexSE2Prior*>(v2);
		if(ptr != NULL){
			ptr2 = dynamic_cast<g2o::VertexPointXY*>(v1);
			if(ptr2 == NULL){
				throw std::runtime_error("Pointers are not of compatible type. Second pointer is a SE2 while the first is not a PointXY");
			}
			else{
				from = v2;
				toward = v1;
			}
		}
		else{
			throw std::runtime_error("Pointers are not of compatible type. No pointer point to a VertexSE2Prior");
		}
	}
	
	Eigen::Matrix2d covariance_link; 
	covariance_link.fill(0.);
	covariance_link(0, 0) = _linkNoise[0]*_linkNoise[0];
	covariance_link(1, 1) = _linkNoise[1]*_linkNoise[1];
	
	std::cout << "Link cov " << covariance_link << std::endl;
	
// 			covariance_link(2, 2) = 13;//<- Rotation covariance link is more than 4PI
	Eigen::Matrix2d information_link = covariance_link.inverse();
	
	g2o::EdgeLinkXY_malcolm* linkObservation = new g2o::EdgeLinkXY_malcolm;
	linkObservation->vertices()[0] = from;
	linkObservation->vertices()[1] = toward;
	linkObservation->setMeasurement(pos);
	linkObservation->setInformation(information_link);
	linkObservation->setParameterId(0, _sensorOffset->id());
	
	std::cout << "Adding edge!" <<ptr << ptr2 << std::endl;
	
	_optimizable_graph.addEdge(linkObservation);
	_edge_link.push_back(linkObservation);
	
	std::cout << "Done" << std::endl;
	return linkObservation;
}
g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraph::addLinkBetweenMaps(const g2o::Vector2D& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addLinkBetweenMaps(pos, from_ptr, toward_ptr);
}


void AASS::acg::AutoCompleteGraph::removeLinkBetweenMaps(g2o::EdgeLinkXY_malcolm* v1)
{
// 	//Getting the two vertices
// 	g2o::VertexSE2* ptr;
// 	g2o::VertexPointXY* ptr2;
// 	for(auto ite2 = v1->vertices().begin(); ite2 != v1->vertices().end() ; ++ite2){
// 		g2o::VertexSE2* ptr_tmp = dynamic_cast<g2o::VertexSE2*>((*ite2));
// 		g2o::VertexPointXY* ptr2_tmp = dynamic_cast<g2o::VertexPointXY*>((*ite2));
// 		if(ptr_tmp != NULL){
// 			std::cout << "Got a VertexSE2" << std::endl;
// 			ptr = ptr_tmp;
// 		}
// 		else if(ptr2_tmp != NULL){
// 			std::cout << "Got a VertexPOINTXY" << std::endl;
// 			ptr2 = ptr2_tmp;
// 		}
// 		else{
// 			throw std::runtime_error("Links do not have the good vertex type");
// 		}		
// 	}
	
	_optimizable_graph.removeEdge(v1);
	auto it = _edge_link.begin();
	for(it; it != _edge_link.end() ;){
		if(*it == v1){
			_edge_link.erase(it);
			break;
		}
	}
	
	//TODO :need to do the same for the SLAM
	
	//Get all edges from each vertex
// 	std::vector<g2o::EdgeSE2Prior_malcolm*> all_other_prior_edge;
// 	std::vector<g2o::EdgeLinkXY_malcolm*> all_other_links;
// 	for(auto ite = ptr->edges().begin(); ite != ptr->edges().end() ; ++ite){
// 		
// 		g2o::EdgeSE2Prior_malcolm* ptr_tmp = dynamic_cast<g2o::EdgeSE2Prior_malcolm*>((*ite));
// 		g2o::EdgeLinkXY_malcolm* ptr2_tmp = dynamic_cast<g2o::EdgeLinkXY_malcolm*>((*ite));
// 		if(ptr_tmp != NULL){
// 			all_other_prior_edge.push_back(ptr_tmp);
// 		}
// 		else if(ptr2_tmp != NULL){
// 			std::cout << "Got a VertexPOINTXY" << std::endl;
// 			all_other_links.push_back(ptr2_tmp);
// 		}
// 		else{
// 			throw std::runtime_error("vertex do not have the good edge type");
// 		}		
// 	}
// 	
// 	
// 	//Reinitiallise if no more links
// 	if(all_other_links.size() == 0){
// 		for(auto ite = all_other_prior_edge.begin(); ite != all_other_prior_edge.end() ; ++ite){
// 			auto se2 = (*ite)->getOriginalValue();
// 			(*ite)->setMeasurement(se2);
// 		}
// 	}
	
}



//FUNCTION TO REMOVE A VERTEX
void AASS::acg::AutoCompleteGraph::removeVertex(g2o::HyperGraph::Vertex* v1){
	//Prior
	g2o::VertexSE2Prior* ptr = dynamic_cast<g2o::VertexSE2Prior*>(v1);
	g2o::VertexSE2* ptr_se2 = dynamic_cast<g2o::VertexSE2*>(v1);
	g2o::VertexPointXY* ptr_se3 = dynamic_cast<g2o::VertexPointXY*>(v1);
	
	if(ptr != NULL){
		int index = findPriorNode(v1);
		assert(index != -1);
		std::vector<g2o::VertexSE2Prior*>::iterator which = _nodes_prior.begin() + index;
		_nodes_prior.erase(which);
	}
	//Robot node
	else if( ptr_se2 != NULL){
		int index = findRobotNode(v1);
		assert(index != -1);
		auto which = _nodes_ndt.begin() + index;
		_nodes_ndt.erase(which);
		
	}
	//Landmark Node
	else if( ptr_se3 != NULL){
		int index = findLandmarkNode(v1);
		assert(index != -1);
		std::vector<g2o::VertexPointXY*>::iterator which = _nodes_landmark.begin() + index;
		_nodes_landmark.erase(which);
	}
	else{
		throw std::runtime_error("Vertex type not found in list");
	}
	_optimizable_graph.removeVertex(v1, true);
}

int AASS::acg::AutoCompleteGraph::findRobotNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_ndt.begin();
	for(it ; it != _nodes_ndt.end() ; ++it){
		if(it->getNode() == v){
			return pos;
		}
		++pos;
	}
	return -1;
}
int AASS::acg::AutoCompleteGraph::findLandmarkNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_landmark.begin();
	for(it ; it != _nodes_landmark.end() ; ++it){
		if(*it == v){
			return pos;
		}
		++pos;
	}
	return -1;
	
}
int AASS::acg::AutoCompleteGraph::findPriorNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_prior.begin();
	for(it ; it != _nodes_prior.end() ; ++it){
		if(*it == v){
			return pos;
		}
		++pos;
	}
	return -1;
	
}

void AASS::acg::AutoCompleteGraph::addPriorGraph(const bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>& graph){
	
	std::pair< bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::VertexIterator > vp;
	std::deque<bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex> vec_deque;
	std::vector<g2o::VertexSE2Prior*> out_prior;
	
// 	std::cout << "NOOOOOOW" << std::endl << std::endl; 
	
	assert( _nodes_prior.size() == 0 );
	
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
		auto v = *vp.first;
		//ATTENTION Magic number
		
// 		std::cout << "Prior Landmark : " << graph[v].getX() << " " << graph[v].getY() << std::endl;
		
		
		g2o::VertexSE2Prior* res = addPriorLandmarkPose(graph[v].getX(), graph[v].getY(), 0);
		vec_deque.push_back(v);
		out_prior.push_back(res);
// 		_nodes_prior.push_back(res);
	}
	
	assert( _edge_prior.size() == 0);
	
	int count = 0;
	int self_link = 0 ;
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
		auto v = *vp.first;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Edge e;
		
		for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph)); 
			out_i != out_end; ++out_i) {
			e = *out_i;
// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, (graph));
			auto targ = boost::target(e, (graph));
		
			int idx = -1;
			for(size_t ii = count ; ii < vec_deque.size() ; ++ii){
				if(targ == vec_deque[ii]){
					idx = ii;
				}
			}
			if(idx == -1){
				//SKIP
// 				throw std::runtime_error("Didn't find the vertex target :(");
			}
			else if(idx == count){
				self_link = self_link + 1 ;
			}
			else{
				
				assert(idx != count);
				
				g2o::VertexSE2Prior* from = out_prior[count]; //<-Base
				g2o::VertexSE2Prior* toward = out_prior[idx]; //<-Targ
				
				double x_diff = graph[targ].getX() - graph[v].getX();
				double y_diff = graph[targ].getY() - graph[v].getY();
				
				std::cout << "Index" << idx << " " <<count << std::endl;
				
				std::cout << "Because : " << graph[targ].getX() << " - " << graph[v].getX() << " and " << graph[targ].getY() << " - " << graph[v].getY() << std::endl;
				
// 				std::cout << "diff: " <<x_diff << " " << y_diff << std::endl;
				
				g2o::SE2 se2(x_diff, y_diff, 0);
				
				std::cout << "SE2 pushed in edge \n" << se2.toVector() << std::endl;
				
				auto edge_out = addEdgePrior(se2, from, toward);
// 				_edge_prior.push_back(edge_out);
			}
		
		}
		++count;
	}
	
	std::cout << _edge_prior.size() << " == " << graph.getNumEdges() << " - " << self_link / 2 << std::endl;
	std::cout << _nodes_prior.size() << " == " << graph.getNumVertices() << std::endl;
	
	assert( _nodes_prior.size() == graph.getNumVertices() );
	//Self link / 2 because they are seen twice
	assert( _edge_prior.size() == graph.getNumEdges() - (self_link / 2) );
	
}

void AASS::acg::AutoCompleteGraph::updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph){
	
	std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement> corners_end;
	double cell_size = 0;
	
	std::cout << "Should we check " <<ndt_graph.getNbNodes() << ">" << _previous_number_of_node_in_ndtgraph << std::endl;
	
	//************* Add and create all new nodes, and extract the corners from the new NDT maps *********//
	
	if(ndt_graph.getNbNodes() > _previous_number_of_node_in_ndtgraph){
		
		std::cout << "Found new nodes" << std::endl;
		
		//TODO just get the links that are interesting for you instead of all of them !
		//Doing the registration here myself
		size_t i;
		auto links = ndt_graph.getOdometryLinks();
		ndt_graph.updateLinksUsingNDTRegistration(links, 10, true);
		
		Eigen::Isometry2d diff;
		diff.matrix() << 1, 0, 0, 
				0, 1, 0, 
				0, 0, 0;
				
		//**************** Calculate the previous transformations if there as already been something added *** //
				
		if(_previous_number_of_node_in_ndtgraph != 0){
			auto node = _nodes_ndt[_nodes_ndt.size() - 1].getNode();
			auto shift = node->estimate().toIsometry();
			
	// 		double angle = atan2(shift.rotation()(1,0), shift.rotation()(0,0));//rot.angle();//acosa2d.rotation()(0,1)/a2d.rotation()(0,0);
	// 		auto shift3d = Eigen::Translation3d(shift.translation()(0), shift.translation()(1), 0.) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
			auto getRobustYawFromAffine3d = [](const Eigen::Affine3d &a) -> double {
				// To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
				Eigen::Vector3d v1(1,0,0);
				Eigen::Vector3d v2 = a.rotation()*v1;
				double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
				double angle = acos(dot);
				// Need to find the sign
				if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
					return angle;
				return -angle;
			};
			
			auto original3d = _nodes_ndt[_nodes_ndt.size() - 1].getPose();
			auto original2d = Eigen::Translation2d(original3d.translation().topRows<2>()) * Eigen::Rotation2D<double>(getRobustYawFromAffine3d(original3d));
			
// 			Eigen::Affine2d original2d;
// 			original2d.matrix() << original3d(0, 0), original3d(0, 1), original3d(0, 3),
// 								   original3d(1, 0), original3d(1, 1), original3d(1, 3),
// 												  0,                0, original3d(2, 3);


			
			auto diff_affine = original2d.inverse() * shift;
			diff.translation() = diff_affine.translation();
			diff.linear() = diff_affine.rotation();
		}
		//Calculate the original transformation of all 
		
		//Assume that all node in the NDT graph must been analysed
		i = _previous_number_of_node_in_ndtgraph;
		
		for (i; i < ndt_graph.getNbNodes(); ++i) {
			
			std::cout << "Checking node nb " << i << std::endl;
			//RObot pose
				ndt_feature::NDTFeatureNode* feature = new ndt_feature::NDTFeatureNode();
				std::cout << "Copy feature" << std::endl;
				feature->copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
			Eigen::Affine3d affine = Eigen::Affine3d(feature->getPose());
			Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
			
			//TODO make this work
			isometry2d =  diff * isometry2d;
			
			g2o::SE2 robot_pos(isometry2d);
			double resolution = feature->map->params_.resolution;
				delete feature;
// 			Eigen::Vector2d robot_pos; robot_pos << robot_pos_tmp(0), robot_pos_tmp(1);
// 			robot_pos << ndt_graph.getNode(i).T(0), ndt_graph.getNode(i).T(1);
			
			//ATTENTION THIS GETS FORGOTTEN
			lslgeneric::NDTMap* map = ndt_graph.getMap(i);
			
			//Use a a msg to copy to a new pointer so it doesn't get forgotten :|
			ndt_map::NDTMapMsg msg;
			//ATTENTION Frame shouldn't be fixed
			bool good = lslgeneric::toMessage(map, msg, "/world");
			lslgeneric::NDTMap* map_copy = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
			lslgeneric::LazyGrid *lz = dynamic_cast<lslgeneric::LazyGrid*>(map_copy->getMyIndex() );
			std::string frame;
			bool good2 = lslgeneric::fromMessage(lz, map_copy, msg, frame);
			
			
			g2o::VertexSE2* robot_ptr = addRobotPose(robot_pos, affine, map_copy);
			//Add Odometry if there is more than one node
			if(i > 0 ){
				std::cout << "adding the odometry" << std::endl;
				g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[i - 1]);
				std::cout << " ref " << links[i-1].getRefIdx() << " and mov " << links[i-1].getMovIdx() << std::endl;
				assert( links[i-1].getRefIdx() < _nodes_ndt.size() );
				assert( links[i-1].getMovIdx() < _nodes_ndt.size() );
				auto from = _nodes_ndt[ links[i-1].getRefIdx() ] ;
				auto toward = _nodes_ndt[ links[i-1].getMovIdx() ] ;
				
				std::cout << "Saving cov " << std::endl;
				//TODO : transpose to 3d and use in odometry!
				Eigen::MatrixXd cov = links[i - 1].cov_3d;
				
				std::cout << "Saving cov to 2d" << std::endl;
				Eigen::Matrix3d cov_2d;
				cov_2d << cov(0, 0), cov(0, 1), 0,
						  cov(1, 0), cov(1, 1), 0,
						  0, 		 0, 		cov(5, 5);
						  
				std::cout << "Saving information " << std::endl;
				Eigen::Matrix3d information = cov_2d.inverse();
				
				std::cout << "Saving odometry " << std::endl;
				addOdometry(odometry, from.getNode(), toward.getNode(), information);
			}
			
			
			//********************** Extract the corners *****************//
			
			//HACK For now : we translate the Corner extracted and not the ndt-maps
			auto cells = map->getAllCells();
			double x2, y2, z2;
			map->getCellSizeInMeters(x2, y2, z2);
			cell_size = x2;
			
			AASS::das::NDTCorner cornersExtractor;
			auto ret_export = cornersExtractor.getAllCorners(*map);
			auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();		
			
			
			
			auto it = ret_opencv_point_corner.begin();
			
			std::cout << "Found " << ret_opencv_point_corner.size() << " corners " << std::endl;
			
			//Find all the observations :
			//**************** HACK: translate the corners now : **************//
			for(it ; it != ret_opencv_point_corner.end() ; ++it){
// 						std::cout << "MOVE : "<< it -> x << " " << it-> y << std::endl;
				Eigen::Vector3d vec;
				vec << it->x, it->y, 0;
				
				//Wrong because we need to take in account the orientation
// 				Eigen::Vector3d vec;
// 				vec << it->x, it->y, 0;
				
				
				Eigen::Vector3d vec_out = ndt_graph.getNode(i).T * vec;
				cv::Point2f p_out(vec_out(0), vec_out(1));
				
// 				g2o::Vector2D observation;
// 				observation << it->x - p_out.x, it->y  - p_out.y ;
				
				Eigen::Vector2d real_obs; real_obs << p_out.x, p_out.y;
				Eigen::Vector2d observation;
				//Projecting real_obs into robot coordinate frame
				Eigen::Vector2d trueObservation = robot_pos.inverse() * real_obs;
				observation = trueObservation;

				
				std::cout << "Node transfo " << ndt_graph.getNode(i).T.matrix() << std::endl;
				std::cout << "Position node " << robot_pos.toVector() << std::endl;
				std::cout << " vec " << vec << std::endl;
				std::cout << "Well " << robot_pos.toVector() + vec << "==" << ndt_graph.getNode(i).T * vec << std::endl;
				
				//ATTENTION THIS IS NOT TRUE BUT REALLY CLOSE
// 				assert (robot_pos + vec == ndt_graph.getNode(i).T * vec);
				
// 				std::cout << "NEW POINT : "<< p_out << std::endl;
				
				NDTCornerGraphElement cor(p_out);
				cor.addAllObserv(i, robot_ptr, observation);
				corners_end.push_back(cor);
// 						cor.addNode(i);
// 						cor.nodes_linked_ptr.push_back(robot_ptr);
// 						cor.addObservation(observation);
				
			}
			//At this point, we have all the corners
		}
		//Save new number of nodes to update
		
		
	}		
	
	std::vector<g2o::VertexPointXY*> all_new_landmarks;
	
	// ************ Add the landmarks that were added the corners_end ************************************//
	
	//Add stuff directly in the optimization graph : 
	for(size_t i = 0 ; i < corners_end.size() ; ++i){
		std::cout << "checking corner : " ;  corners_end[i].print() ; std::cout << std::endl;	
		bool seen = false;
		g2o::VertexPointXY* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2D landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));
			
			double res = cv::norm(point_land - corners_end[i].point);
			
// 			std::cout << "res : " << res << " points "  << point_land << " " << corners_end[i].point << "  cell size " << cell_size << std::endl;
			
			//If we found the landmark, we save the data
			if( res < cell_size){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}
		if(seen == false){
			std::cout << "New point" << std::endl;
			g2o::Vector2D vec;
			vec << corners_end[i].point.x, corners_end[i].point.y ;
			g2o::VertexPointXY* ptr = addLandmarkPose(vec, 1);
// 			g2o::VertexPointXY* ptr;
			all_new_landmarks.push_back(ptr);
			//Adding all links
			for(int no = 0 ; no < corners_end[i].getNodeLinked().size() ; ++no){
// 						addLandmarkObservation(corners_end[i].getObservations()[no], _nodes_ndt[corners_end[i].getNodeLinked()[no], ptr);
				addLandmarkObservation(corners_end[i].getObservations()[no], corners_end[i].getNodeLinkedPtr()[no], ptr);
			}
		}
		else{
			std::cout << "Point seen " << std::endl;
			for(int no = 0 ; no < corners_end[i].getNodeLinked().size() ; ++no){
				addLandmarkObservation(corners_end[i].getObservations()[no], corners_end[i].getNodeLinkedPtr()[no], ptr_landmark_seen);
			}				
		}
	}
	_previous_number_of_node_in_ndtgraph = ndt_graph.getNbNodes();
	
	
	//TODO, update links using the newly found landmarks. No need to update the rest obviously (<-HAHA that was so wrrrrongggg need to update it since they moved :P!)
	
	updateLinksAfterNDTGraph(all_new_landmarks);

	
}


void AASS::acg::AutoCompleteGraph::updateLinksAfterNDTGraph(const std::vector<g2o::VertexPointXY*>& new_landmarks)
{
	std::vector < std::pair < g2o::VertexPointXY*, g2o::VertexSE2Prior*> > links;
	
	std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
	std::cout << "Prior " << _nodes_prior.size() << std::endl;
// 	if(_nodes_prior.size() > 0){
		
	//Update ALL links
	auto it = _nodes_landmark.begin();
	for(it ; it != _nodes_landmark.end() ; it++){
		std::cout << "Working on links " << std::endl;
		Eigen::Vector2d pose_landmark = (*it)->estimate();
		auto it_prior = _nodes_prior.begin();
		
		Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
		Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
		
		double norm = (pose_prior - pose_landmark).norm();
		g2o::VertexSE2Prior* ptr_closest = *it_prior;
		
		for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){
			pose_tmp = (*it_prior)->estimate().toVector();
			pose_prior << pose_tmp(0), pose_tmp(1);
			double norm_tmp = (pose_prior - pose_landmark).norm();
			
			std::cout << "NORM" << norm_tmp << "min dist " << _min_distance_for_link_in_meter << std::endl;
			
			//Update the link
			if(norm_tmp < _min_distance_for_link_in_meter){
				ptr_closest = *it_prior;
				norm = norm_tmp;
				//Pushing the link
				std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
				links.push_back(std::pair<g2o::VertexPointXY*, g2o::VertexSE2Prior*>(*it, ptr_closest));
			}					
		}
		//Pushing the link
// 			std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
// 			links.push_back(std::pair<g2o::VertexPointXY*, g2o::VertexSE2Prior*>(*it, ptr_closest));
	}
	
	std::cout << "\n";
	std::cout << "Adding the links" << std::endl;
	
	auto it_links = links.begin();
	for(it_links ; it_links != links.end() ; it_links++){
		g2o::Vector2D vec;
		vec << 0, 0;
		std::cout << "Creating " << it_links->second << " and " << it_links->first << std::endl;
		addLinkBetweenMaps(vec, it_links->second, it_links->first);
	}
// 	}
	
	
	//Remove links that went too far away from the points and restor the edges to original state when possible:
	
	auto it_old_links = _edge_link.begin();
	for(it_old_links; it_old_links != _edge_link.end() ;){
		
		std::vector<Eigen::Vector3d> vertex_out;
		
		for(auto ite2 = (*it_old_links)->vertices().begin(); ite2 != (*it_old_links)->vertices().end() ; ++ite2){
			g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
			g2o::VertexPointXY* ptr2 = dynamic_cast<g2o::VertexPointXY*>((*ite2));

			if(ptr != NULL){
				std::cout << "Got a VertexSE2" << std::endl;
				auto vertex = ptr->estimate().toVector();
				vertex_out.push_back(vertex);
			}
			else if(ptr2 != NULL){
				std::cout << "Got a VertexPOINTXY" << std::endl;
				auto vertex = ptr2->estimate();
				Eigen::Vector3d pose_prior; pose_prior << vertex(0), vertex(1), 0;
				vertex_out.push_back(pose_prior);
			}
			else{
				throw std::runtime_error("Links do not have the good vertex type");
			}		
			
		}
		
		assert(vertex_out.size() == 2);
		double norm = (vertex_out[0] - vertex_out[1]).norm();
		if(norm >= _min_distance_for_link_in_meter * 2 ){
			it_old_links ++;
			removeLinkBetweenMaps(*it_old_links);
		}
		else{
			it_old_links++;
		}
	}
	
	
}


void AASS::acg::AutoCompleteGraph::updatePriorEdgeCovariance()
{
	auto edges = _edge_prior;	
	auto it = edges.begin();
	for(it ; it != edges.end() ; ++it){
		g2o::VertexSE2Prior* v_ptr = dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[0]);
		if(v_ptr == NULL){
			std::runtime_error("no");
		}
		g2o::VertexSE2Prior* v_ptr2 = dynamic_cast<g2o::VertexSE2Prior*>((*it)->vertices()[1]);
		if(v_ptr2 == NULL){
			std::runtime_error("no2");
		}
		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
		Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
			
		// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
		// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;
			
		Eigen::Vector2d eigenvec; 
		eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
		
		double newnorm = (pose1 - pose2).norm();
		std::cout << "new norm " << newnorm << " because " << pose1 << " " << pose2 << std::endl;
		g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
		Eigen::Vector3d vecold = oldnormse2.toVector();
		double oldnorm = (vecold).norm();
		std::cout << "oldnorm" << newnorm << std::endl;
		assert(oldnorm >= 0);
		
		double diff_norm = std::abs(oldnorm - newnorm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		if(diff_norm > oldnorm){
			diff_norm = oldnorm;
		}
		diff_norm = std::abs(diff_norm);
		std::cout << "Diff norm " << diff_norm << std::endl;
		assert(diff_norm >= 0);
		assert(diff_norm <= oldnorm);
		double normalizer_own = 1 / oldnorm;
		double diff_norm_normalized = 1 - (diff_norm * normalizer_own);
		
		double new_cov = diff_norm_normalized;
		
		assert(new_cov <= _priorNoise(0));
		assert(new_cov >= 0);
	// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
		std::pair<double, double> eigenval(new_cov, _priorNoise(1));
		
		Eigen::Matrix2d cov = getCovarianceVec(eigenvec, eigenval);
		
	// 			std::cout << "Covariance prior " << std::endl << cov.format(cleanFmt) << std::endl;
		
		Eigen::Matrix3d covariance_prior;
		covariance_prior.fill(0.);
		covariance_prior(0, 0) = cov(0, 0);
		covariance_prior(0, 1) = cov(0, 1);
		covariance_prior(1, 0) = cov(1, 0);
		covariance_prior(1, 1) = cov(1, 1);
	// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
		covariance_prior(2, 2) = _prior_rot * _prior_rot;
		Eigen::Matrix3d information_prior = covariance_prior.inverse();
		(*it)->setInformation(information_prior);
		
	}
	
	
	
	
}

