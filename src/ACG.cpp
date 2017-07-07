#include "auto_complete_graph/ACG.hpp"

Eigen::Vector2d AASS::acg::EdgeSE2Prior_malcolm::getDirection2D(const AASS::acg::VertexSE2Prior& from) const
{
// 	std::cout << "From " << from.estimate().toVector(); 
	AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>(this->vertices()[0]);
	AASS::acg::VertexSE2Prior* ptr2 = dynamic_cast<AASS::acg::VertexSE2Prior*>(this->vertices()[1]);
	assert(ptr != NULL);
	assert(ptr2 != NULL);
	assert(ptr == &from || ptr2 == &from);
	Eigen::Vector3d from_1;
	Eigen::Vector3d toward;
	if(ptr == &from){
		from_1 = ptr->estimate().toVector();
		toward = ptr2->estimate().toVector();
	}
	else if(ptr2 == &from){
		from_1 = ptr2->estimate().toVector();
		toward = ptr->estimate().toVector();
	}
	else{
		assert(true == false && "Weird, the original vertex wasn't found before");
	}
	
// 	std::cout << " toward " << toward << std::endl;
	
	Eigen::Vector3d pose_from1 = toward - from_1;
	Eigen::Vector2d pose_prior; pose_prior << pose_from1(0), pose_from1(1);
	return pose_prior;
	
}



AASS::acg::VertexSE2RobotPose* AASS::acg::AutoCompleteGraph::addRobotPose(const g2o::SE2& se2, const Eigen::Affine3d& affine, const std::shared_ptr< lslgeneric::NDTMap >& map){
	
	std::cout << "Adding the robot pose " << std::endl;
	AASS::acg::VertexSE2RobotPose* robot =  new AASS::acg::VertexSE2RobotPose();

	robot->setEstimate(se2);
	robot->setId(new_id_);
	++new_id_;
	
	robot->setMap(map);
	robot->setPose(affine);
	robot->initial_transfo = robot->estimate();
	
	_optimizable_graph.addVertex(robot);
	
// 	NDTNodeAndMap nodeAndMap(robot, map, affine);
	
	_nodes_ndt.push_back(robot);
	return robot;
}
AASS::acg::VertexSE2RobotPose* AASS::acg::AutoCompleteGraph::addRobotPose(const Eigen::Vector3d& rob, const Eigen::Affine3d& affine, const std::shared_ptr< lslgeneric::NDTMap >& map){
	g2o::SE2 se2(rob(0), rob(1), rob(2));
	return addRobotPose(se2, affine, map);
}
AASS::acg::VertexSE2RobotPose* AASS::acg::AutoCompleteGraph::addRobotPose(double x, double y, double theta, const Eigen::Affine3d& affine, const std::shared_ptr< lslgeneric::NDTMap >& map){
	Eigen::Vector3d robot1;
	robot1 << x, y, theta;
	return addRobotPose(robot1, affine, map);
}

AASS::acg::VertexLandmarkNDT* AASS::acg::AutoCompleteGraph::addLandmarkPose(const g2o::Vector2D& pos, const cv::Point2f& pos_img, int strength){
	AASS::acg::VertexLandmarkNDT* landmark = new AASS::acg::VertexLandmarkNDT();
	landmark->setId(new_id_);
	++new_id_;
	landmark->setEstimate(pos);
	landmark->position = pos_img;
	_optimizable_graph.addVertex(landmark);
	_nodes_landmark.push_back(landmark);
	return landmark;
}
AASS::acg::VertexLandmarkNDT* AASS::acg::AutoCompleteGraph::addLandmarkPose(double x, double y, const cv::Point2f& pos_img, int strength){
	g2o::Vector2D lan;
	lan << x, y;
	return addLandmarkPose(lan, pos_img, strength);
}

AASS::acg::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(const g2o::SE2& se2, const PriorAttr& priorAttr){
	AASS::acg::VertexSE2Prior* priorlandmark = new AASS::acg::VertexSE2Prior();
	priorlandmark->setId(new_id_);
	++new_id_;
	priorlandmark->setEstimate(se2);
	priorlandmark->priorattr = priorAttr;
	
	_optimizable_graph.addVertex(priorlandmark);
	_nodes_prior.push_back(priorlandmark);
	return priorlandmark;
}
AASS::acg::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(const Eigen::Vector3d& lan, const PriorAttr& priorAttr){
	g2o::SE2 se2(lan(0), lan(1), lan(2));
	return addPriorLandmarkPose(se2, priorAttr);
}
AASS::acg::VertexSE2Prior* AASS::acg::AutoCompleteGraph::addPriorLandmarkPose(double x, double y, double theta, const AASS::acg::PriorAttr& priorAttr){
	Eigen::Vector3d lan;
	lan << x, y, theta;
	return addPriorLandmarkPose(lan, priorAttr);
}


g2o::EdgeOdometry_malcolm* AASS::acg::AutoCompleteGraph::addOdometry(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2, const Eigen::Matrix3d& information_tmp){
	
	Eigen::Matrix3d information;
	if(_use_user_robot_pose_cov == true){
		Eigen::Matrix3d covariance_robot;
		covariance_robot.fill(0.);
		covariance_robot(0, 0) = _transNoise[0]*_transNoise[0];
		covariance_robot(1, 1) = _transNoise[1]*_transNoise[1];
	// 	covariance_prior(2, 2) = 13;//<- Rotation covariance prior landmark is more than 4PI
		covariance_robot(2, 2) = _rotNoise * _rotNoise;
		information = covariance_robot.inverse();
	}
	else{
		 information = information_tmp;
	}
	
	assert(information.isZero(1e-10) == false);
	
	g2o::EdgeOdometry_malcolm* odometry = new g2o::EdgeOdometry_malcolm;
	odometry->vertices()[0] = v1 ;
	odometry->vertices()[1] = v2 ;
	odometry->setMeasurement(se2);
	odometry->setInformation(information);
	
// 	odometry->interface.setAge(_age_start_value);
	
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
	
	assert(information_landmark.isZero(1e-10) == false);
	
	landmarkObservation->setInformation(information_landmark);
	landmarkObservation->setParameterId(0, _sensorOffset->id());
	
	
// 	landmarkObservation->interface.setAge(_age_start_value);
	
	_optimizable_graph.addEdge(landmarkObservation);
	_edge_landmark.push_back(landmarkObservation);
	
	return landmarkObservation;
}
g2o::EdgeLandmark_malcolm* AASS::acg::AutoCompleteGraph::addLandmarkObservation(const g2o::Vector2D& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	return addLandmarkObservation(pos, from_ptr, toward_ptr);
}

AASS::acg::EdgeSE2Prior_malcolm* AASS::acg::AutoCompleteGraph::addEdgePrior(const g2o::SE2& se2, g2o::HyperGraph::Vertex* v1, g2o::HyperGraph::Vertex* v2){
	
	
	//Get Eigen vector
	AASS::acg::VertexSE2Prior* v_ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>(v1);
	AASS::acg::VertexSE2Prior* v_ptr2 = dynamic_cast<AASS::acg::VertexSE2Prior*>(v2);
	Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
	Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
	
// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;
	
	Eigen::Vector2d eigenvec; 
	eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
	double newnorm_old = (pose1 - pose2).norm();
	
// 	double test_newnorm = newnorm_old / 2;
	//ATTENTION NOT A MAGIC NUMBER
	double newnorm = (newnorm_old *  _priorNoise(0)) / 100;
	
	int test_tt = newnorm * 100;
	int test_ttt = newnorm_old * 100;
// 	std::cout << newnorm << " " << test_ttt << " test " << test_tt << " base " << _priorNoise(0) << " "<< _priorNoise(1) << std::endl;
	
	assert(test_tt <= test_ttt);
	assert(newnorm >= 0);
	
	std::pair<double, double> eigenval(newnorm, _priorNoise(1));
// 	std::pair<double, double> eigenval(_priorNoise(0), _priorNoise(1));
	
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
	
	assert(information_prior.isZero(1e-10) == false);
	
	AASS::acg::EdgeSE2Prior_malcolm* priorObservation =  new AASS::acg::EdgeSE2Prior_malcolm;
	priorObservation->vertices()[0] = v1;
	priorObservation->vertices()[1] = v2;
	priorObservation->setMeasurement(se2);
	priorObservation->setInformation(information_prior);
	priorObservation->setParameterId(0, _sensorOffset->id());

// 	priorObservation->interface.setAge(_age_start_value);
// 	priorObservation->interface.setOriginalValue(se2);
	
	_optimizable_graph.addEdge(priorObservation);
	
// 	EdgePriorAndInitialValue epiv(priorObservation, se2);
	_edge_prior.push_back(priorObservation);
	
	std::cout << "After adding an edge" << std::endl;
	checkNoRepeatingPriorEdge();
	
	return priorObservation;
}


g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraph::addLinkBetweenMaps(const g2o::Vector2D& pos, AASS::acg::VertexSE2Prior* v2, AASS::acg::VertexLandmarkNDT* v1){
	std::cout << "Adding link" << std::endl;

	
	Eigen::Matrix2d covariance_link; 
	covariance_link.fill(0.);
	covariance_link(0, 0) = _linkNoise[0]*_linkNoise[0];
	covariance_link(1, 1) = _linkNoise[1]*_linkNoise[1];
	
// 	std::cout << "Link cov " << covariance_link << std::endl;
	
// 			covariance_link(2, 2) = 13;//<- Rotation covariance link is more than 4PI
	Eigen::Matrix2d information_link = covariance_link.inverse();
	
	assert(information_link.isZero(1e-10) == false);
// 	std::cout << "Link cov2 " << covariance_link << std::endl;
	
	g2o::EdgeLinkXY_malcolm* linkObservation = new g2o::EdgeLinkXY_malcolm;
	
// 	std::cout << "Link cov3 " << covariance_link << std::endl;
// 	g2o::HyperGraph::Vertex* from = dynamic_cast<g2o::HyperGraph::Vertex*>(v2);
	assert(v2 != NULL);
	linkObservation->vertices()[0] = v2;
	assert(linkObservation->vertices()[0] == v2);
// 	std::cout << "Link cov4 " << covariance_link << std::endl;
// 	g2o::HyperGraph::Vertex* toward = dynamic_cast<g2o::HyperGraph::Vertex*>(v1);
	assert(v1 != NULL);
	linkObservation->vertices()[1] = v1;
	assert(linkObservation->vertices()[1] == v1);
// 	std::cout << "Link cov5 " << covariance_link << std::endl;
	linkObservation->setMeasurement(pos);
// 	std::cout << "Link cov6 " << covariance_link << std::endl;
	linkObservation->setInformation(information_link);
// 	std::cout << "Link cov7 " << _sensorOffset->id() << std::endl;
	linkObservation->setParameterId(0, _sensorOffset->id());
	
	//HACK REPLACEMENT
	EdgeInterface edgeinter;
// 	std::cout << "Setting age value" << std::endl;
// 	std::cout << "Setting age value" << std::endl;
	edgeinter.setAge(_age_start_value);
// 	std::cout << "Manueal" << std::endl;
	assert(edgeinter.manuallySetAge() == true);
	
	_optimizable_graph.addEdge(linkObservation);
	_edge_link.push_back(linkObservation);
	_edge_interface_of_links.push_back(edgeinter);
	
	assert(_edge_interface_of_links.size() == _edge_link.size());
	
// 	std::cout << "Done" << std::endl;
	return linkObservation;
}
g2o::EdgeLinkXY_malcolm* AASS::acg::AutoCompleteGraph::addLinkBetweenMaps(const g2o::Vector2D& pos, int from_id, int toward_id){
	g2o::HyperGraph::Vertex* from_ptr = _optimizable_graph.vertex(from_id);
	g2o::HyperGraph::Vertex* toward_ptr = _optimizable_graph.vertex(toward_id);
	
	AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>(from_ptr);
	AASS::acg::VertexLandmarkNDT* ptr2;
	if(ptr != NULL){
		AASS::acg::VertexLandmarkNDT* ptr2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>(toward_ptr);
		if(ptr2 == NULL){
			throw std::runtime_error("Pointers are not of compatible type. First pointer should be prior while the second is not a PointXY while it should");
		}
	}
	return addLinkBetweenMaps(pos, ptr, ptr2);
}


std::vector <g2o::EdgeLinkXY_malcolm* >::iterator AASS::acg::AutoCompleteGraph::removeLinkBetweenMaps(g2o::EdgeLinkXY_malcolm* v1)
{
	
	std::vector <g2o::EdgeLinkXY_malcolm* >::iterator next_el;
	auto it = _edge_link.begin();
	int size = _edge_link.size();
	int place = 0;
	for(it; it != _edge_link.end() ;){
		if(*it == v1){
			std::cout << "Found"<<std::endl;
			next_el = _edge_link.erase(it);
			break;
		}
		else{
			place++;
			it++;
		}
	}
	
	assert(place != size);
	_edge_interface_of_links.erase(_edge_interface_of_links.begin() + place);
	
	_optimizable_graph.removeEdge(v1);
	assert(_edge_link.size() == size - 1);
	assert(_edge_interface_of_links.size() == _edge_link.size());
	
	std::cout << "Out of remove edge" << std::endl;
	
	return next_el;
	
}



//FUNCTION TO REMOVE A VERTEX
//TODO : remove all link edges from list
void AASS::acg::AutoCompleteGraph::removeVertex(g2o::HyperGraph::Vertex* v1){
	//Prior
	AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>(v1);
	g2o::VertexSE2* ptr_se2 = dynamic_cast<g2o::VertexSE2*>(v1);
	AASS::acg::VertexLandmarkNDT* ptr_se3 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>(v1);
	
	if(ptr != NULL){
		std::cout <<"Find prior" << std::endl;
		int index = findPriorNode(v1);
		assert(index != -1);
		std::vector<AASS::acg::VertexSE2Prior*>::iterator which = _nodes_prior.begin() + index;
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
		std::vector<AASS::acg::VertexLandmarkNDT*>::iterator which = _nodes_landmark.begin() + index;
		_nodes_landmark.erase(which);
	}
	else{
		throw std::runtime_error("Vertex type not found in list");
	}
	_optimizable_graph.removeVertex(v1, false);
}

int AASS::acg::AutoCompleteGraph::findRobotNode(g2o::HyperGraph::Vertex* v){
	int pos = 0;
	auto it = _nodes_ndt.begin();
	for(it ; it != _nodes_ndt.end() ; ++it){
		if(*it == v){
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

void AASS::acg::AutoCompleteGraph::addPriorGraph(const PriorLoaderInterface::PriorGraph& graph){
		
	std::pair< PriorLoaderInterface::PriorGraph::VertexIterator, PriorLoaderInterface::PriorGraph::VertexIterator > vp;
	std::deque<PriorLoaderInterface::PriorGraph::Vertex> vec_deque;
	std::vector<AASS::acg::VertexSE2Prior*> out_prior;
	
	std::cout << "NOOOOOOW" << _optimizable_graph.vertices().size() << std::endl << std::endl; 
	
	assert( _nodes_prior.size() == 0 );
	
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
// 		bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex v = *vp.first;
		auto v = *vp.first;
		//ATTENTION Magic number
		
// 		std::cout << "Prior Landmark : " << graph[v].getX() << " " << graph[v].getY() << std::endl;
		
		
		AASS::acg::VertexSE2Prior* res = addPriorLandmarkPose(graph[v].getX(), graph[v].getY(), 0, graph[v]);
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
		
		std::vector<PriorLoaderInterface::PriorGraph::Vertex> tmp_for_double_edges;
				
		for (boost::tie(out_i, out_end) = boost::out_edges(v, (graph)); 
			out_i != out_end; ++out_i) {
			e = *out_i;
// 			bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex targ = boost::target(e, (graph));
			auto targ = boost::target(e, (graph));
		
			//Avoiding double edge to the same vertex target but that is not a self loop!

			int idx = -1;
			for(size_t ii = count ; ii < vec_deque.size() ; ++ii){
				if(targ == vec_deque[ii]){
					idx = ii;
				}
			}
			for(auto it_tmp_double = tmp_for_double_edges.begin() ; it_tmp_double != tmp_for_double_edges.end() ; ++it_tmp_double){
				if(*it_tmp_double == targ){
					//it's a double we cancel idx and add it to the self_lopp count
					idx = count;
				}
			}
			tmp_for_double_edges.push_back(targ);
			
			if(idx == -1){
				//SKIP
// 				throw std::runtime_error("Didn't find the vertex target :(");
			}
			else if(idx == count){
				self_link = self_link + 1 ;
			}
			else{
				
				assert(idx != count);
				
				AASS::acg::VertexSE2Prior* from = out_prior[count]; //<-Base
				AASS::acg::VertexSE2Prior* toward = out_prior[idx]; //<-Targ
				
				double x_diff = graph[targ].getX() - graph[v].getX();
				double y_diff = graph[targ].getY() - graph[v].getY();
				
// 				std::cout << "Index" << idx << " " <<count << std::endl;
				
// 				std::cout << "Because : " << graph[targ].getX() << " - " << graph[v].getX() << " and " << graph[targ].getY() << " - " << graph[v].getY() << std::endl;
				
// 				std::cout << "diff: " <<x_diff << " " << y_diff << std::endl;
				
				g2o::SE2 se2(x_diff, y_diff, 0);
				
// 				std::cout << "SE2 pushed in edge \n" << se2.toVector() << std::endl;
				
				auto edge_out = addEdgePrior(se2, from, toward);
// 				_edge_prior.push_back(edge_out);
			}
		
		}
		++count;
	}
	
// 	std::cout << _edge_prior.size() << " == " << graph.getNumEdges() << " - " << self_link / 2 << std::endl;
// 	std::cout << _nodes_prior.size() << " == " << graph.getNumVertices() << std::endl;
	
	assert( _nodes_prior.size() == graph.getNumVertices() );
	//Self link / 2 because they are seen twice
	assert( _edge_prior.size() == graph.getNumEdges() - (self_link / 2) );
		
	std::cout << "After update graph prior" << std::endl;
	checkNoRepeatingPriorEdge();
	
}


Eigen::Vector3d AASS::acg::AutoCompleteGraph::getLastTransformation()
{
	Eigen::Vector3d diff_vec; diff_vec << 0, 0, 0;
	//**************** Calculate the previous transformations if there as already been something added *** //
			
// 	if(_previous_number_of_node_in_ndtgraph != 0){
	if(_nodes_ndt.size() != 0){
		auto node = _nodes_ndt[_nodes_ndt.size() - 1];
		auto original3d = _nodes_ndt[_nodes_ndt.size() - 1]->getPose();
		
		/*******' Using Vec********/
		Eigen::Isometry2d original2d_aff = Affine3d2Isometry2d(original3d);
		auto shift_vec = node->estimate().toVector();
		g2o::SE2 se2_tmptmp(original2d_aff);
		auto original3d_vec = se2_tmptmp.toVector();
		diff_vec = original3d_vec - shift_vec;
		
	}

	return diff_vec;
}



std::shared_ptr<lslgeneric::NDTMap> AASS::acg::AutoCompleteGraph::addElementNDT(ndt_feature::NDTFeatureGraph& ndt_graph, const std::vector< ndt_feature::NDTFeatureLink >& links, int element, double deviation, AASS::acg::VertexSE2RobotPose** robot_ptr, g2o::SE2& robot_pos)
{
	
	//Return Gaussian white noise
	auto randomNoise = [](double mean, double deviationt) -> double {
		std::default_random_engine engine{std::random_device()() };
		std::normal_distribution<double> dist(mean, deviationt);
		return dist(engine);
	};
	
	Eigen::Vector3d diff_vec = getLastTransformation();
	
	//Calculate noise
	auto noise_x = randomNoise(0, deviation);
	auto noise_y = randomNoise(0, deviation);
	auto noise_t = randomNoise(0, deviation);
	g2o::SE2 noise_se2(noise_x, noise_y, noise_t);

	std::cout << "Checking node nb " << element << std::endl;
	
	//RObot pose
// 			ndt_feature::NDTFeatureNode& feature = dynamic_cast<ndt_feature::NDTFeatureNode&>(ndt_graph.getNodeInterface(i));
	std::cout << "Copy feature" << std::endl;
	
// 			feature.copyNDTFeatureNode( (const ndt_feature::NDTFeatureNode&)ndt_graph.getNodeInterface(i) );
	Eigen::Affine3d affine = Eigen::Affine3d(ndt_graph.getNodeInterface(element).getPose());
	Eigen::Isometry2d isometry2d = Affine3d2Isometry2d(affine);
	
	//TODO make this work
// 			isometry2d =  diff * isometry2d;
	
	//BUG IS AFTER
	std::cout << "print" << std::endl;
	
	robot_pos = g2o::SE2(isometry2d);
// 			std::cout << "robot pose done : " << isometry2d.matrix() << std::endl;
	g2o::SE2 diff_vec_se2(diff_vec);
// 			std::cout << "diff vec done" << diff_vec << std::endl;
	robot_pos = robot_pos * diff_vec_se2;
	std::cout << "multiply" << std::endl;
		
	lslgeneric::NDTMap* map = ndt_graph.getMap(element);
		
	std::cout << "get res" << std::endl;
// 			double resolution = dynamic_cast<ndt_feature::NDTFeatureNode&>( ndt_graph.getNodeInterface(i) ).map->params_.resolution;
// 			Use a a msg to copy to a new pointer so it doesn't get forgotten :|
	ndt_map::NDTMapMsg msg;
// 			ATTENTION Frame shouldn't be fixed
	bool good = lslgeneric::toMessage(map, msg, "/world");
	lslgeneric::NDTMap* map_copy;
	lslgeneric::LazyGrid* lz;
	std::string frame;
	bool good2 = lslgeneric::fromMessage(lz, map_copy, msg, frame);
	std::shared_ptr<lslgeneric::NDTMap> shared_map(map_copy);

	*robot_ptr = addRobotPose(robot_pos, affine, shared_map);
	assert(*robot_ptr != NULL);
	//Add Odometry if it is not the first node
	if(element > 0 ){
		std::cout << "adding the odometry" << std::endl;
		
		g2o::SE2 odometry = NDTFeatureLink2EdgeSE2(links[element - 1]);
		std::cout << " ref " << links[element-1].getRefIdx() << " and mov " << links[element-1].getMovIdx() << std::endl;
		
		assert( links[element-1].getRefIdx() < _nodes_ndt.size() );
		assert( links[element-1].getMovIdx() < _nodes_ndt.size() );
		
		auto from = _nodes_ndt[ links[element-1].getRefIdx() ] ;
		auto toward = _nodes_ndt[ links[element-1].getMovIdx() ] ;
		
		std::cout << "Saving cov " << std::endl;
		//TODO : transpose to 3d and use in odometry!
		Eigen::MatrixXd cov = links[element - 1].cov_3d;
		
		std::cout << "COV " << cov << std::endl;
		
		std::cout << "Saving cov to 2d" << std::endl;
		Eigen::Matrix3d cov_2d;
		cov_2d << 	cov(0, 0), 	cov(0, 1), 	0,
					cov(1, 0), 	cov(1, 1), 	0,
					0, 		 	0, 			cov(5, 5);
					
		std::cout << "Saving information " << std::endl;
		Eigen::Matrix3d information = cov_2d.inverse();
		
// 				if(noise_flag = true && i != 0){
// 					odometry = odometry * noise_se2;
// 				}
		
		std::cout << "Saving odometry " << std::endl;
		addOdometry(odometry, from, toward, information);
	}
	
	return shared_map;

}


void AASS::acg::AutoCompleteGraph::extractCornerNDTMap(const std::shared_ptr<lslgeneric::NDTMap>& map, AASS::acg::VertexSE2RobotPose* robot_ptr, const g2o::SE2& robot_pos)
{
	
	std::vector<AASS::acg::AutoCompleteGraph::NDTCornerGraphElement> corners_end;
			
	//HACK For now : we translate the Corner extracted and not the ndt-maps
	auto cells = map->getAllCellsShared();
	std::cout << "got all cell shared" << std::endl;
	double x2, y2, z2;
	map->getCellSizeInMeters(x2, y2, z2);
	std::cout << "got all cell sized" << std::endl;
	double cell_size = x2;
	
	perception_oru::ndt_feature_finder::NDTCorner cornersExtractor;
	std::cout << "hopidy" << std::endl;
	auto ret_export = cornersExtractor.getAllCorners(*map);
	std::cout << "gotall corner" << std::endl;
	auto ret_opencv_point_corner = cornersExtractor.getAccurateCvCorners();	
	std::cout << "got all accurate corners" << std::endl;	
	auto angles = cornersExtractor.getAngles();
// 	std::cout << "got all angles" << std::endl;
	
	auto it = ret_opencv_point_corner.begin();
	
	std::cout << "Found " << ret_opencv_point_corner.size() << " corners " << std::endl;			
	//Find all the observations :
	
	//**************** HACK: translate the corners now : **************//
	
	int count_tmp = 0;
	for(it ; it != ret_opencv_point_corner.end() ; ++it){
// 						std::cout << "MOVE : "<< it -> x << " " << it-> y << std::endl;
		Eigen::Vector3d vec;
		vec << it->x, it->y, angles[count_tmp].second;
		
// 		auto vec_out_se2 = _nodes_ndt[i]->estimate();

		//Calculate obstacle position in global coordinates.
		
		//Node it beong to :
		g2o::SE2 vec_out_se2 = g2o::SE2(robot_pos);
		//Pose of landmajr
		g2o::SE2 se2_tmp(vec);
		//Composition
		vec_out_se2 = vec_out_se2 * se2_tmp;
		Eigen::Vector3d vec_out = vec_out_se2.toVector();
		
		//Pose of landmark in global reference
		cv::Point2f p_out(vec_out(0), vec_out(1));
				
		Eigen::Vector2d real_obs; real_obs << p_out.x, p_out.y;
		Eigen::Vector2d observation2d_test;
		//Projecting real_obs into robot coordinate frame
		
		auto trueObservation_tmp = robot_pos.inverse() * vec_out_se2;
		Eigen::Vector3d trueObservation = trueObservation_tmp.toVector();
		Eigen::Vector2d observation; observation << trueObservation(0), trueObservation(1);
		
		//***************************Old version for testing***************************//
		Eigen::Vector2d trueObservation2d = robot_pos.inverse() * real_obs;
// 		std::cout << trueObservation(0) << " == " << trueObservation2d(0) << std::endl;
// 		assert(trueObservation(0) == trueObservation2d(0));
// 		std::cout << trueObservation(1) << " == " << trueObservation2d(1) << std::endl;
// 		assert(trueObservation(1) == trueObservation2d(1));
		observation2d_test = trueObservation2d;
// 		std::cout << observation(0) << " == " << observation2d_test(0) << " minus " << observation(0) - observation2d_test(0) << std::endl;
// 		assert(trueObservation(0) == trueObservation2d(0));
// 		std::cout << observation(1) << " == " << observation2d_test(1) << " minus " << observation(1) - observation2d_test(1) << std::endl;
// 		int a ;
// 		std::cin >> a;
		//******************************************************************************//
		
		double angle_landmark = trueObservation(2);
		
// 		std::cout << "Node transfo " << ndt_graph.getNode(i).T.matrix() << std::endl;
		std::cout << "Position node " << robot_pos.toVector() << std::endl;
		std::cout << " vec " << vec << std::endl;
// 		std::cout << "Well " << robot_pos.toVector() + vec << "==" << ndt_graph.getNode(i).T * vec << std::endl;
		
		//ATTENTION THIS IS NOT TRUE BUT REALLY CLOSE
// 				assert (robot_pos + vec == ndt_graph.getNode(i).T * vec);
		
// 				std::cout << "NEW POINT : "<< p_out << std::endl;
		
		NDTCornerGraphElement cor(p_out);
		cor.addAllObserv(robot_ptr, observation, angle_landmark, angles[count_tmp].first);
		corners_end.push_back(cor);
		count_tmp++;
		
	}
	//At this point, we have all the corners
// 	assert(corners_end.size() - c_size == ret_opencv_point_corner_tmp.size());
	assert(corners_end.size() == ret_opencv_point_corner.size());
// 	assert(corners_end.size() - c_size == angles_tmp.size());
// 	assert(corners_end.size() == angles.size());
// 	c_size = corners_end.size();
	
	/***************** ADD THE CORNERS INTO THE GRAPH***********************/
	
	for(size_t i = 0 ; i < corners_end.size() ; ++i){
// 		std::cout << "checking corner : " << _nodes_landmark.size() << std::endl ;  /*corners_end[i].print()*/ ; std::cout << std::endl;	
		bool seen = false;
		AASS::acg::VertexLandmarkNDT* ptr_landmark_seen = NULL;
		for(size_t j = 0 ; j <_nodes_landmark.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			g2o::Vector2D landmark = _nodes_landmark[j]->estimate();
			cv::Point2f point_land(landmark(0), landmark(1));
			
			double res = cv::norm(point_land - corners_end[i].point);
			
// 			std::cout << "res : " << res << " points "  << point_land << " " << corners_end[i].point << "  cell size " << cell_size << std::endl;
			
			//If we found the landmark, we save the data
			if( res < cell_size * 2){
				seen = true;
				ptr_landmark_seen = _nodes_landmark[j];
			}
		}
		if(seen == false){
// 			std::cout << "New point " << i << " " <<  ret_opencv_point_corner.size() << std::endl;
			assert(i < ret_opencv_point_corner.size());
			g2o::Vector2D vec;
			vec << corners_end[i].point.x, corners_end[i].point.y ;
			AASS::acg::VertexLandmarkNDT* ptr = addLandmarkPose(vec, ret_opencv_point_corner[i], 1);
			ptr->addAngleDirection(corners_end[i].getAngleWidth(), corners_end[i].getDirection());
			ptr->first_seen_from = robot_ptr;

			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr);
			
			
		}
		else{
			std::cout << "Point seen " << std::endl;
			addLandmarkObservation(corners_end[i].getObservations(), corners_end[i].getNodeLinkedPtr(), ptr_landmark_seen);
		}
	}

}


void AASS::acg::AutoCompleteGraph::updateNDTGraph(ndt_feature::NDTFeatureGraph& ndt_graph, bool noise_flag, double deviation){

	std::cout << "BEfore " << std::endl;
// 	printCellsNum();
	std::cout << "Should we check " <<ndt_graph.getNbNodes() << ">" << _previous_number_of_node_in_ndtgraph << std::endl;
	
	//************* Add and create all new nodes, and extract the corners from the new NDT maps *********//
	
	if(ndt_graph.getNbNodes() > _previous_number_of_node_in_ndtgraph){
		
		std::cout << "Found new nodes" << std::endl;
		
		//TODO just get the links that are interesting for you instead of all of them !
		//Doing the registration here myself
		
		auto links = ndt_graph.getOdometryLinks();
		
		//Update the link in all node not already added
		
		//Need at least three node to start considering link since the last one is garbage
		if(_previous_number_of_node_in_ndtgraph >= 2){
			//Ignore last link
			for (size_t i = _previous_number_of_node_in_ndtgraph - 2; i < links.size() - 1; i++) {
		      std::cout << "updating link : " << i << " (size of links :" << links.size() << ")" << std::endl;
				ndt_graph.updateLinkUsingNDTRegistration(links[i], 10, true);
			}
		}
		else{
			ndt_graph.updateLinksUsingNDTRegistration(links, 10, true);
		}
		
		
		
		//Calculate the original transformation of all 
		
		//Assume that all node in the NDT graph must been analysed
		size_t i;
		i = _previous_number_of_node_in_ndtgraph - 1;
// 		std::vector<cv::Point2f> ret_opencv_point_corner;
// 		std::vector<std::pair<double, double> > angles;
				
		//ATTENTION Ignore the last node because it's not good usually !
		for (i; i < ndt_graph.getNbNodes() - 1 ; ++i) {
			
			AASS::acg::VertexSE2RobotPose* robot_ptr;
			g2o::SE2 robot_pos;
			auto map = addElementNDT(ndt_graph, links, i, deviation, &robot_ptr, robot_pos);
			assert(robot_ptr != NULL);
			std::cout << "TEST pointer " << std::endl; std::cout << robot_ptr->getPose().matrix() << std::endl;
			//********************** Extract the corners *****************//
			extractCornerNDTMap(map, robot_ptr, robot_pos);
			//********************** Add the time stamp ******************//
			robot_ptr->setTime(ndt_graph.getNode(i).time_last_update);
			
		}
		//Save new number of nodes to update
	
// 		assert(corners_end.size() == ret_opencv_point_corner.size());
// 		std::vector<AASS::acg::VertexLandmarkNDT*> all_new_landmarks;
		
		// ************ Add the landmarks that were added the corners_end ************************************//
		
		//Add stuff directly in the optimization graph : 
		_previous_number_of_node_in_ndtgraph = ndt_graph.getNbNodes();
	}
	
	
	
	//TODO, update links using the newly found landmarks. No need to update the rest obviously (<-HAHA that was so wrrrrongggg need to update it since they moved :P!)
	
	updateLinks();

	std::cout << "Test no double links" << std::endl;
	noDoubleLinks();
	
	std::cout << "After " << std::endl;
// 	printCellsNum();
		
}



void AASS::acg::AutoCompleteGraph::updateLinks()
{

	std::cout << "Create new links" << std::endl;
	int count = countLinkToMake();
	int count2 = createNewLinks();
	if(count != count2){
		std::cout << "Weird different detection" << std::endl;
		throw std::runtime_error("ARF NOT GOOD COUNT in update link");
	}
	std::cout << "updateLink no small forgotten" << std::endl;
	checkLinkNotForgotten();
	removeBadLinks();
	std::cout << "update link not too big" << std::endl;
	checkLinkNotTooBig();
}

int AASS::acg::AutoCompleteGraph::createNewLinks()
{
	int count = 0 ;
// 	std::vector < std::pair < AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*> > links;
	
	std::cout << "Number new landmarks " << _nodes_landmark.size() << std::endl;
	std::cout << "Prior " << _nodes_prior.size() << std::endl;
// 	if(_nodes_prior.size() > 0){
		
	//Update ALL links
	auto it = _nodes_landmark.begin();
	for(it ; it != _nodes_landmark.end() ; it++){
// 		std::cout << "Working on links " << std::endl;
		Eigen::Vector2d pose_landmark = (*it)->estimate();
		auto it_prior = _nodes_prior.begin();
		for(it_prior ; it_prior != _nodes_prior.end() ; ++it_prior){
			
			Eigen::Vector3d pose_tmp = (*it_prior)->estimate().toVector();
			Eigen::Vector2d pose_prior; pose_prior << pose_tmp(0), pose_tmp(1);
			double norm_tmp = (pose_prior - pose_landmark).norm();
			
			
			
			//Don't add the same link twice
			if(linkAlreadyExist(*it, *it_prior) == false){
				
// 				std::cout << "new" << std::endl;
				
				//Update the link
				if(norm_tmp <= _min_distance_for_link_in_meter){
					std::cout << "NORM " << norm_tmp << "min dist " << _min_distance_for_link_in_meter << std::endl;
// 					ptr_closest = *it_prior;
// 					norm = norm_tmp;
					//Pushing the link
// 					std::cout << "Pushing " << *it << " and " << ptr_closest << std::endl;
// 					links.push_back(std::pair<AASS::acg::VertexLandmarkNDT*, AASS::acg::VertexSE2Prior*>(*it, *it_prior));
					
					g2o::Vector2D vec;
					vec << 0, 0;
					addLinkBetweenMaps(vec, *it_prior, *it);
					
					++count;
				}	
			}
			else{
				
				std::cout << "Already exist" << std::endl;
			}
		}
	}

return count;
	
}

void AASS::acg::AutoCompleteGraph::removeBadLinks()
{
	//Remove links that went too far away from the points and restor the edges to original state when possible:
	int count = 0 ;
	size_t siz = _edge_link.size();
	auto it_old_links = _edge_link.begin();
	for(it_old_links; it_old_links != _edge_link.end();){
		
// 		std::cout << "Studying links "<< std::endl;
		std::vector<Eigen::Vector3d> vertex_out;
		
		assert((*it_old_links)->vertices().size() == 2);
		
// 		std::cout << " LINK " << _edge_link.size() << std::endl;
		AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it_old_links)->vertices()[0]);
		if(ptr == NULL){
			std::cout << ptr << " and " << (*it_old_links)->vertices()[0] << std::endl;
			throw std::runtime_error("Links do not have the good vertex type. Prior");
		}
		auto vertex = ptr->estimate().toVector();
		vertex_out.push_back(vertex);
		
		AASS::acg::VertexLandmarkNDT* ptr2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>((*it_old_links)->vertices()[1]);
		if(ptr2 == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Landmark");
		}
		auto vertex2 = ptr2->estimate();
		Eigen::Vector3d pose_prior; pose_prior << vertex2(0), vertex2(1), 0;
		vertex_out.push_back(pose_prior);

		
// 		std::cout << "bottom of ACG.cpp" << std::endl;
		assert(vertex_out.size() == 2);
		double norm = (vertex_out[0] - vertex_out[1]).norm();
		//Attention magic number
// 		auto it_tmp = it_old_links;
				
// 		it_old_links++;
				
// 		std::vector <g2o::EdgeLinkXY_malcolm* >::iterator next_el = it_old_links;
		
		if(norm > _max_distance_for_link_in_meter ){
			std::cout << "Removing a link" << std::endl;
			std::cout << "NORM " << norm << "min dist " << _max_distance_for_link_in_meter << std::endl;
			it_old_links = removeLinkBetweenMaps(*it_old_links);
		}
		else{
			it_old_links++;
		}	
		std::cout << "Count " << count << " size " << _edge_link.size() << std::endl;
		assert(count <= siz); 
		count++;

	}	
}






void AASS::acg::AutoCompleteGraph::testNoNanInPrior(const std::string& before) const {
	
	std::cout << "Test No nan in prior after " << before << std::endl;
	auto it = _nodes_prior.begin();
	for(it ; it != _nodes_prior.end() ; ++it){
		AASS::acg::VertexSE2Prior* v_ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it));
		if(v_ptr == NULL){
			throw std::runtime_error("not good vertex type");
		}
		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));
		assert(!std::isnan(pose1[2]));
	
	}
	
	std::cout << "Testing the edges now" << std::endl;
	
	auto edges = _edge_prior;	
	auto it_edge = edges.begin();
	for(it_edge ; it_edge != edges.end() ; ++it_edge){
		AASS::acg::VertexSE2Prior* v_ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it_edge)->vertices()[0]);
		if(v_ptr == NULL){
			throw std::runtime_error("no");
		}
		AASS::acg::VertexSE2Prior* v_ptr2 = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it_edge)->vertices()[1]);
		if(v_ptr2 == NULL){
			throw std::runtime_error("no2");
		}
		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
		Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
		
		assert(!std::isnan(pose1[0]));
		assert(!std::isnan(pose1[1]));
		assert(!std::isnan(pose1[2]));
		
		assert(!std::isnan(pose2[0]));
		assert(!std::isnan(pose2[1]));
		assert(!std::isnan(pose2[2]));
	}
	
}

void AASS::acg::AutoCompleteGraph::testInfoNonNul(const std::string& before) const {
	
	std::cout << "Test info non nul after " << before << std::endl;
	auto idmapedges = _optimizable_graph.edges();
	
	for ( auto ite = idmapedges.begin(); ite != idmapedges.end(); ++ite ){
		assert((*ite)->vertices().size() >= 1);
		
		g2o::EdgeLandmark_malcolm* ptr = dynamic_cast<g2o::EdgeLandmark_malcolm*>(*ite);
		AASS::acg::EdgeSE2Prior_malcolm* ptr1 = dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(*ite);
		g2o::EdgeOdometry_malcolm* ptr2 = dynamic_cast<g2o::EdgeOdometry_malcolm*>(*ite);
		g2o::EdgeLinkXY_malcolm* ptr3 = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(*ite);
		if(ptr != NULL){
			assert(ptr->information().isZero(1e-10) == false);
		}		
		else if(ptr1 != NULL){
			assert(ptr1->information().isZero(1e-10) == false);
		}	
		else if(ptr2 != NULL){
			assert(ptr2->information().isZero(1e-10) == false);
		}	
		else if(ptr3 != NULL){
			assert(ptr3->information().isZero(1e-10) == false);
		}
		else{
			throw std::runtime_error("Didn't find the type of the edge :S");
		}
	}	

	
}


void AASS::acg::AutoCompleteGraph::updatePriorEdgeCovariance()
{
	
	std::cout << "DO NOT USE " << std::endl;
	testNoNanInPrior("no dtat");
	assert(false);
	
	auto edges = _edge_prior;	
	auto it = edges.begin();
	for(it ; it != edges.end() ; ++it){
		AASS::acg::VertexSE2Prior* v_ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it)->vertices()[0]);
		if(v_ptr == NULL){
			throw std::runtime_error("no");
		}
		AASS::acg::VertexSE2Prior* v_ptr2 = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it)->vertices()[1]);
		if(v_ptr2 == NULL){
			throw std::runtime_error("no2");
		}
		Eigen::Vector3d pose1 = v_ptr->estimate().toVector();
		Eigen::Vector3d pose2 = v_ptr2->estimate().toVector();
			
		// 			std::cout << "Poses 1 " << std::endl << pose1.format(cleanFmt) << std::endl;
		// 			std::cout << "Poses 2 " << std::endl << pose2.format(cleanFmt) << std::endl;
			
		double tre[3];
		(*it)->getMeasurementData(tre);
		
		Eigen::Vector2d length; length << tre[0], tre[1] ;
		
		Eigen::Vector2d eigenvec; 
		eigenvec << pose1(0) - pose2(0), pose1(1) - pose2(1);
		
		double newnorm = (pose1 - pose2).norm();
		double len_norm = length.norm();
		std::cout << "new norm " << newnorm << " because " << pose1 << " " << pose2 << " and lennorm " << len_norm << "because  " <<length << std::endl;
		g2o::SE2 oldnormse2 = (*it)->interface.getOriginalValue();
		Eigen::Vector3d vecold = oldnormse2.toVector();
		double oldnorm = (vecold).norm();
		std::cout << "oldnorm" << oldnorm << std::endl;
		assert(oldnorm >= 0);
		
		//Using the diff so we cannot shrink it or stretch it easily.
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
		
		//Normalizes it
		double normalizer_own = 1 / oldnorm;
		
		//Between 0 and 1 between 0 and oldnorm
// 		double diff_norm_normalized = 1 - (diff_norm * normalizer_own);
// 		double min = 0;
// 		double max = oldnorm;
// 		double max_range = 1;
// 		double min_range = 0;
		
		//Between 0 and 1 between oldnorm / 2 and oldnorm
		//This is between 0 and oldnorm/2 because we work on the diff and not on the length ==> best length is 0 diff and worse will be half of oldnorm
		double min = 0;
		double max = (oldnorm / 2);
		double max_range = 1;
		double min_range = 0;
		
		double diff_norm_normalized = 1 - ( ( ( (max_range - min_range) * (diff_norm - min ) ) / (max - min) ) + min_range );
		
		double new_cov = diff_norm_normalized;
		
		std::cout << "min " << min << " max " << max << " max_range " << max_range << " min_range " << min_range << " diff norm  " << diff_norm << " cov " << new_cov << std::endl;
		
		assert(new_cov <= 1);
		
		//Sometime the optimization in one turn goes under the limit so need to correct those cases ;)
// 		assert(new_cov >= 0);
		
		if(new_cov <= 0.001){
			//Apparently the vaqlue in the edge does not get changed so it's useless modifying it ?
			//See :
// 			double tre[3];
// 			(*it)->getMeasurementData(tre);
// 			Eigen::Vector2d length; length << tre[0], tre[1] ;
			
			
			new_cov = 0.001;
		}
		
		//Scale it again.depending on user inputed value
		if(_use_user_prior_cov == true){
			new_cov = new_cov * _priorNoise(0);
			assert(new_cov <= _priorNoise(0));
			assert(new_cov >= 0);
		}
		else{
		//Scale it again. depending on oldnorm/10
			new_cov = new_cov * (oldnorm / 10);
			assert(new_cov <= (oldnorm / 10));
			assert(new_cov >= 0);
		}
		
		
	// 				std::cout << "EigenVec " << std::endl << eigenvec.format(cleanFmt) << std::endl;
		std::pair<double, double> eigenval(new_cov, _priorNoise(1));
		
		std::cout << "Eigen vec " << eigenvec << " egenval " << eigenval.first << " " << eigenval.second << std::endl;
		
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
		
		std::cout << "ALL INFO \n" << information_prior << "\n new cov " << new_cov << " cov mat " << cov << std::endl; 
		
		(*it)->setInformation(information_prior);
		
	}
	
	
	
	
}


bool AASS::acg::AutoCompleteGraph::linkAlreadyExist(AASS::acg::VertexLandmarkNDT* v_pt, AASS::acg::VertexSE2Prior* v_prior)
{
	auto it = _edge_link.begin();
	linkAlreadyExist(v_pt, v_prior, it);
}



bool AASS::acg::AutoCompleteGraph::linkAlreadyExist(AASS::acg::VertexLandmarkNDT* v_pt, AASS::acg::VertexSE2Prior* v_prior, std::vector <g2o::EdgeLinkXY_malcolm* >::iterator& it)
{
// 	std::cout << "Testing links" << std::endl;
	for (it ; it != _edge_link.end() ; ++it){
		
		assert((*it)->vertices().size() == 2);
		
// 		std::cout << " LINK " << _edge_link.size() << std::endl;
		AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it)->vertices()[0]);
		if(ptr == NULL){
			std::cout << ptr << " and " << (*it)->vertices()[0] << std::endl;
			throw std::runtime_error("Links do not have the good vertex type. Prior lae");
		}
		AASS::acg::VertexLandmarkNDT* ptr2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>((*it)->vertices()[1]);
		if(ptr2 == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Landmark");
		}
				
// 		std::cout << "if Testing links" << std::endl;
		if(v_pt == ptr2 && v_prior == ptr){
			return true;
		}
		
// 		std::cout << "after Testing links" << std::endl;
		
	}
	return false;

}


bool AASS::acg::AutoCompleteGraph::noDoubleLinks()
{
	auto it = _edge_link.begin();
	for (it ; it != _edge_link.end() ; ){
		
		assert((*it)->vertices().size() == 2);
		
// 		std::cout << " LINK " << _edge_link.size() << std::endl;
		AASS::acg::VertexSE2Prior* ptr = dynamic_cast<AASS::acg::VertexSE2Prior*>((*it)->vertices()[0]);
		if(ptr == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Prior ndl");
		}
		AASS::acg::VertexLandmarkNDT* ptr2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>((*it)->vertices()[1]);
		if(ptr2 == NULL){
			throw std::runtime_error("Links do not have the good vertex type. Landmark");
		}
		
		++it;
		if( it != _edge_link.end()){
			auto it_tmp = it;
			if(linkAlreadyExist(ptr2, ptr, it_tmp) == true){
				return false;
			}
		}
		
	}
	return true;

}

void  AASS::acg::AutoCompleteGraph::setKernelSizeDependingOnAge(g2o::OptimizableGraph::Edge* e, bool step){
			
	g2o::EdgeLinkXY_malcolm* v_linkxy = dynamic_cast<g2o::EdgeLinkXY_malcolm*>(e);
	g2o::EdgeLandmark_malcolm* v_land = dynamic_cast<g2o::EdgeLandmark_malcolm*>(e);
	AASS::acg::EdgeSE2Prior_malcolm* v_prior = dynamic_cast<AASS::acg::EdgeSE2Prior_malcolm*>(e);
	g2o::EdgeOdometry_malcolm* v_odom = dynamic_cast<g2o::EdgeOdometry_malcolm*>(e);
	double age = -1;
	
	assert(_min_age >= 0);
	assert(_max_age >= 0);
	assert(_min_age <= _max_age);
	
	if(v_linkxy != NULL){
// 		assert(v_linkxy->interface->manuallySetAge());
// 		age = v_linkxy->interface->getAge();
		
		//Finding age
		int place = -1;
		for(int i = 0; i < _edge_link.size() ; ++i){
			if(_edge_link[i] == v_linkxy){
				place = i;
				break;
			}
		}
		
		assert(place != -1);
		assert(_edge_interface_of_links[place].manuallySetAge());
		age = _edge_interface_of_links[place].getAge();
		
		//Updating age: The step can go up and down it doesn't matter
// 		std::cout << "Age at first " << age <<std::endl;
		if(step == true){
			if(age + _age_step <= _max_age && age + _age_step >= _min_age || _max_age == -1){
				_edge_interface_of_links[place].setAge(age + _age_step);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
			}
			else if (age + _age_step > _max_age){
				_edge_interface_of_links[place].setAge(_max_age);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
				assert(age_tmp >= age);
			}
			else if (age + _age_step < _min_age){
				_edge_interface_of_links[place].setAge(_min_age);
				double age_tmp = _edge_interface_of_links.at(place).getAge();
				assert(age_tmp <= age);
			}
		}
		
		assert(_edge_interface_of_links.at(place).getAge() <= _max_age);
		assert(_edge_interface_of_links.at(place).getAge() >= _min_age);
		
		//Keep going
// 		std::cout << "kernel size : " << age << " age max " << _max_age << " start " << _age_start_value << std::endl;
		e->robustKernel()->setDelta(age);
		if(_max_age != -1){
			assert(age <= _max_age);
		}
		
	}
	else if(v_land != NULL){
		age = v_land->interface.getAge();
		v_land->interface.setAge(age + 1);
		
// 				std::cout << "kernel size : " << age << std::endl;
		e->robustKernel()->setDelta(100);
	}
	else if(v_prior != NULL){
		age = v_prior->interface.getAge();
		v_prior->interface.setAge(age + 1);
		
// 				std::cout << "kernel size : " << age << std::endl;
		e->robustKernel()->setDelta(100);
		
	}
	else if(v_odom != NULL){
		age = v_odom->interface.getAge();
		v_odom->interface.setAge(age + 1);
		
// 				std::cout << "kernel size : " << age << std::endl;
		e->robustKernel()->setDelta(100);
		
	}
	else{
		std::runtime_error("didn't find edge type");
	}
	
// 			std::cout << "AGE : " << age << std::endl;
	
// 			age = 1 / age;
	
	
}






void AASS::acg::AutoCompleteGraph::getExtremaPrior(double& size_x, double& size_y) const{
	auto edges = getPriorEdges();
		
	double max_x, min_x, max_y, min_y;
	bool flag_init = false;
	auto it = edges.begin();
	for(it ; it != edges.end() ; ++it){
		for(auto ite2 = (*it)->vertices().begin(); ite2 != (*it)->vertices().end() ; ++ite2){
			geometry_msgs::Point p;
			g2o::VertexSE2* ptr = dynamic_cast<g2o::VertexSE2*>((*ite2));
			auto vertex = ptr->estimate().toVector();
			if(flag_init == false){
				flag_init = true;
				max_x = vertex(0);
				max_y = vertex(1);
				min_x = vertex(0);
				min_y = vertex(1);
			}
			else{
				if(max_x < vertex(0)){
					max_x = vertex(0);
				}
				if(max_y < vertex(1)){
					max_y = vertex(1);
				}
				if(min_x > vertex(0)){
					min_x = vertex(0);
				}
				if(min_y > vertex(1)){
					min_y = vertex(1);
				}
			}
			
		}
		
	}
	
	max_x = std::abs(max_x);
	max_y = std::abs(max_y);
	min_x = std::abs(min_x);
	min_y = std::abs(min_y);
	
	if(max_x > min_x){
		size_x = max_x + 10;
	}
	else{
		size_x = min_x + 10;
	}
	if(max_y > min_y){
		size_y = max_y + 10;
	}
	else{
		size_y = min_y + 10;
	}	
}


