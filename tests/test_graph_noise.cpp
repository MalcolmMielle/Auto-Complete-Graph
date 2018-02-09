#include "auto_complete_graph/ACG.hpp"

//Return Gaussian white noise
double randomNoise(double mean, double deviation){
	std::default_random_engine engine{std::random_device()() };
	std::normal_distribution<double> dist(mean, deviation);
	return dist(engine);
};

auto noise_x = randomNoise(0, 1);
auto noise_y = randomNoise(0, 1);
auto noise_t = randomNoise(0, 1);
g2o::SE2 noise_se2(noise_x, noise_y, noise_t);



void simpleGraph(AASS::acg::AutoCompleteGraph& acg){
	
	std::cout << "Making the landmarks" << std::endl;
	//Adding robot poses
	g2o::Vector2 se2; se2 << 1, 1;
	cv::Point2f p;
	AASS::acg::VertexLandmarkNDT* out0 = acg.addLandmarkPose(se2, p);
	out0->setId(0);
	g2o::Vector2 se22; se22 << 2, 1.1;	
	AASS::acg::VertexLandmarkNDT* out1 = acg.addLandmarkPose(se22, p);
	out1->setId(1);
	g2o::Vector2 se222; se222 << 3, 2;
	AASS::acg::VertexLandmarkNDT* out2 = acg.addLandmarkPose(se222, p);
	out2->setId(2);
	
	assert(acg.getGraph().vertices().size() == 3 && "landmark crash");
	//Adding odometry
// 	g2o::EdgeSE2* odo0 = acg.addOdometry(se22, out0, out1);
// 	g2o::SE2 odse2(1, 0.5, 0);
// 	g2o::EdgeSE2* odo1 = acg.addOdometry(se22, out1, out2);
	
	//Adding the Prior now
	
	acg.printGraph();
	
}

void prior(AASS::acg::AutoCompleteGraph& acg){
	
	std::cout << "Making the prior" << std::endl;
	
	acg.printGraph();
	AASS::acg::PriorAttr at;
	g2o::SE2 priorse20(1, 1, 0);
	AASS::acg::VertexSE2Prior* prior0 = acg.addPriorLandmarkPose(priorse20, at);
	prior0->setId(3);
	g2o::SE2 priorse21(2, 1, 0);
	AASS::acg::VertexSE2Prior* prior1 = acg.addPriorLandmarkPose(priorse21, at);
	prior1->setId(4);
	g2o::SE2 priorse22(3, 1, 0);
	AASS::acg::VertexSE2Prior* prior2 = acg.addPriorLandmarkPose(priorse22, at);
	prior2->setId(5);
	
	
	acg.printGraph();
	
	g2o::SE2 move(1, 0, 0);
	
	AASS::acg::EdgeSE2Prior_malcolm* wall0 = acg.addEdgePrior(move, prior0, prior1);
	AASS::acg::EdgeSE2Prior_malcolm* wall1 = acg.addEdgePrior(move, prior1, prior2);
	
	assert(acg.getGraph().vertices().size() == 6 && "prior crash");
	
	auto land0 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>(acg.getGraph().vertex(0));
	auto land1 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>(acg.getGraph().vertex(1));
	auto land2 = dynamic_cast<AASS::acg::VertexLandmarkNDT*>(acg.getGraph().vertex(2));
	
	// TODO : why is this not working
	assert(land0 != NULL);
	assert(land1 != NULL);
	assert(land2 != NULL);
	
	g2o::Vector2 link; link << 0, 0;
	AASS::acg::EdgeLinkXY_malcolm* link0 = acg.addLinkBetweenMaps(link, prior0, land0);
	AASS::acg::EdgeLinkXY_malcolm* link1 = acg.addLinkBetweenMaps(link, prior1, land1);
	AASS::acg::EdgeLinkXY_malcolm* link2 = acg.addLinkBetweenMaps(link, prior2, land2);
	
	std::cout << "Done adding the links" << std::endl;
	
}



void robotGraph(AASS::acg::AutoCompleteGraph& acg){
	
	perception_oru::NDTMap* map = new perception_oru::NDTMap();
	std::shared_ptr<perception_oru::NDTMap> smap(map);
	Eigen::Affine3d aff;
	std::cout << "Making the robot" << std::endl;
	g2o::SE2 rpose(1, 1, 0);
	rpose = rpose * noise_se2;
	auto robot0 = acg.addRobotPose(rpose, aff, smap);
	robot0->setId(6);
	g2o::SE2 rpose2(1, 2, 0);
	rpose2 = rpose2 * noise_se2;
	auto robot1 = acg.addRobotPose(rpose2, aff, smap);
	robot1->setId(7);
	g2o::SE2 rpose3(1, 3, 0);
	rpose3 = rpose3 * noise_se2;
	auto robot2 = acg.addRobotPose(rpose3, aff, smap);
	robot2->setId(8);
	
	g2o::SE2 move(0, 1, 0);
	auto odo0 = acg.addOdometry(move, robot0, robot1);
	auto odo1 = acg.addOdometry(move, robot1, robot2);
	
	Eigen::Vector2d vec; vec << 0, 0;
	auto obs1 = acg.addLandmarkObservation(vec, 6, 0);
	Eigen::Vector2d vec2; vec2 << 1, -0.9;
	auto obs2 = acg.addLandmarkObservation(vec2, 7, 1);
	Eigen::Vector2d vec3; vec3 << 2, -1;
	auto obs3 = acg.addLandmarkObservation(vec3, 8, 2);
	
}


int main(){
	
	AASS::acg::AutoCompleteGraph acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.00005, 0.00001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(1, 1), //Landmarks noise
		Eigen::Vector2d(0.00005, 0.00005), //Prior noise
		sqrt( DEG2RAD(2.) ), //Prior rot
		Eigen::Vector2d(0.00005, 0.00005) //Link noise,
	);
	
	simpleGraph(acg);
	prior(acg);
	robotGraph(acg);
	acg.save("/home/malcolm/ACG_folder/simpleGraph.g2o");
	
	
	std::cout << "NOISE : " << noise_x <<" " << noise_y<<" " << noise_t << std::endl;
	
}
