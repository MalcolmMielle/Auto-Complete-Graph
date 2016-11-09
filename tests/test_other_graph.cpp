#include "auto_complete_graph/ACG.hpp"



void simpleGraph(AASS::acg::AutoCompleteGraph& acg){
	
	std::cout << "Making the landmarks" << std::endl;
	//Adding robot poses
	g2o::Vector2D se2; se2 << 1, 1;
	g2o::VertexPointXY* out0 = acg.addLandmarkPose(se2);
	out0->setId(0);
	g2o::Vector2D se22; se22 << 2, 1.1;	
	g2o::VertexPointXY* out1 = acg.addLandmarkPose(se22);
	out1->setId(1);
	g2o::Vector2D se222; se222 << 5, 2;
	g2o::VertexPointXY* out2 = acg.addLandmarkPose(se222);
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
	
	g2o::SE2 priorse20(1, 1, 0);
	g2o::VertexSE2Prior* prior0 = acg.addPriorLandmarkPose(priorse20);
	prior0->setId(3);
	g2o::SE2 priorse21(2, 1, 0);
	g2o::VertexSE2Prior* prior1 = acg.addPriorLandmarkPose(priorse21);
	prior1->setId(4);
	g2o::SE2 priorse22(2, 0, 0);
	g2o::VertexSE2Prior* prior2 = acg.addPriorLandmarkPose(priorse22);
	prior2->setId(5);
	
	
	acg.printGraph();
	
	g2o::SE2 move(1, 0, 0);
	g2o::SE2 move2(0, -1, 0);
	
	g2o::EdgeSE2Prior_malcolm* wall0 = acg.addEdgePrior(move, prior0, prior1);
	g2o::EdgeSE2Prior_malcolm* wall1 = acg.addEdgePrior(move2, prior1, prior2);
	
	assert(acg.getGraph().vertices().size() == 6 && "prior crash");
	
	auto land0 = dynamic_cast<g2o::VertexPointXY*>(acg.getGraph().vertex(0));
	auto land1 = dynamic_cast<g2o::VertexPointXY*>(acg.getGraph().vertex(1));
	auto land2 = dynamic_cast<g2o::VertexPointXY*>(acg.getGraph().vertex(2));
	
	// TODO : why is this not working
	assert(land0 != NULL);
	assert(land1 != NULL);
	assert(land2 != NULL);
	
	g2o::Vector2D link; link << 0, 0;
	g2o::EdgeLinkXY_malcolm* link0 = acg.addLinkBetweenMaps(link, prior0, land0);
	g2o::EdgeLinkXY_malcolm* link1 = acg.addLinkBetweenMaps(link, prior1, land1);
	g2o::EdgeLinkXY_malcolm* link2 = acg.addLinkBetweenMaps(link, prior2, land2);
	
	std::cout << "Done adding the links" << std::endl;
	
}



void robotGraph(AASS::acg::AutoCompleteGraph& acg){
	
	lslgeneric::NDTMap map;
	std::cout << "Making the robot" << std::endl;
	g2o::SE2 rpose(1, 1, 0);
	auto robot0 = acg.addRobotPose(rpose, &map);
	robot0->setId(6);
	g2o::SE2 rpose2(1, 2, 0);
	auto robot1 = acg.addRobotPose(rpose2, &map);
	robot1->setId(7);
	g2o::SE2 rpose3(1, 3, 0);
	auto robot2 = acg.addRobotPose(rpose3, &map);
	robot2->setId(8);
	
	g2o::SE2 move(0, 1, 0);
	auto odo0 = acg.addOdometry(move, robot0, robot1);
	auto odo1 = acg.addOdometry(move, robot1, robot2);
	
	Eigen::Vector2d vec; vec << 0, 0;
	auto obs1 = acg.addLandmarkObservation(vec, 6, 0);
	Eigen::Vector2d vec2; vec2 << 1, -0.9;
	auto obs2 = acg.addLandmarkObservation(vec2, 7, 1);
	Eigen::Vector2d vec3; vec3 << 4, -1;
	auto obs3 = acg.addLandmarkObservation(vec3, 8, 2);
	
}


int main(){
	
	AASS::acg::AutoCompleteGraph acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.05, 0.01), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.5, 0.5), //Landmarks noise
		Eigen::Vector2d(1, 0.00005), //Prior noise
		DEG2RAD(2.), //Prior rot
		Eigen::Vector2d(0.00005, 0.00005) //Link noise,
	);
	
	simpleGraph(acg);
	prior(acg);
	robotGraph(acg);
	acg.save("/home/malcolm/ACG_folder/simpleGraph.g2o");
	
}