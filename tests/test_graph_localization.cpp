#include "auto_complete_graph/Localization/ACG_localization.hpp"



g2o::VertexSE2RobotLocalization* robot_node;
g2o::EdgeXYPriorACG* wall;



void simpleGraph(AASS::acg::AutoCompleteGraphLocalization& acg){

	std::cout << "Making the landmarks" << std::endl;
	//Adding robot poses
	const cv::Point2f position;
	g2o::Vector2 se2; se2 << 1, 1;
	g2o::VertexLandmarkNDT* out0 = acg.addLandmarkPose(se2, position);
	out0->setId(0);
	g2o::Vector2 se22; se22 << 2, 1.1;
	g2o::VertexLandmarkNDT* out1 = acg.addLandmarkPose(se22, position);
	out1->setId(1);
	g2o::Vector2 se222; se222 << 3, 2;
	g2o::VertexLandmarkNDT* out2 = acg.addLandmarkPose(se222, position);
	out2->setId(2);

	assert(acg.getGraph().vertices().size() == 3 && "landmark crash");
	//Adding odometry
// 	g2o::EdgeSE2* odo0 = acg.addOdometry(se22, out0, out1);
// 	g2o::SE2 odse2(1, 0.5, 0);
// 	g2o::EdgeSE2* odo1 = acg.addOdometry(se22, out1, out2);

	//Adding the Prior now

	acg.printGraph();

}

void prior(AASS::acg::AutoCompleteGraphLocalization& acg){

	std::cout << "Making the prior" << std::endl;

	acg.printGraph();


	AASS::acg::PriorAttr priorattr;

	g2o::SE2 priorse20(1, 1, 0);
	g2o::VertexXYPrior* prior0 = acg.addPriorLandmarkPose(priorse20, priorattr);
	prior0->setId(3);
	g2o::SE2 priorse21(2, 1, 0);
	g2o::VertexXYPrior* prior1 = acg.addPriorLandmarkPose(priorse21, priorattr);
	prior1->setId(4);
	g2o::SE2 priorse22(3, 1, 0);
	g2o::VertexXYPrior* prior2 = acg.addPriorLandmarkPose(priorse22, priorattr);
	prior2->setId(5);


	acg.printGraph();

	g2o::SE2 move(1, 0, 0);

	std::cout << "Adding the walls: " << prior0->estimate() << " to " << prior1->estimate() << std::endl;

	wall = acg.addEdgePrior(move, prior0, prior1);
//	g2o::EdgeXYPriorACG* wall1 = acg.addEdgePrior(move, prior1, prior2);

	assert(acg.getGraph().vertices().size() == 6 && "prior crash");

	auto land0 = dynamic_cast<g2o::VertexLandmarkNDT*>(acg.getGraph().vertex(0));
	auto land1 = dynamic_cast<g2o::VertexLandmarkNDT*>(acg.getGraph().vertex(1));
	auto land2 = dynamic_cast<g2o::VertexLandmarkNDT*>(acg.getGraph().vertex(2));

	// TODO : why is this not working
	assert(land0 != NULL);
	assert(land1 != NULL);
	assert(land2 != NULL);

//	g2o::Vector2D link; link << 0, 0;
//	g2o::EdgeLinkXY_malcolm* link0 = acg.addLinkBetweenMaps(link, prior0, land0);
//	g2o::EdgeLinkXY_malcolm* link1 = acg.addLinkBetweenMaps(link, prior1, land1);
//	g2o::EdgeLinkXY_malcolm* link2 = acg.addLinkBetweenMaps(link, prior2, land2);





	std::cout << "Done adding the links" << std::endl;

}



void robotGraph(AASS::acg::AutoCompleteGraphLocalization& acg){

	std::cout << "Doing the robot graph" << std::endl;
//	perception_oru::NDTMap map;
	std::shared_ptr<perception_oru::NDTMap> map;

//	Eigen::Affine3d affine;

	std::cout << "Making the robot" << std::endl;
	g2o::SE2 rpose(1, 1, 0);
	Eigen::Affine3d affine_original_robot_pose = Eigen::Affine3d::Identity();
	Eigen::Vector3d to_robot_localization; to_robot_localization << 0, 0, 0;
	Eigen::Matrix3d cov_localization = Eigen::Matrix3d::Identity();
	robot_node = acg.addRobotLocalization(rpose, affine_original_robot_pose, to_robot_localization, cov_localization, map);
	robot_node->setId(6);

	std::cout << "Robot pose " << std::endl;
	std::cout << robot_node->estimate().toVector() << std::endl;

	g2o::SE2 rpose2(1, 2, 0);
	auto robot1 = acg.addRobotLocalization(rpose2, affine_original_robot_pose, to_robot_localization, cov_localization, map);
	robot1->setId(7);
	g2o::SE2 rpose3(1, 3, 0);
	auto robot2 = acg.addRobotLocalization(rpose3, affine_original_robot_pose, to_robot_localization, cov_localization, map);
	robot2->setId(8);

	std::cout << "Saving cov to 2d" << std::endl;
	Eigen::Matrix3d cov_2d;
	cov_2d << 0.5, 0, 0,
			0, 0.5, 0,
			0, 0, 0.1;

// 		tf::matrixEigenToMsg(cov, ndt_graph.factors[element - 1].covariance);
//			std::cout << "Saving information " << std::endl;
	Eigen::Matrix3d information = cov_2d.inverse();

//			std::cout << "Saving odometry " << odometry.toVector() << " from " << from << " toward " << toward << " info " << information << " " << std::endl;
//	addOdometry(odometry, from, toward, information);
	g2o::SE2 move(0, 1, 0);
	auto odo0 = acg.addOdometry(move, robot_node, robot1, information);
	auto odo1 = acg.addOdometry(move, robot1, robot2, information);

	Eigen::Vector2d vec; vec << 0, 0;
	auto obs1 = acg.addLandmarkObservation(vec, 6, 0);
	Eigen::Vector2d vec2; vec2 << 1, -0.9;
	auto obs2 = acg.addLandmarkObservation(vec2, 7, 1);
	Eigen::Vector2d vec3; vec3 << 2, -1;
	auto obs3 = acg.addLandmarkObservation(vec3, 8, 2);

}


void ndtGraph(AASS::acg::AutoCompleteGraphLocalization& acg){

	std::cout << "Doing the ndt " << std::endl;
	Eigen::Vector2d pose; pose << 1, 1;
	boost::shared_ptr<perception_oru::NDTCell> cell(new perception_oru::NDTCell());
	Eigen::Vector3d mean; mean << 1, 1, 0;
	cell->setMean(mean);
	std::cout << "Doing the ndt 2 " << std::endl;
	auto ndt_cell = acg.addNDTCellVertex(pose, cell, robot_node);

	std::cout << "Robot pose " << std::endl;
	std::cout << robot_node->estimate().toVector() << std::endl;

	g2o::Vector2 pos; pos << pose - robot_node->estimate().toVector().head(2);
	std::cout << "Adding observation" << std::endl;
	auto ndt_cell_observation = acg.addNDTCellObservation(pos, robot_node, ndt_cell);
	Eigen::Matrix2d covariance_landmark = Eigen::Matrix2d::Identity();
	std::cout << "Adding association" << std::endl;
	auto ndt_cell_asso = acg.addNDTCellAssociation(ndt_cell, wall, covariance_landmark);
}


int main(){

	AASS::acg::AutoCompleteGraphLocalization acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.0005, 0.0005), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.05, 0.05), //Landmarks noise
		Eigen::Vector2d(50, 0.005), //Prior noise
		DEG2RAD(2.)//Prior rot
	);
	acg.useUserCovForRobotPose(false);

	simpleGraph(acg);
	prior(acg);
	robotGraph(acg);
	ndtGraph(acg);

	acg.setFirst();
	acg.prepare();
	acg.optimize();


//	acg.save("/home/malcolm/ACG_folder/simpleGraph.g2o");

}