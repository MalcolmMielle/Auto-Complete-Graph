#include "auto_complete_graph/ACG.hpp"

g2o::VertexSE2Prior* prior0;

void simpleGraph(AASS::acg::AutoCompleteGraph& acg){
	
	std::cout << "Making the landmarks" << std::endl;
	//Adding robot poses
	g2o::Vector2D se2; se2 << 1, 1;
	g2o::VertexPointXY* out0 = acg.addLandmarkPose(se2);
	out0->setId(0);

	
	assert(acg.getGraph().vertices().size() == 1 && "landmark crash");
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
	
	g2o::SE2 priorse20(2, 0, 0);
	prior0 = acg.addPriorLandmarkPose(priorse20);
	prior0->setId(1);
	g2o::SE2 priorse21(2, 10, 0);
	g2o::VertexSE2Prior* prior1 = acg.addPriorLandmarkPose(priorse21);
	prior1->setId(2);
	
	acg.printGraph();
	
	g2o::SE2 move(0, 10, 0);
	
	g2o::EdgeSE2Prior_malcolm* wall0 = acg.addEdgePrior(move, prior0, prior1);
	
	assert(acg.getGraph().vertices().size() == 3 && "prior crash");
	
	auto land0 = dynamic_cast<g2o::VertexPointXY*>(acg.getGraph().vertex(0));
	
	// TODO : why is this not working
	assert(land0 != NULL);
	
	g2o::Vector2D link; link << 0, 0;
	g2o::EdgeLinkXY_malcolm* link0 = acg.addLinkBetweenMaps(link, prior0, land0);
	g2o::EdgeLinkXY_malcolm* link1 = acg.addLinkBetweenMaps(link, prior1, land0);
	
	std::cout << "Done adding the links" << std::endl;
	
}



void addPostNodes(AASS::acg::AutoCompleteGraph& acg){
	
	std::cout << "Making the landmarks" << std::endl;
	//Adding robot poses
	g2o::Vector2D se2; se2 << 3, 3;
	g2o::VertexPointXY* out0 = acg.addLandmarkPose(se2);
	out0->setId(3);
	
	g2o::Vector2D link; link << 0, 0;
	g2o::EdgeLinkXY_malcolm* link0 = acg.addLinkBetweenMaps(link, prior0, out0);
	
// 	assert(acg.getGraph().vertices().size() == 1 && "landmark crash");
	//Adding odometry
// 	g2o::EdgeSE2* odo0 = acg.addOdometry(se22, out0, out1);
// 	g2o::SE2 odse2(1, 0.5, 0);
// 	g2o::EdgeSE2* odo1 = acg.addOdometry(se22, out1, out2);
	
	//Adding the Prior now
	
	acg.printGraph();
	
}


int main(){
	
	AASS::acg::AutoCompleteGraph acg(g2o::SE2(0.2, 0.1, -0.1),
		Eigen::Vector2d(0.0005, 0.0001), //Robot translation noise
		DEG2RAD(2.), 				//Rotation noise for robot
		Eigen::Vector2d(0.05, 0.05), //Landmarks noise
		Eigen::Vector2d(1, 0.01), //Prior noise
		DEG2RAD(2.), //Prior rot							 
		Eigen::Vector2d(0.2, 0.2) //Link noise,
	);
	
	simpleGraph(acg);
	prior(acg);
// 	robotGraph(acg);
	acg.save("/home/malcolm/ACG_folder/simpleGraph.g2o");
	
	acg.getGraph().setFirst();
	acg.prepare();
// 	acg.initializeOptimization();
	acg.computeInitialGuess();
// 	acg.prepare();
	
	acg.save("/home/malcolm/ACG_folder/initguess.g2o");
	acg.optimize(1);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_0after.g2o");

	acg.optimize(1);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_1after.g2o");

	acg.optimize(1);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_2after.g2o");

	acg.optimize(1);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_3after.g2o");
	
	addPostNodes(acg);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_poseNode.g2o");
	
	acg.prepare();
	acg.save("/home/malcolm/ACG_folder/simpleGraph_poseNodeinitOpti.g2o");
	
	acg.optimize(1);
	acg.save("/home/malcolm/ACG_folder/simpleGraph_poseNodeopti1.g2o");
	
	
}