#include "auto_complete_graph/utils.hpp"
#include <iostream>

int main(){
	
	//Testing translations.
	
	std::cout << "TEST" << std::endl;
	Eigen::Vector3d input; input << 0, 0, 0;
	g2o::SE2 robot_pose_frame(0, 0, 0);
	Eigen::Vector3d input_in_robotframe_eigen;
	
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	assert(input_in_robotframe_eigen(0) == 0);
	assert(input_in_robotframe_eigen(1) == 0);
	assert(input_in_robotframe_eigen(2) == 0);
	
	std::cout << "TEST" << std::endl;
	input << 1, -1, 0;
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	assert(input_in_robotframe_eigen(0) == 1);
	assert(input_in_robotframe_eigen(1) == -1);
	assert(input_in_robotframe_eigen(2) == 0);
	
	std::cout << "TEST" << std::endl;
	input << 0, 0, 0;
	Eigen::Vector3d inputrobot; inputrobot << 10, 0, 0;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	assert(input_in_robotframe_eigen(0) == 10);
	assert(input_in_robotframe_eigen(1) == 0);
	assert(input_in_robotframe_eigen(2) == 0);	
	
	std::cout << "TEST" << std::endl;
	input << 2, 0, 0;
	inputrobot << 10, 0, 0;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	assert(input_in_robotframe_eigen(0) == 12);
	assert(input_in_robotframe_eigen(1) == 0);
	assert(input_in_robotframe_eigen(2) == 0);
	
	std::cout << "TEST" << std::endl;
	input << 2, -5, 0;
	inputrobot << 10, -1, 0;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	assert(input_in_robotframe_eigen(0) == 12);
	assert(input_in_robotframe_eigen(1) == -6);
	assert(input_in_robotframe_eigen(2) == 0);
	
	//Testing rotations
	std::cout << "TEST" << std::endl;
	input << 0, 0, 0;
	inputrobot << 0, 0, 3.1415;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	std::cout << input_in_robotframe_eigen << "\n\n" << std::endl;
	assert(input_in_robotframe_eigen(0) == 0);
	assert(input_in_robotframe_eigen(1) == 0);
// 	assert(input_in_robotframe_eigen(2) == -3.1415);
	
	//Testing rotations
	std::cout << "TEST" << std::endl;
	input << 2, 0, 0;
	inputrobot << 0, 0, 3.1415;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	std::cout << input_in_robotframe_eigen << std::endl;
	
// 	assert(input_in_robotframe_eigen(0) == -2);
// 	assert(input_in_robotframe_eigen(1) == 0);
// 	assert(input_in_robotframe_eigen(2) == 0);
	
	std::cout << "TEST" << std::endl;
	input << 2, 0, 0;
	inputrobot << 10, -1, 3.1415;
	robot_pose_frame.fromVector(inputrobot);
	AASS::acg::translateFromRobotFrameToGlobalFrame(input, robot_pose_frame, input_in_robotframe_eigen);
	
	std::cout << input_in_robotframe_eigen << std::endl;

	std::cout << "TEST LINE POINT DISTANCE" << std::endl;


	Eigen::Vector2d p0; p0 << 0, 0;
	Eigen::Vector2d p1_line; p1_line << -1, 0;
	Eigen::Vector2d p2_line; p2_line << 1, 0;

	double distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 0 : " << distance << std::endl;

	Eigen::Vector2d p1p0 = p0 - p1_line;
	Eigen::Vector2d p1p2 = p2_line - p1_line;
	Eigen::Vector2d p2p0 = p0 - p2_line;

	std::cout << "vecs " << p1p0 << " \n" << p1p2 << " \n" << p2p0 << " \n" << -p1p2 << std::endl;
	std::cout << p1p2.dot(p2p0) << ">= 0 && " << (- p1p2).dot(p1p0) << " >= 0 " << std::endl;

	//Check if closest segment point is on the line segment of the wall:
	if(p1p2.dot(p2p0) <= 0 && (- p1p2).dot(p1p0) <= 0) {
		std::cout << "INSIDE GOOD" << std::endl;
	}
	else{
		std::cout << "OUTSIDE BUG " << std::endl;
	}

//	Eigen::Vector2d AB = B - A;
//	Eigen::Vector2d AC = C - A;

	double a1 = p1p0.dot(p1p2 / p1p2.norm() );
	Eigen::Vector2d p1p0p = a1 * (p1p2 / p1p2.norm() );
	Eigen::Vector2d p0p0p = -p1p0 + p1p0p;

	std::cout << " Projection point 1 0 -> " << p1p0p << "Projection from p0 0 0 -> " << p0p0p << std::endl;

	p0 << 0, 0;
	p1_line << -1, 1;
	p2_line << 3, 1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 1 : " << distance << std::endl;

	p1p0 = p0 - p1_line;
	p1p2 = p2_line - p1_line;
	p2p0 = p0 - p2_line;

	//Check if closest segment point is on the line segment of the wall:
	if(p1p2.dot(p2p0) <= 0 && (- p1p2).dot(p1p0) <= 0) {
		std::cout << "INSIDE GOOD" << std::endl;
	}
	else{
		std::cout << "OUTSIDE BUG " << std::endl;
	}
	a1 = p1p0.dot(p1p2 / p1p2.norm() );
	p1p0p = a1 * (p1p2 / p1p2.norm() );
	p0p0p = -p1p0 + p1p0p;

	std::cout << "Projection point 1 0 -> " << p1p0p << "Projection from p0 0 1 -> " << p0p0p << "distance 1  " << std::get<1>( AASS::acg::distancePointSegment(p0, p1_line, p2_line) ) << std::endl;

	p0 << 0, 0;
	p1_line << -1, 1;
	p2_line << 1, -1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 0 : " << distance << std::endl;

	p1p0 = p0 - p1_line;
	p1p2 = p2_line - p1_line;
	p2p0 = p0 - p2_line;

	//Check if closest segment point is on the line segment of the wall:
	if(p1p2.dot(p2p0) <= 0 && (- p1p2).dot(p1p0) <= 0) {
		std::cout << "INSIDE GOOD" << std::endl;
	}
	else{
		std::cout << "OUTSIDE BUG " << std::endl;
	}
	a1 = p1p0.dot(p1p2 / p1p2.norm() );
	p1p0p = a1 * (p1p2 / p1p2.norm() );
	p0p0p = -p1p0 + p1p0p;

	std::cout << "Projection point 1 -1 -> " << p1p0p << "Projection from p0 0 0 -> " << p0p0p << "distance 0  " << std::get<1>( AASS::acg::distancePointSegment(p0, p1_line, p2_line) )<< std::endl;



	p0 << 5, 0;
	p1_line << -1, -1;
	p2_line << 1, -1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 1 : " << distance << std::endl;

	p1p0 = p0 - p1_line;
	p1p2 = p2_line - p1_line;
	p2p0 = p0 - p2_line;

	//Check if closest segment point is on the line segment of the wall:
	if(p1p2.dot(p2p0) <= 0 && (- p1p2).dot(p1p0) <= 0) {
		std::cout << "INSIDE SO BUG" << std::endl;
	}
	else{
		std::cout << "GOOD " << std::endl;
	}
	if(p1p2.dot(p2p0) >= 0){
		p0p0p = -p2p0;
	}else if( (-p1p2).dot(p1p0) >= 0 ){
		p0p0p = -p1p0;
	}else {
		//Vector to line
		a1 = p1p0.dot(p1p2 / p1p2.norm() );
		p1p0p = a1 * (p1p2 / p1p2.norm() );
		p0p0p = -p1p0 + p1p0p;
	}

	std::cout << "Projection point 6 0 -> " << p1p0p << "Projection from p0 -4 -1 -> " << p0p0p << "distance 5.099  " << std::get<1>(AASS::acg::distancePointSegment(p0, p1_line, p2_line) ) << std::endl;

	p1_line << -3, -1;
	std::cout << "SAME ? Projection point 6 0 -> " << p1p0p << "Projection from p0 -4 -1 -> " << p0p0p << "distance 5.099  " << std::get<1>(AASS::acg::distancePointSegment(p0, p1_line, p2_line) ) << std::endl;


	p0 << -0.909635,
	      -38.2811;
//	3.6253
//	-0.397038
	p1_line << -4.57298,
	           -38.2312;
	p2_line << 0.88458,
	           -38.8289;


	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be ? : " << distance << std::endl;

	p1p0 = p0 - p1_line;
	p1p2 = p2_line - p1_line;
	p2p0 = p0 - p2_line;

	//Check if closest segment point is on the line segment of the wall:
	if(p1p2.dot(p2p0) <= 0 && (- p1p2).dot(p1p0) <= 0) {
		std::cout << "INSIDE" << std::endl;
	}
	else{
		std::cout << "OUTSIDE BUG " << std::endl;
	}
	if(p1p2.dot(p2p0) >= 0){
		p0p0p = -p2p0;
	}else if( (-p1p2).dot(p1p0) >= 0 ){
		p0p0p = -p1p0;
	}else {
		//Vector to line
		a1 = p1p0.dot(p1p2 / p1p2.norm() );
		p1p0p = a1 * (p1p2 / p1p2.norm() );
		p0p0p = -p1p0 + p1p0p;
	}

	std::cout << "Projection point ? ? -> " << p1p0p << "Projection from p0 -? ? -> " << p0p0p << "distance ?  " << p0p0p.norm() << " and " << std::get<1>(AASS::acg::distancePointSegment(p0, p1_line, p2_line) ) << std::endl;


//	1 -4.57298
//	-38.2312
//	p2  0.88458
//	    -38.8289
//	p0 -0.909635
//	-38.2811
//	min value 0.5 distance 0



// 	assert(input_in_robotframe_eigen(0) == 8);
// 	assert(input_in_robotframe_eigen(1) == -1);
// 	assert(input_in_robotframe_eigen(2) == 0);
	
}