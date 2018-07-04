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

	p0 << 0, 0;
	p1_line << -1, 1;
	p2_line << 3, 1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 1 : " << distance << std::endl;

	p0 << 0, 0;
	p1_line << -1, 1;
	p2_line << 1, -1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 0 : " << distance << std::endl;


	p0 << 5, 0;
	p1_line << -1, -1;
	p2_line << 1, -1;

	distance = AASS::acg::distancePointLine(p0, p1_line, p2_line);

	std::cout << "distance should be 1 : " << distance << std::endl;


// 	assert(input_in_robotframe_eigen(0) == 8);
// 	assert(input_in_robotframe_eigen(1) == -1);
// 	assert(input_in_robotframe_eigen(2) == 0);
	
}