
#include "auto_complete_graph/Localization/ACG_localization.hpp"
#include "auto_complete_graph/covariance.hpp"


int main() {
	Eigen::Matrix2d cov_inv_2d;
	cov_inv_2d << 2, 0, 0, 2;

//	std::cout << "Cov " << cov_inv_2d.matrix() << " Inverse " << cov_inv_2d.inverse().matrix() << std::endl;

	Eigen::Vector2d pose_prior; pose_prior << 0, 0 ;
	Eigen::Vector2d pose_landmark; pose_landmark << 2, 0 ;

	double scaling_factor_gaussian = 0.5;

	double l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	double score =  exp(- scaling_factor_gaussian * l); /// / 2.0);

	std::cout << "distance: " << 2 << " l: " << l << " score : " << score << std::endl;



	cov_inv_2d << 2, 0, 0, 2;
	pose_prior << 0, 0 ;
	pose_landmark << 1, 0 ;
	scaling_factor_gaussian = 0.5;
	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 1 << " l: " << l << " score : " << score << std::endl;


	cov_inv_2d << 4, 0, 0, 4;
	pose_prior << 0, 0 ;
	pose_landmark << 2, 0 ;
	scaling_factor_gaussian = 0.5;
	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "all mutlitplied by 2: distance: " << 1 << " l: " << l << " score : " << score << std::endl;


	cov_inv_2d << 8, 0, 0, 8;
	pose_prior << 0, 0 ;
	pose_landmark << 4, 0 ;
	scaling_factor_gaussian = 0.5;
	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "all mutlitplied by 4: distance: " << 1 << " l: " << l << " score : " << score << std::endl;



	cov_inv_2d << 2, 0, 0, 2;
	pose_prior << 0, 0 ;
	pose_landmark << 4, 0 ;
	scaling_factor_gaussian = 0.5;
	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << std::endl;



	cov_inv_2d << 4, 0, 0, 4;
	pose_prior << 0, 0 ;
	pose_landmark << 4, 0 ;
	scaling_factor_gaussian = 0.5;
	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << " l should be smaller here than before " <<std::endl;



	Eigen::Vector2d pose_mcl; pose_mcl << 0, 0;
	Eigen::Rotation2D<double> rot2(0.4);
	double euclidean_distance = (pose_landmark - pose_mcl).norm();
	double opposite_side = euclidean_distance * std::tan(0.4);
	std::cout << "Opposite side " << opposite_side << " distance " << euclidean_distance << std::endl;

	Eigen::Vector2d eigen = (pose_landmark - pose_mcl) / euclidean_distance;
	Eigen::Vector2d main_eigen; main_eigen << -eigen(1), eigen(0);

	Eigen::Matrix2d eigenvec;
	eigenvec << main_eigen(0), eigen(0),
			main_eigen(1), eigen(1);
	//For testing
	std::pair<double, double> eigenval; eigenval.first = opposite_side; eigenval.second = opposite_side / 10;
	Eigen::Matrix2d cov_rotation = AASS::acg::getCovariance(eigenvec, eigenval);

	std::cout << "Cov of rotation " << cov_rotation.matrix() << std::endl;

	cov_inv_2d << 4, 0, 0, 4;
	cov_inv_2d = cov_inv_2d + cov_rotation;
	cov_inv_2d = cov_inv_2d / 2;

	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << " l with rotation " <<std::endl;


	pose_mcl << 0, 1;
//	Eigen::Rotation2D<double> rot3(0.4);
	euclidean_distance = (pose_landmark - pose_mcl).norm();
	opposite_side = euclidean_distance * std::tan(0.4);
	std::cout << "Opposite side " << opposite_side << " distance " << euclidean_distance << std::endl;

	eigen = (pose_landmark - pose_mcl) / euclidean_distance;
	main_eigen << -eigen(1), eigen(0);

	eigenvec << main_eigen(0), eigen(0),
			main_eigen(1), eigen(1);
	//For testing
	eigenval.first = opposite_side; eigenval.second = opposite_side / 10;
	cov_rotation = AASS::acg::getCovariance(eigenvec, eigenval);

	std::cout << "Cov of rotation " << cov_rotation.matrix() << std::endl;

	cov_inv_2d << 4, 0, 0, 4;
	cov_inv_2d = cov_inv_2d + cov_rotation;
	cov_inv_2d = cov_inv_2d / 2;

	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << " l with rotation should be smaller " <<std::endl;



	pose_mcl << 0, 5;
//	Eigen::Rotation2D<double> rot3(0.4);
	euclidean_distance = (pose_landmark - pose_mcl).norm();
	opposite_side = euclidean_distance * std::tan(0.4);
	std::cout << "Opposite side " << opposite_side << " distance " << euclidean_distance << std::endl;

	eigen = (pose_landmark - pose_mcl) / euclidean_distance;
	main_eigen << -eigen(1), eigen(0);

	eigenvec << main_eigen(0), eigen(0),
			main_eigen(1), eigen(1);
	//For testing
	eigenval.first = opposite_side; eigenval.second = opposite_side / 10;
	cov_rotation = AASS::acg::getCovariance(eigenvec, eigenval);

	std::cout << "Cov of rotation " << cov_rotation.matrix() << std::endl;

	cov_inv_2d << 4, 0, 0, 4;
	cov_inv_2d = cov_inv_2d + cov_rotation;
	cov_inv_2d = cov_inv_2d / 2;

	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << " l with rotation should be even smaller " <<std::endl;


	pose_mcl << 0, 10;
//	Eigen::Rotation2D<double> rot3(0.4);
	euclidean_distance = (pose_landmark - pose_mcl).norm();
	opposite_side = euclidean_distance * std::tan(0.4);
	std::cout << "Opposite side " << opposite_side << " distance " << euclidean_distance << std::endl;

	eigen = (pose_landmark - pose_mcl) / euclidean_distance;
	main_eigen << -eigen(1), eigen(0);

	eigenvec << main_eigen(0), eigen(0),
			main_eigen(1), eigen(1);
	//For testing
	eigenval.first = opposite_side; eigenval.second = opposite_side / 10;
	cov_rotation = AASS::acg::getCovariance(eigenvec, eigenval);

	std::cout << "Cov of rotation " << cov_rotation.matrix() << std::endl;

	cov_inv_2d << 4, 0, 0, 4;
	cov_inv_2d = cov_inv_2d + cov_rotation;
	cov_inv_2d = cov_inv_2d / 2;

	l = (pose_prior - pose_landmark).dot(cov_inv_2d.inverse() * (pose_prior - pose_landmark));
	//I need to remove that 2 in the formula since we haven't added the covariance together ! Hence no need to divided by two :)
	score = exp(- scaling_factor_gaussian * l); /// / 2.0);
	std::cout << "distance: " << 4 << " l: " << l << " score : " << score << " l with rotation should be even even smaller " <<std::endl;




















}