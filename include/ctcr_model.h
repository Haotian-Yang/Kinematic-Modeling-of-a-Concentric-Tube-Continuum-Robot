#pragma once

#include <robot_independent.h>

//stl
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include <algorithm>
#include <array>

//Eigen
#include <Eigen/Dense>

// Class that implements the kinematic model of a tendon driven continuum robot
class CTCRModel
{  
	private:
		//Member variables defining the parameters of the CTCR
		std::array<double,3> m_length; //Total length of each tube
		std::array<double,3> m_straight_length; //Straight length of each tube
		std::array<double,3> m_ro; //Outer radius of each tube
		std::array<double,3> m_ri; //Inner radius of each tube
		std::array<double,3> m_curvature; //Curvature of the curved segment of each tube
		double m_youngs_modulus;//Youngs modulus of the tubes' material
		int m_points_per_seg; // Number of equally distributed discrete points in each resulting subsegment for that a frame should be returned
		
		Eigen::MatrixXd m_current_config;
		Eigen::Matrix4d m_ee_frame;
		Eigen::MatrixXd m_backbone_centerline;
		Eigen::Matrix4d m_base_frame;
		
	public:
	
		
		CTCRModel(std::array<double,3> length, std::array<double,3> ro, std::array<double,3> ri, std::array<double,3> straight_length, std::array<double,3> curvature, double youngs_modulus, int pts_per_seg, Eigen::Matrix4d base_frame);
		~CTCRModel();
		
		
		// This function should implement the forward kinematics of a concnetric tube continuum robot (i.e. mapping tube rotations and translations in joint space to the robot's end-effector and shape)
		// Inputs:
		// q						6x1 matrix/vector holding actuation values.
		//							The first three entries are each tube's rotation, while the last three are the tube's translations.
		//
		// Outputs:
		// ee_frame					4x4 matrix storing the end-effector frame resulting from the actuation q.
		// disk_frames				4x4(m*n+1) dimensional matrix storing n frames for each of the m subsegments of the CTCR.
		//							The leftmost 4x4 block should store the initial base/disk frame of the robot.
		// tube_ind					Vector with m entries, specifying the outermost tube for each of the m subsegments.
		// boolean return value		True if kinematics have been calculated successfully, false if not.
		//							Also return false, if the joints limits and inequality constraints are invalidated.
		bool forward_kinematics(Eigen::Matrix4d &ee_frame, Eigen::MatrixXd &backbone_centerline, std::vector<int> &tube_ind, Eigen::MatrixXd q);
		
		
		Eigen::MatrixXd get_current_config();
		Eigen::Matrix4d get_ee_frame();
		Eigen::MatrixXd get_backbone_centerline();
		Eigen::Matrix4d get_base_frame();
};
