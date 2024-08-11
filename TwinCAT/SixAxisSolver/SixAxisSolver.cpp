// SixAxisSolver.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// https://github.com/AndrewPst/6DOF_inverse_kinematics
// https://github.com/ModySaggaf/Forward-Kinematics-for-6-DoF-Robotic-Arm
#include "Matrix.h"
#include "Kinematics.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

// original
//man.alfa = { -M_PI_2, 0, -M_PI_2, M_PI_2, -M_PI_2, 0 };
//man.theta = { 0, -M_PI_2, 0, 0, 0, 0 };

// https://docs.duet3d.com/User_manual/Machine_configuration/Configuring_Robot_DH_parameters
//man.alfa = { -M_PI_2, 0, -M_PI_2, M_PI_2, -M_PI_2, 0 };
//man.theta = { 0, -M_PI_2, 0, 0, 0, 0 };

// https://automaticaddison.com/homogeneous-transformation-matrices-using-denavit-hartenberg/
//man.alfa = { -M_PI_2, M_PI, M_PI_2, -M_PI_2, M_PI_2, M_PI };
//man.theta = { 0, -M_PI_2, M_PI, 0, 0, M_PI };

// Codesys
//man.alfa = { M_PI_2, 0, M_PI_2, M_PI_2, -M_PI_2, 0 };
//man.theta = { 0, M_PI_2, 0, 0, 0, 0 };

// AR4
//man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
//man.theta = { 0, -M_PI_2, M_PI, 0, 0, 0 };

// TwinCAT "straight up"
//man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
//man.theta = { 0, 0, 0, 0, 0, 0 };

// TwinCAT "right angle"
//man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
//man.theta = { 0, M_PI_2, 0, 0, 0, 0 };

// trial and error
//man.alfa = { -M_PI_2, 0, -M_PI_2, M_PI_2, -M_PI_2, 0 };
//man.theta = { 0, -M_PI_2, 0, 0, 0, 0 };


//struct DHParam {
//	double theta;
//	double d;
//	double a;
//	double alpha;
//};
//
//struct Quaternion {
//	double w, x, y, z;
//};
//
//struct Pose {
//	double x, y, z;
//	double roll, pitch, yaw;
//};
//
//struct JointAngles {
//	double theta1, theta2, theta3, theta4, theta5, theta6;
//};
//
//std::vector<std::vector<double>> dhToMatrix(const DHParam& param) {
//	double ct = cos(param.theta);
//	double st = sin(param.theta);
//	double ca = cos(param.alpha);
//	double sa = sin(param.alpha);
//
//	return {
//		{ct, -st * ca, st * sa, param.a * ct},
//		{st, ct * ca, -ct * sa, param.a * st},
//		{0, sa, ca, param.d},
//		{0, 0, 0, 1}
//	};
//}
//
//std::vector<std::vector<double>> multiplyMatrices(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
//	std::vector<std::vector<double>> C(4, std::vector<double>(4, 0));
//	for (int i = 0; i < 4; ++i) {
//		for (int j = 0; j < 4; ++j) {
//			for (int k = 0; k < 4; ++k) {
//				C[i][j] += A[i][k] * B[k][j];
//			}
//		}
//	}
//	return C;
//}
//
//Pose forwardKinematics(const std::vector<DHParam>& params) {
//	std::vector<std::vector<double>> T = {
//		{1, 0, 0, 0},
//		{0, 1, 0, 0},
//		{0, 0, 1, 0},
//		{0, 0, 0, 1}
//	};
//
//	for (const auto& param : params) {
//		T = multiplyMatrices(T, dhToMatrix(param));
//	}
//	
//	Pose p;
//	p.x = T[0][3];
//	p.y = T[1][3];
//	p.z = T[2][3];
//	p.roll = atan2(T[2][1], T[2][2]);
//	p.pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
//	p.yaw = atan2(T[1][0], T[0][0]);
//
//	return p;
//}
//
//Quaternion matrixToQuaternion(const std::vector<std::vector<double>>& T) {
//	Quaternion q;
//	q.w = sqrt(1.0 + T[0][0] + T[1][1] + T[2][2]) / 2.0;
//	double w4 = 4.0 * q.w;
//	q.x = (T[2][1] - T[1][2]) / w4;
//	q.y = (T[0][2] - T[2][0]) / w4;
//	q.z = (T[1][0] - T[0][1]) / w4;
//	return q;
//}
//
//void quaternionToEuler(const Quaternion& q, double& roll, double& pitch, double& yaw) {
//	// Roll (x-axis rotation)
//	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//	roll = atan2(sinr_cosp, cosr_cosp);
//
//	// Pitch (y-axis rotation)
//	double sinp = 2 * (q.w * q.y - q.z * q.x);
//	if (abs(sinp) >= 1)
//		pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
//	else
//		pitch = asin(sinp);
//
//	// Yaw (z-axis rotation)
//	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//	yaw = atan2(siny_cosp, cosy_cosp);
//}
//
//JointAngles inverseKinematics(const Pose& target, const std::vector<DHParam>& params) {
//	JointAngles jointAngles;
//
//	// Calculate wrist center position
//	double d6 = params[5].d; // Length of the last link
//	double wx = target.x - d6 * cos(target.yaw) * cos(target.pitch);
//	double wy = target.y - d6 * sin(target.yaw) * cos(target.pitch);
//	double wz = target.z - d6 * sin(target.pitch);
//
//	// Debug statements
//	std::cout << "Wrist center: (" << wx << ", " << wy << ", " << wz << ")" << std::endl;
//
//	// Calculate joint angles for the position
//	jointAngles.theta1 = atan2(wy, wx);
//	double r = sqrt(wx * wx + wy * wy);
//	double s = wz - params[0].d; // Assuming d1 = params[0].d
//	double d = sqrt(r * r + s * s);
//
//	// Debug statements
//	std::cout << "r: " << r << ", s: " << s << ", d: " << d << std::endl;
//
//	jointAngles.theta2 = atan2(s, r) - acos((params[1].a * params[1].a + d * d - params[2].a * params[2].a) / (2 * params[1].a * d));
//	jointAngles.theta3 = M_PI / 2 - acos((params[1].a * params[1].a + params[2].a * params[2].a - d * d) / (2 * params[1].a * params[2].a));
//
//	// Calculate joint angles for the orientation
//	double r11 = cos(target.yaw) * cos(target.pitch);
//	double r21 = sin(target.yaw) * cos(target.pitch);
//	double r31 = -sin(target.pitch);
//	double r32 = cos(target.pitch) * sin(target.roll);
//	double r33 = cos(target.pitch) * cos(target.roll);
//
//	jointAngles.theta4 = atan2(r32, r33);
//	jointAngles.theta5 = atan2(sqrt(r31 * r31 + r32 * r32), r33);
//	jointAngles.theta6 = atan2(r21, r11);
//
//	// Debug statements
//	std::cout << "Joint angles: (" << jointAngles.theta1 << ", " << jointAngles.theta2 << ", " << jointAngles.theta3 << ", "
//		<< jointAngles.theta4 << ", " << jointAngles.theta5 << ", " << jointAngles.theta6 << ")" << std::endl;
//
//
//
//
//	//JointAngles jointAngles;
//
//	//// Calculate wrist center position
//	//double d6 = params[5].d; // Length of the last link
//	//double wx = target.x - d6 * cos(target.yaw) * cos(target.pitch);
//	//double wy = target.y - d6 * sin(target.yaw) * cos(target.pitch);
//	//double wz = target.z - d6 * sin(target.pitch);
//
//	//// Calculate joint angles for the position
//	//jointAngles.theta1 = atan2(wy, wx);
//	//double r = sqrt(wx * wx + wy * wy);
//	//double s = wz - params[0].d; // Assuming d1 = params[0].d
//	//double d = sqrt(r * r + s * s);
//	//jointAngles.theta2 = atan2(s, r) - acos((params[1].a * params[1].a + d * d - params[2].a * params[2].a) / (2 * params[1].a * d));
//	//jointAngles.theta3 = M_PI / 2 - acos((params[1].a * params[1].a + params[2].a * params[2].a - d * d) / (2 * params[1].a * params[2].a));
//
//	//// Calculate joint angles for the orientation
//	//double r11 = cos(target.yaw) * cos(target.pitch);
//	//double r21 = sin(target.yaw) * cos(target.pitch);
//	//double r31 = -sin(target.pitch);
//	//double r32 = cos(target.pitch) * sin(target.roll);
//	//double r33 = cos(target.pitch) * cos(target.roll);
//
//	//jointAngles.theta4 = atan2(r32, r33);
//	//jointAngles.theta5 = atan2(sqrt(r31 * r31 + r32 * r32), r33);
//	//jointAngles.theta6 = atan2(r21, r11);
//
//	//double Zero = 0.0001;
//	//if (jointAngles.theta1 < Zero && jointAngles.theta1 > -Zero || isnan(jointAngles.theta1)) {
//	//	jointAngles.theta1 = 0.0;
//	//}
//	//if (jointAngles.theta2 < Zero && jointAngles.theta2 > -Zero || isnan(jointAngles.theta2)) {
//	//	jointAngles.theta2 = 0.0;
//	//}
//	//if (jointAngles.theta3 < Zero && jointAngles.theta3 > -Zero || isnan(jointAngles.theta3)) {
//	//	jointAngles.theta3 = 0.0;
//	//}
//	//if (jointAngles.theta4 < Zero && jointAngles.theta4 > -Zero || isnan(jointAngles.theta4)) {
//	//	jointAngles.theta4 = 0.0;
//	//}
//	//if (jointAngles.theta5 < Zero && jointAngles.theta5 > -Zero || isnan(jointAngles.theta5)) {
//	//	jointAngles.theta5 = 0.0;
//	//}
//	//if (jointAngles.theta6 < Zero && jointAngles.theta6 > -Zero || isnan(jointAngles.theta6)) {
//	//	jointAngles.theta6 = 0.0;
//	//}
//
//	return jointAngles;
//}
//
//void printPose1(Pose p) {
//
//	std::cout << "Pose    : (  " << p.x << "  " << p.y << "  " << p.z << "  ";
//	std::cout << RAD_TO_DEG(p.roll) << "  " << RAD_TO_DEG(p.pitch) << "  " << RAD_TO_DEG(p.yaw) << "  )" << std::endl;
//
//}
//
//void printPose2(const std::vector<std::vector<double>>& T) {
//	double x = T[0][3];
//	double y = T[1][3];
//	double z = T[2][3];
//	double roll = atan2(T[2][1], T[2][2]);
//	double pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
//	double yaw = atan2(T[1][0], T[0][0]);
//
//	// Handle flipping issue
//	if (pitch > M_PI / 2) {
//		pitch -= M_PI;
//		roll += M_PI;
//		yaw += M_PI;
//	}
//	else if (pitch < -M_PI / 2) {
//		pitch += M_PI;
//		roll -= M_PI;
//		yaw -= M_PI;
//	}
//
//	// Handle flipping issue
//	/*if (pitch > M_PI / 2) {
//		pitch = M_PI - pitch;
//		roll += M_PI;
//		yaw += M_PI;
//	}
//	else if (pitch < -M_PI / 2) {
//		pitch = -M_PI - pitch;
//		roll += M_PI;
//		yaw += M_PI;
//	}*/
//
//	// Normalize angles to the range [-pi, pi]
//	roll = fmod(roll + M_PI, 2 * M_PI) - M_PI;
//	pitch = fmod(pitch + M_PI, 2 * M_PI) - M_PI;
//	yaw = fmod(yaw + M_PI, 2 * M_PI) - M_PI;
//
//	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
//	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;
//}
//
//void printPose3(const std::vector<std::vector<double>>& T) {
//	double x = T[0][3];
//	double y = T[1][3];
//	double z = T[2][3];
//	Quaternion q = matrixToQuaternion(T);
//
//	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
//	std::cout << "Orientation (Quaternion): (w: " << RAD_TO_DEG(q.w) << ", x: " << RAD_TO_DEG(q.x) << ", y: " << RAD_TO_DEG(q.y) << ", z: " << RAD_TO_DEG(q.z) << ")" << std::endl;
//}
//
//void printPose4(const std::vector<std::vector<double>>& T) {
//	double x = T[0][3];
//	double y = T[1][3];
//	double z = T[2][3];
//	Quaternion q = matrixToQuaternion(T);
//
//	double roll, pitch, yaw;
//	quaternionToEuler(q, roll, pitch, yaw);
//
//	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
//	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;
//}
//
//void printPose5(const std::vector<std::vector<double>>& T) {
//	double x = T[0][3];
//	double y = T[1][3];
//	double z = T[2][3];
//
//	// Extract rotation matrix elements
//	double r11 = T[0][0], r12 = T[0][1], r13 = T[0][2];
//	double r21 = T[1][0], r22 = T[1][1], r23 = T[1][2];
//	double r31 = T[2][0], r32 = T[2][1], r33 = T[2][2];
//
//	// Calculate Euler angles from rotation matrix
//	double roll = atan2(r32, r33);
//	double pitch = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
//	double yaw = atan2(r21, r11);
//
//	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
//	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;
//}
//
//void pringAxisConfiguration(JointAngles jointAngles) {
//
//	//
//	std::cout << "Axes: (  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta1) << "  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta2) << "  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta3) << "  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta4) << "  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta5) << "  ";
//	std::cout << RAD_TO_DEG(jointAngles.theta6) << "  ";
//	std::cout << "  )" << std::endl;
//}




using namespace std;

struct Pose {
	double x, y, z, roll, pitch, yaw;
};

struct DHParams {
	double alpha, a, d, theta;
};

struct JointAngles {
	double theta1, theta2, theta3, theta4, theta5, theta6;
};

// Function to create a transformation matrix using DH parameters
array<array<double, 4>, 4> dhToMatrix(double alpha, double a, double d, double theta) {
	array<array<double, 4>, 4> T = { {
		{cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)},
		{sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)},
		{0, sin(alpha), cos(alpha), d},
		{0, 0, 0, 1}
	} };
	return T;
}

// Function to multiply two 4x4 matrices
array<array<double, 4>, 4> multiplyMatrices(const array<array<double, 4>, 4>& A, const array<array<double, 4>, 4>& B) {
	array<array<double, 4>, 4> C = { 0 };

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			C[i][j] = 0;
			for (int k = 0; k < 4; ++k) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}

	return C;
}

// Function to compute the pose from the final transformation matrix
Pose computePoseFromMatrix(const array<array<double, 4>, 4>& T) {
	Pose pose;

	// Position
	pose.x = T[0][3];
	pose.y = T[1][3];
	pose.z = T[2][3];

	// Orientation (roll, pitch, yaw)
	pose.roll = atan2(T[2][1], T[2][2]);
	pose.pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
	pose.yaw = atan2(T[1][0], T[0][0]);

	return pose;
}

// Function to calculate the pose given joint angles and DH parameters
Pose calculatePose(const vector<double>& jointAngles, const vector<DHParams>& dhParams) {
	array<array<double, 4>, 4> T = { {
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	} };

	// Multiply transformation matrices for each joint
	for (size_t i = 0; i < jointAngles.size(); ++i) {
		array<array<double, 4>, 4> T_i = dhToMatrix(dhParams[i].alpha, dhParams[i].a, dhParams[i].d, jointAngles[i] + dhParams[i].theta);
		T = multiplyMatrices(T, T_i);
	}

	// Compute and return the pose
	return computePoseFromMatrix(T);
}


// Function to calculate the rotation matrix from roll, pitch, and yaw
array<array<double, 3>, 3> rpyToRotationMatrix(double roll, double pitch, double yaw) {
	array<array<double, 3>, 3> R;

	R[0][0] = cos(yaw) * cos(pitch);
	R[0][1] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
	R[0][2] = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);

	R[1][0] = sin(yaw) * cos(pitch);
	R[1][1] = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
	R[1][2] = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);

	R[2][0] = -sin(pitch);
	R[2][1] = cos(pitch) * sin(roll);
	R[2][2] = cos(pitch) * cos(roll);

	return R;
}

// Function to solve inverse kinematics for a 6-axis robot with a spherical wrist
JointAngles inverseKinematics(const Pose& pose, const vector<DHParams>& dhParams) {
	JointAngles jointAngles;

	// Position of the wrist center (x_w, y_w, z_w)
	double d6 = dhParams[5].d; // Length from wrist center to end effector
	array<array<double, 3>, 3> R = rpyToRotationMatrix(pose.roll, pose.pitch, pose.yaw);
	double x_w = pose.x - d6 * R[0][2];
	double y_w = pose.y - d6 * R[1][2];
	double z_w = pose.z - d6 * R[2][2];

	// Calculate theta1
	double temp;
	temp = atan2(y_w, x_w);
	jointAngles.theta1 = atan2(y_w, x_w);

	// Calculate theta3
	double d1 = dhParams[0].d;
	double a1 = dhParams[0].a;
	double a2 = dhParams[1].a;
	double r = sqrt(x_w * x_w + y_w * y_w) - a1;
	double s = z_w - d1;

	double D = (r * r + s * s - a2 * a2 - dhParams[2].a * dhParams[2].a) / (2 * a2 * dhParams[2].a);
	temp = atan2(sqrt(1 - D * D), D);
	jointAngles.theta3 = atan2(sqrt(1 - D * D), D); // Choose the positive solution

	// Calculate theta2
	double phi2 = atan2(s, r);
	double phi1 = atan2(dhParams[2].a * sin(jointAngles.theta3), a2 + dhParams[2].a * cos(jointAngles.theta3));
	jointAngles.theta2 = phi2 - phi1;

	// Calculate the wrist orientation (theta4, theta5, theta6)
	array<array<double, 4>, 4> T01 = dhToMatrix(dhParams[0].alpha, dhParams[0].a, dhParams[0].d, jointAngles.theta1);
	array<array<double, 4>, 4> T12 = dhToMatrix(dhParams[1].alpha, dhParams[1].a, dhParams[1].d, jointAngles.theta2);
	array<array<double, 4>, 4> T23 = dhToMatrix(dhParams[2].alpha, dhParams[2].a, dhParams[2].d, jointAngles.theta3);

	array<array<double, 4>, 4> T03 = { {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} };
	T03 = multiplyMatrices(T01, T12);
	T03 = multiplyMatrices(T03, T23);

	array<array<double, 3>, 3> R03 = { {
		{T03[0][0], T03[0][1], T03[0][2]},
		{T03[1][0], T03[1][1], T03[1][2]},
		{T03[2][0], T03[2][1], T03[2][2]}
	} };

	array<array<double, 3>, 3> R36;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			R36[i][j] = R03[i][0] * R[0][j] + R03[i][1] * R[1][j] + R03[i][2] * R[2][j];
		}
	}

	jointAngles.theta4 = atan2(R36[1][2], R36[0][2]);
	jointAngles.theta5 = atan2(sqrt(R36[0][2] * R36[0][2] + R36[1][2] * R36[1][2]), R36[2][2]);
	jointAngles.theta6 = atan2(R36[2][1], -R36[2][0]);

	return jointAngles;
}





int main()
{
	for (int _i=-180; _i<= 180; _i+=45)
	//while(true)
	{

		// Example joint angles and DH parameters for a 6-axis robot
		vector<double> jointAnglesi = { DEG_TO_RAD(10.0), DEG_TO_RAD(10.0), DEG_TO_RAD(10.0), 
										DEG_TO_RAD(0.0), DEG_TO_RAD(_i), DEG_TO_RAD(0.0) };
		cout << "Joints In  : ";
		cout << RAD_TO_DEG(jointAnglesi[0]) << "  ";
		cout << RAD_TO_DEG(jointAnglesi[1]) << "  ";
		cout << RAD_TO_DEG(jointAnglesi[2]) << "  ";
		cout << RAD_TO_DEG(jointAnglesi[3]) << "  ";
		cout << RAD_TO_DEG(jointAnglesi[4]) << "  ";
		cout << RAD_TO_DEG(jointAnglesi[5]) << "  ";
		cout << endl;

		vector<DHParams> dhParams = {
			// alpha,		a,			d,			theta
			{-M_PI_2,		64.2,		169.77,		0.0},
			{0.0,			305,		0.0,		-M_PI_2},
			{M_PI_2,		0.0001,		0.0,		M_PI},
			{-M_PI_2,		0.0,		222.63,		0.0},
			{M_PI_2,		0.0,		0.0,		0.0},
			{0.0,			0.0,		36.25,		0.0}
		};

		Pose pose = calculatePose(jointAnglesi, dhParams);

		//cout << "Pose: " << endl;
		//cout << "Position: " << pose.x << "  " << pose.y << "  " << pose.z << "  ";
		//cout << RAD_TO_DEG(pose.roll) << "  " << RAD_TO_DEG(pose.pitch) << "  " << RAD_TO_DEG(pose.yaw) << endl;

		JointAngles jointAngles = inverseKinematics(pose, dhParams);

		//pringAxisConfiguration(jointAngles);

		cout << "Joints Out : ";
		cout << RAD_TO_DEG(jointAngles.theta1) << "  ";
		cout << RAD_TO_DEG(jointAngles.theta2) << "  ";
		cout << RAD_TO_DEG(jointAngles.theta3) << "  ";
		cout << RAD_TO_DEG(jointAngles.theta4) << "  ";
		cout << RAD_TO_DEG(jointAngles.theta5) << "  ";
		cout << RAD_TO_DEG(jointAngles.theta6) << "  ";
		cout << endl;
		cout << endl;























		//JointAngles Jin;
		//Jin.theta1 = DEG_TO_RAD(10.0);
		//Jin.theta2 = DEG_TO_RAD(10.0);
		//Jin.theta3 = DEG_TO_RAD(_i);
		//Jin.theta4 = DEG_TO_RAD(0.0);
		//Jin.theta5 = DEG_TO_RAD(0.0);
		//Jin.theta6 = DEG_TO_RAD(0.0);

		//std::cout << "In  ";
		//pringAxisConfiguration(Jin);

		//// AR4
		//std::vector<DHParam> paramsf = {
		//	// theta					d			a		alpha
		//	{(Jin.theta1) + 0,			169.77,		64.2,	-M_PI_2	},
		//	{(Jin.theta2) + -M_PI_2,	0,			305,	0		},
		//	{(Jin.theta3) + M_PI,		0,			0,		M_PI_2	},
		//	{(Jin.theta4) + 0,			222.63,		0,		-M_PI_2	},
		//	{(Jin.theta5) + 0,			0,			0,		M_PI_2	},
		//	{(Jin.theta6) + 0,			36.25,		0,		0		}
		//};
		//Pose pose = forwardKinematics(paramsf);
		//printPose1(pose);


		//// AR4
		//std::vector<DHParam> paramsi = {
		//	// theta	d			a		alpha
		//	{0,			169.77,		64.2,	-M_PI_2	},
		//	{-M_PI_2,	0,			305,	0		},
		//	{M_PI,		0,			0,		M_PI_2	},
		//	{0,			222.63,		0,		-M_PI_2	},
		//	{0,			0,			0,		M_PI_2	},
		//	{0,			36.25,		0,		0		}
		//};
		//JointAngles Jout = inverseKinematics(pose, paramsi);
		//std::cout << "Out ";
		//pringAxisConfiguration(Jout);
		//std::cout << std::endl;







		
		//// Setup manipulator parameters
		//Manipulator_t<6> man;
		//
		//// AR4
		//man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
		//man.theta = { 0, -M_PI_2, M_PI, 0, 0, 0 };

		//man.r = { 64.2, 305, 0, 0, 0, 0 };
		//man.d = { 169.77, 0, 0, 222.63, 0, 36.25 };

		//// init kinematics calculator
		//KinematicsCalc kin(std::move(man));
		//Position_t pos, pos2;
		//kin.forwardKinematics({	DEG_TO_RAD(ACS[0]), DEG_TO_RAD(ACS[1]), DEG_TO_RAD(ACS[2]), 
		//						DEG_TO_RAD(ACS[3]), DEG_TO_RAD(ACS[4]), DEG_TO_RAD(ACS[5]) },
		//						pos);

		////for debugging
		//pos.wx = RAD_TO_DEG(pos.wx);
		//pos.wy = RAD_TO_DEG(pos.wy);
		//pos.wz = RAD_TO_DEG(pos.wz);

		////std::cout << "Orientation: (Roll: " << (pos.wx) << ", Pitch: " << (pos.wy) << ", Yaw: " << (pos.wz) << ")" << std::endl;

		//pos.wx = DEG_TO_RAD(pos.wx);
		//pos.wy = DEG_TO_RAD(pos.wy);
		//pos.wz = DEG_TO_RAD(pos.wz);

		////
		//std::vector<double> out(6);
		//kin.inverseKinematics(pos, out);

		//
		//std::cout << "Input Axes: (  ";
		//for (int i = 0; i <= 5; i++) {
		//	std::cout << ACS[i] << "  ";
		//}
		//std::cout << "  )" << std::endl;
		//std::cout << "Output Axes: (  ";
		//for (int i = 0; i <= 5; i++) {
		//	ACS2[i] = RAD_TO_DEG(out[i]);
		//	std::cout << ACS2[i] << "  ";
		//}
		//std::cout << "  )" << std::endl;
		//std::cout << std::endl;

	}
}