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


struct DHParam {
	double theta;
	double d;
	double a;
	double alpha;
};

struct Quaternion {
	double w, x, y, z;
};

struct Pose {
	double x, y, z;
	double roll, pitch, yaw;
};

struct JointAngles {
	double theta1, theta2, theta3, theta4, theta5, theta6;
};

std::vector<std::vector<double>> dhToMatrix(const DHParam& param) {
	double ct = cos(param.theta);
	double st = sin(param.theta);
	double ca = cos(param.alpha);
	double sa = sin(param.alpha);

	return {
		{ct, -st * ca, st * sa, param.a * ct},
		{st, ct * ca, -ct * sa, param.a * st},
		{0, sa, ca, param.d},
		{0, 0, 0, 1}
	};
}

std::vector<std::vector<double>> multiplyMatrices(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
	std::vector<std::vector<double>> C(4, std::vector<double>(4, 0));
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			for (int k = 0; k < 4; ++k) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}
	return C;
}

std::vector<std::vector<double>> forwardKinematics(const std::vector<DHParam>& params) {
	std::vector<std::vector<double>> T = {
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	for (const auto& param : params) {
		T = multiplyMatrices(T, dhToMatrix(param));
	}

	return T;
}

Quaternion matrixToQuaternion(const std::vector<std::vector<double>>& T) {
	Quaternion q;
	q.w = sqrt(1.0 + T[0][0] + T[1][1] + T[2][2]) / 2.0;
	double w4 = 4.0 * q.w;
	q.x = (T[2][1] - T[1][2]) / w4;
	q.y = (T[0][2] - T[2][0]) / w4;
	q.z = (T[1][0] - T[0][1]) / w4;
	return q;
}

void quaternionToEuler(const Quaternion& q, double& roll, double& pitch, double& yaw) {
	// Roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr_cosp, cosr_cosp);

	// Pitch (y-axis rotation)
	double sinp = 2 * (q.w * q.y - q.z * q.x);
	if (abs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// Yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	yaw = atan2(siny_cosp, cosy_cosp);
}

JointAngles inverseKinematics(const Pose& target, const std::vector<DHParam>& params) {
	JointAngles jointAngles;

	// Calculate wrist center position
	double d6 = params[5].d; // Length of the last link
	double wx = target.x - d6 * cos(target.yaw) * cos(target.pitch);
	double wy = target.y - d6 * sin(target.yaw) * cos(target.pitch);
	double wz = target.z - d6 * sin(target.pitch);

	// Calculate joint angles for the position
	jointAngles.theta1 = atan2(wy, wx);
	double r = sqrt(wx * wx + wy * wy);
	double s = wz - params[0].d; // Assuming d1 = params[0].d
	double d = sqrt(r * r + s * s);
	jointAngles.theta2 = atan2(s, r) - acos((params[1].a * params[1].a + d * d - params[2].a * params[2].a) / (2 * params[1].a * d));
	jointAngles.theta3 = M_PI / 2 - acos((params[1].a * params[1].a + params[2].a * params[2].a - d * d) / (2 * params[1].a * params[2].a));

	// Calculate joint angles for the orientation
	double r11 = cos(target.yaw) * cos(target.pitch);
	double r21 = sin(target.yaw) * cos(target.pitch);
	double r31 = -sin(target.pitch);
	double r32 = cos(target.pitch) * sin(target.roll);
	double r33 = cos(target.pitch) * cos(target.roll);

	jointAngles.theta4 = atan2(r32, r33);
	jointAngles.theta5 = atan2(sqrt(r31 * r31 + r32 * r32), r33);
	jointAngles.theta6 = atan2(r21, r11);

	return jointAngles;
}

void printPose(const std::vector<std::vector<double>>& T) {
	double x = T[0][3];
	double y = T[1][3];
	double z = T[2][3];
	double roll = atan2(T[2][1], T[2][2]);
	double pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
	double yaw = atan2(T[1][0], T[0][0]);

	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;


}

void printPose2(const std::vector<std::vector<double>>& T) {
	double x = T[0][3];
	double y = T[1][3];
	double z = T[2][3];
	double roll = atan2(T[2][1], T[2][2]);
	double pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
	double yaw = atan2(T[1][0], T[0][0]);

	// Handle flipping issue
	if (pitch > M_PI / 2) {
		pitch -= M_PI;
		roll += M_PI;
		yaw += M_PI;
	}
	else if (pitch < -M_PI / 2) {
		pitch += M_PI;
		roll -= M_PI;
		yaw -= M_PI;
	}

	// Handle flipping issue
	/*if (pitch > M_PI / 2) {
		pitch = M_PI - pitch;
		roll += M_PI;
		yaw += M_PI;
	}
	else if (pitch < -M_PI / 2) {
		pitch = -M_PI - pitch;
		roll += M_PI;
		yaw += M_PI;
	}*/

	// Normalize angles to the range [-pi, pi]
	roll = fmod(roll + M_PI, 2 * M_PI) - M_PI;
	pitch = fmod(pitch + M_PI, 2 * M_PI) - M_PI;
	yaw = fmod(yaw + M_PI, 2 * M_PI) - M_PI;

	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;
}

void printPose3(const std::vector<std::vector<double>>& T) {
	double x = T[0][3];
	double y = T[1][3];
	double z = T[2][3];
	Quaternion q = matrixToQuaternion(T);

	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
	std::cout << "Orientation (Quaternion): (w: " << RAD_TO_DEG(q.w) << ", x: " << RAD_TO_DEG(q.x) << ", y: " << RAD_TO_DEG(q.y) << ", z: " << RAD_TO_DEG(q.z) << ")" << std::endl;
}

void printPose4(const std::vector<std::vector<double>>& T) {
	double x = T[0][3];
	double y = T[1][3];
	double z = T[2][3];
	Quaternion q = matrixToQuaternion(T);

	double roll, pitch, yaw;
	quaternionToEuler(q, roll, pitch, yaw);

	//std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
	std::cout << "Orientation: (Roll: " << RAD_TO_DEG(roll) << ", Pitch: " << RAD_TO_DEG(pitch) << ", Yaw: " << RAD_TO_DEG(yaw) << ")" << std::endl;
}


int main()
{
	for (int _i=-180; _i<= 180; _i++)
	{

	// AR4
	/*std::vector<DHParam> paramsf = {
		// theta						d			a		alpha
		{DEG_TO_RAD(0.0) + 0,			169.77,		64.2,	-M_PI_2	},
		{DEG_TO_RAD(0.0) + -M_PI_2,		0,			305,	0		},
		{DEG_TO_RAD(0.0) + M_PI,		0,			0,		M_PI_2	},
		{DEG_TO_RAD(0.0) + 0,			222.63,		0,		-M_PI_2	},
		{DEG_TO_RAD(_i) + 0,			0,			0,		M_PI_2	},
		{DEG_TO_RAD(0.0) + 0,			36.25,		0,		0		}
	};
	auto T = forwardKinematics(paramsf);
	printPose(T);

	double x = T[0][3];
	double y = T[1][3];
	double z = T[2][3];
	double roll = atan2(T[2][1], T[2][2]);
	double pitch = atan2(-T[2][0], sqrt(T[2][1] * T[2][1] + T[2][2] * T[2][2]));
	double yaw = atan2(T[1][0], T[0][0]);

	roll = RAD_TO_DEG(roll);
	pitch = RAD_TO_DEG(pitch);
	yaw = RAD_TO_DEG(yaw);

	roll = DEG_TO_RAD(roll);
	pitch = DEG_TO_RAD(pitch);
	yaw = DEG_TO_RAD(yaw);


	// AR4
	std::vector<DHParam> paramsi = {
		// theta	d			a		alpha
		{0,			169.77,		64.2,	-M_PI_2	},
		{-M_PI_2,	0,			305,	0		},
		{M_PI,		0,			0,		M_PI_2	},
		{0,			222.63,		0,		-M_PI_2	},
		{0,			0,			0,		M_PI_2	},
		{0,			36.25,		0,		0		}
	};
	Pose target = { x, y, z, roll, pitch, yaw };

	auto jointAngles = inverseKinematics(target, paramsi);

	
	jointAngles.theta1 = RAD_TO_DEG(jointAngles.theta1);
	jointAngles.theta2 = RAD_TO_DEG(jointAngles.theta2);
	jointAngles.theta3 = RAD_TO_DEG(jointAngles.theta3);
	jointAngles.theta4 = RAD_TO_DEG(jointAngles.theta4);
	jointAngles.theta5 = RAD_TO_DEG(jointAngles.theta5);
	jointAngles.theta6 = RAD_TO_DEG(jointAngles.theta6);

	jointAngles.theta1 = DEG_TO_RAD(jointAngles.theta1);
	jointAngles.theta2 = DEG_TO_RAD(jointAngles.theta2);
	jointAngles.theta3 = DEG_TO_RAD(jointAngles.theta3);
	jointAngles.theta4 = DEG_TO_RAD(jointAngles.theta4);
	jointAngles.theta5 = DEG_TO_RAD(jointAngles.theta5);
	jointAngles.theta6 = DEG_TO_RAD(jointAngles.theta6);*/

	/*std::cout << "Theta1: " << RAD_TO_DEG(jointAngles.theta1) << std::endl;
	std::cout << "Theta2: " << RAD_TO_DEG(jointAngles.theta2) << std::endl;
	std::cout << "Theta3: " << RAD_TO_DEG(jointAngles.theta3) << std::endl;
	std::cout << "Theta4: " << RAD_TO_DEG(jointAngles.theta4) << std::endl;
	std::cout << "Theta5: " << RAD_TO_DEG(jointAngles.theta5) << std::endl;
	std::cout << "Theta6: " << RAD_TO_DEG(jointAngles.theta6) << std::endl;*/



























		double ACS[6];
		double ACS2[6];

		// seed axis angles for testing
		//ACS[0] = -12;
		//ACS[1] = 28;
		//ACS[2] = -9;
		//ACS[3] = 0;
		//ACS[4] = 14;
		//ACS[5] = 62;
		
		ACS[0] = 0.0;
		ACS[1] = 0.0;
		ACS[2] = 0.0;
		ACS[3] = 0.0;
		ACS[4] = (double)_i;
		ACS[5] = 0.0;
		
		// Setup manipulator parameters
		Manipulator_t<6> man;
		
		// AR4
		man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
		man.theta = { 0, -M_PI_2, M_PI, 0, 0, 0 };

		man.r = { 64.2, 305, 0, 0, 0, 0 };
		man.d = { 169.77, 0, 0, 222.63, 0, 36.25 };

		// init kinematics calculator
		KinematicsCalc kin(std::move(man));
		Position_t pos, pos2;
		kin.forwardKinematicsOptimized({	DEG_TO_RAD(ACS[0]), DEG_TO_RAD(ACS[1]), DEG_TO_RAD(ACS[2]), 
											DEG_TO_RAD(ACS[3]), DEG_TO_RAD(ACS[4]), DEG_TO_RAD(ACS[5]) },
											pos);

		// for debugging
		//pos.wx = RAD_TO_DEG(pos.wx);
		//pos.wy = RAD_TO_DEG(pos.wy);
		//pos.wz = RAD_TO_DEG(pos.wz);

		std::cout << "Orientation: (Roll: " << RAD_TO_DEG(pos.wx) << ", Pitch: " << RAD_TO_DEG(pos.wy) << ", Yaw: " << RAD_TO_DEG(pos.wz) << ")" << std::endl;

		//pos.wx = DEG_TO_RAD(pos.wx);
		//pos.wy = DEG_TO_RAD(pos.wy);
		//pos.wz = DEG_TO_RAD(pos.wz);

		//
		std::vector<double> out(6);
		kin.inverseKinematicsOptimized(pos, out);

		//
		for (int i = 0; i <= 5; i++) {
			ACS2[i] = RAD_TO_DEG(out[i]);
		}

	}
}