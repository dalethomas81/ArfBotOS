// RobotManipulator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "ArduinoEigen/ArduinoEigen.h"
#include "robot_manipulator.h"

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

void printMatrix(const Eigen::Matrix3d& mat) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            //Serial.print(mat(i, j));
            //Serial.print("\t");
        }
        //Serial.println();
    }
}

int main()
{
    // 
    JointParameters joints[] = {
      // a      alpha       d           theta
    { 64.2,     -M_PI_2,    169.77,     0.0 },
    { 305,      0.0,		0.0,		-M_PI_2 },
    { 0.0,      M_PI_2,		0.0,		M_PI },
    { 0.0,      -M_PI_2,	222.63,		0.0 },
    { 0.0,      M_PI_2,		0.0,		0.0 },
    { 0.0,      0.0,		36.25,		0.0 }
    };


    // Create a RobotManipulator instance
    RobotManipulator robot(joints, 6);

    // Define joint angles for a specific robot configuration
    double joint_angles_in[] = { DEG_TO_RAD(0.0), DEG_TO_RAD(0.0), DEG_TO_RAD(90.0),
                                DEG_TO_RAD(0.0), DEG_TO_RAD(0.0), DEG_TO_RAD(0.0) };

    // Forward Kinematics
    ForwardKinematicsResult fkResult = robot.forwardKinematics(joint_angles_in);

    //// Print Forward Kinematics results
    //Serial.println("Forward Kinematics Result:");
    //Serial.print("End-Effector Position: ");
    //Serial.print(fkResult.position[0]);
    //Serial.print(", ");
    //Serial.print(fkResult.position[1]);
    //Serial.print(", ");
    //Serial.println(fkResult.position[2]);
    //Serial.println("End-Effector Orientation:");
    //printMatrix(fkResult.orientation);

    // Inverse Kinematics target position and orientation
    //Eigen::Vector3d targetPosition(1.5, 0.5, 1.0);
    //Eigen::Matrix3d targetOrientation;
    //targetOrientation << 1, 0, 0,
    //    0, 1, 0,
    //    0, 0, 1;

    // Inverse Kinematics
    InverseKinematicsResult ikResult = robot.inverseKinematics(fkResult.position, fkResult.orientation);

    //// Print Inverse Kinematics results
    //Serial.println("\nInverse Kinematics Result:");
    //if (ikResult.success) {
    //    Serial.print("Joint Angles: ");
    //    for (int i = 0; i < 3; ++i) {
    //        Serial.print(ikResult.jointAngles[i]);
    //        Serial.print(", ");
    //    }
    //    Serial.println();
    //}
    //else {
    //    Serial.println("Inverse Kinematics failed to converge.");
    //}

    std::cout << "angles out: ";
    for (int _i = 0; _i <= 5; _i++) {
        std::cout << RAD_TO_DEG(ikResult.jointAngles[_i]) << " ";
    }
    std::cout << std::endl;


}


/*
  // Define joint parameters for a simple 3-DOF robot
  JointParameters joints[] = {
    {1.0, 0.0, 0.0, 0.0},  // Joint 1
    {1.0, 0.0, 0.0, 0.0},  // Joint 2
    {1.0, 0.0, 0.0, 0.0}   // Joint 3
  };

  // Create a RobotManipulator instance
  RobotManipulator robot(joints, 3);

  // Define joint angles for a specific robot configuration
  double jointAngles[] = {0.1, 0.2, 0.3};

  // Forward Kinematics
  ForwardKinematicsResult fkResult = robot.forwardKinematics(jointAngles);

  // Print Forward Kinematics results
  Serial.println("Forward Kinematics Result:");
  Serial.print("End-Effector Position: ");
  Serial.print(fkResult.position[0]);
  Serial.print(", ");
  Serial.print(fkResult.position[1]);
  Serial.print(", ");
  Serial.println(fkResult.position[2]);
  Serial.println("End-Effector Orientation:");
  printMatrix(fkResult.orientation);

  // Inverse Kinematics target position and orientation
  Eigen::Vector3d targetPosition(1.5, 0.5, 1.0);
  Eigen::Matrix3d targetOrientation;
  targetOrientation << 1, 0, 0,
                       0, 1, 0,
                       0, 0, 1;

  // Inverse Kinematics
  InverseKinematicsResult ikResult = robot.inverseKinematics(targetPosition, targetOrientation);

  // Print Inverse Kinematics results
  Serial.println("\nInverse Kinematics Result:");
  if (ikResult.success) {
    Serial.print("Joint Angles: ");
    for (int i = 0; i < 3; ++i) {
      Serial.print(ikResult.jointAngles[i]);
      Serial.print(", ");
    }
    Serial.println();
  } else {
    Serial.println("Inverse Kinematics failed to converge.");
  }
*/