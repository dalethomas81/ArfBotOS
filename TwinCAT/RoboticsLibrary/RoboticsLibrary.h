#pragma once

#ifndef ROBOTICS_WRAPPER_LIBRARY_H
#define ROBOTICS_WRAPPER_LIBRARY_H

class RoboticsL {
public:
	static int add(int x, int y);

	// Disallow creating an instance of this object
	// (Making all constructors private also works but is not ideal and does not
	// convey your intent as well)
	//RoboticsL() = delete;

	static int HelloWorld();
};

#endif