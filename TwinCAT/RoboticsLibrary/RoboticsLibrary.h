#pragma once

class RoboticsL {
public:
	static int add(int x, int y);

	// Disallow creating an instance of this object
	// (Making all constructors private also works but is not ideal and does not
	// convey your intent as well)
	RoboticsL() = delete;
};