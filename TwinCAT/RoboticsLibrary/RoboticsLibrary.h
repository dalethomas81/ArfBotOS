#pragma once

/*
#ifdef ROBOTICSLIBRARY_EXPORTS
#define ROBOTICSLIBRARY_API __declspec(dllexport)
#else
#define ROBOTICSLIBRARY_API __declspec(dllimport)
#endif
*/


//class ROBOTICSLIBRARY_API RoboticsL {
class RoboticsL {
public:
	static int add(int x, int y);

	// Disallow creating an instance of this object
	// (Making all constructors private also works but is not ideal and does not
	// convey your intent as well)
	RoboticsL() = delete;

	static int HelloWorld();
};