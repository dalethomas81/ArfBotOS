#pragma once

#define __DEBUG__KIN //Output to console the debug data

#include "Matrix.h"

#include <iostream>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h>

template <int _Dof = 6>
struct Manipulator_t
{
    std::array<double, _Dof> d;
    std::array<double, _Dof> alfa;
    std::array<double, _Dof> r;
    std::array<double, _Dof> theta;
};

struct Position_t
{
    double x {0}, y{0}, z{0};
    double wx{0}, wy{0}, wz{0};
};

enum class CalculationResult_t
{
    RESULT_ERROR = 0,
    RESULT_SUCCESSFULL,
};

#ifdef __DEBUG__KIN
#define DEBUG_MATRIX(m) m.printMatrix(std::cout);
#define DEBUG_STR(s) std::cout << s << std::endl;
#define DEBUG_DH 
#else
#define DEBUG_MATRIX(m)
#define DEBUG_STR(s)
#endif

class KinematicsCalc
{
    using calc_t = double;
private:
    static const int DEFAULT_DOF = 6;

    const Manipulator_t<DEFAULT_DOF> &_man;

public:
    explicit KinematicsCalc(const Manipulator_t<> &);
    explicit KinematicsCalc(Manipulator_t<> &&);

    // Calculation the effector position by angles - 6DOF
    CalculationResult_t forwardKinematics(const std::vector<calc_t>& t, Position_t& out);
    CalculationResult_t forwardKinematicsOptimized(const std::vector<calc_t>& t, Position_t& out);

    // Calculation the joints angles by effector position
    CalculationResult_t inverseKinematics(const Position_t&, std::vector<calc_t>&);
    CalculationResult_t inverseKinematicsOptimized(const Position_t&, std::vector<calc_t>&);

private:
    void inverseTransformMatrix(Matrix<calc_t> &m, Matrix<calc_t> &out);
    void transformMatrixToPosition(Matrix<calc_t> &m, Position_t &out);
    void positionToTransformMatrix(const Position_t &pos, Matrix<calc_t> &out);
    void createDHFrameMatrix(calc_t theta, calc_t alfa, calc_t r, calc_t d, Matrix<calc_t> &out);
};