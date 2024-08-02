#pragma once

//#define __DEBUG__KIN //Output to console the debug data

#include "Matrix.h"

//#include <iostream>
#include <vector>
//#define _USE_MATH_DEFINES
#include <math.h>

// need to include these for twincat sdk
#define M_E        2.71828182845904523536   // e
#define M_LOG2E    1.44269504088896340736   // log2(e)
#define M_LOG10E   0.434294481903251827651  // log10(e)
#define M_LN2      0.693147180559945309417  // ln(2)
#define M_LN10     2.30258509299404568402   // ln(10)
#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2
#define M_PI_4     0.785398163397448309616  // pi/4
#define M_1_PI     0.318309886183790671538  // 1/pi
#define M_2_PI     0.636619772367581343076  // 2/pi
#define M_2_SQRTPI 1.12837916709551257390   // 2/sqrt(pi)
#define M_SQRT2    1.41421356237309504880   // sqrt(2)
#define M_SQRT1_2  0.707106781186547524401  // 1/sqrt(2)

// somthing extra handy
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

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