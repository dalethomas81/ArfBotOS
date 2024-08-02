#include "TcPch.h"
#pragma hdrstop

#include "Kinematics.h"

KinematicsCalc::KinematicsCalc(const Manipulator_t<> &man)
    : _man(man)
{
}

KinematicsCalc::KinematicsCalc(Manipulator_t<> &&man)
    : _man(std::move(man))
{
//#ifdef DEBUG_DH

    //std::cout << "Denavit-Hanterberg parameters\n";
    //std::cout << "Theta\tAlfa\tR\tD\t\n";
    //for (size_t i = 0; i < DEFAULT_DOF; i++)
    //{
    //    std::cout << _man.theta[i] << '\t' << _man.alfa[i] << '\t' << _man.r[i] << '\t' << _man.d[i] << std::endl;
    //}
    //std::cout << "===================\n";

//#endif
}

CalculationResult_t KinematicsCalc::inverseKinematics(const Position_t &Xik, std::vector<calc_t> &Jik)
{
    //DEBUG_STR("Inverse kinematics start")

    Position_t Xwf; // work frame
    Position_t Xft; // tool frame

    Matrix<calc_t> Twf(4, 4); // work frame matrix;
    positionToTransformMatrix(Xwf, Twf);

    Matrix<calc_t> Ttf(4, 4); // tool frame transf matrix;
    positionToTransformMatrix(Xft, Ttf);

    Matrix<calc_t> Twt(4, 4); // total transf matrix
    positionToTransformMatrix(Xik, Twt);

    //DEBUG_STR("Total transform  matrix")
    //DEBUG_MATRIX(Twt)

    // find T06
    Matrix<calc_t> inTwf(4, 4), inTtf(4, 4), Tw6(4, 4), T06(4, 4);
    inverseTransformMatrix(Twf, inTwf);
    inverseTransformMatrix(Ttf, inTtf);
    Tw6 = Twt * inTtf;
    T06 = inTwf * Tw6;

    //DEBUG_STR("T06 matrix")
    //DEBUG_MATRIX(T06)

    calc_t Xsw[3];
    Xsw[0] = T06.at(0, 3) - _man.d[5] * T06.at(0, 2);
    Xsw[1] = T06.at(1, 3) - _man.d[5] * T06.at(1, 2);
    Xsw[2] = T06.at(2, 3) - _man.d[5] * T06.at(2, 2);

    // joints variable

    // First joint
    Jik[0] = atan2(Xsw[1], Xsw[0]) - atan2(_man.d[2], sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]));

    // Second joint
    Jik[1] = M_PI / 2.0 - acos((_man.r[1] * _man.r[1] + (Xsw[2] - _man.d[0]) * (Xsw[2] - _man.d[0]) /*1*/ + (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0]) /*2*/ * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0]) /*3*/ - (_man.r[2] * _man.r[2] + _man.d[3] * _man.d[3])) / (2.0 * _man.r[1] * sqrt((Xsw[2] - _man.d[0]) * (Xsw[2] - _man.d[0]) /*4*/ + (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0]) * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0])))) - atan((Xsw[2] - _man.d[0]) / (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0])); // Jik(2)=pi/2-acos((_man.r(2)^2+(Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((Xsw(3)-d(1))^2+(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)))-atan((Xsw(3)-d(1))/(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1)));

    // third joint
    Jik[2] = M_PI - acos((_man.r[1] * _man.r[1] + _man.r[2] * _man.r[2] + _man.d[3] * _man.d[3] - (Xsw[2] - _man.d[0]) * (Xsw[2] - _man.d[0]) - (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0]) * (sqrt(Xsw[0] * Xsw[0] + Xsw[1] * Xsw[1] - _man.d[2] * _man.d[2]) - _man.r[0])) / (2 * _man.r[1] * sqrt(_man.r[2] * _man.r[2] + _man.d[3] * _man.d[3]))) - atan2(_man.d[3], _man.r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(Xsw(3)-d(1))^2-(sqrt(Xsw(1)^2+Xsw(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));

    Matrix<calc_t> T01(4, 4), T12(4, 4), T23(4, 4), T02(4, 4), T03(4, 4), inT03(4, 4), T36(4, 4);
    createDHFrameMatrix(_man.theta[0] + Jik[0], _man.alfa[0], _man.r[0], _man.d[0], T01);
    createDHFrameMatrix(_man.theta[1] + Jik[1], _man.alfa[1], _man.r[1], _man.d[1], T12);
    createDHFrameMatrix(_man.theta[2] + Jik[2], _man.alfa[2], _man.r[2], _man.d[2], T23);
    T02 = T01 * T12;
    T03 = T02 * T23;
    //DEBUG_STR("T03 matrix")
    //DEBUG_MATRIX(T03)

    inverseTransformMatrix(T03, inT03);
    T36 = inT03 * T06;
    //DEBUG_STR("T36 matrix")
    //DEBUG_MATRIX(T36)

    Jik[3] = atan2(-T36.at(1, 2), -T36.at(0, 2));
    Jik[4] = atan2(sqrt(T36.at(0, 2) * T36.at(0, 2) + T36.at(1, 2) * T36.at(1, 2)), T36.at(2, 2));
    Jik[5] = atan2(-T36.at(2, 1), T36.at(2, 0));

    //DEBUG_STR("Inverse kinematics end")

    return CalculationResult_t::RESULT_SUCCESSFULL;
}

CalculationResult_t KinematicsCalc::inverseKinematicsOptimized(const Position_t &Xik, std::vector<calc_t> &Jik)
{
    //DEBUG_STR("Inverse kinematics start")

    Matrix<calc_t> Twf(4, 4); // work frame matrix;
    positionToTransformMatrix(Position_t(), Twf);
    Matrix<calc_t> Ttf(Twf); // tool frame transform matrix;

    Matrix<calc_t> Twt(4, 4); // total transform matrix
    positionToTransformMatrix(Xik, Twt);

    // find T06 - transformation matrix from 0 to N frame
    Matrix<calc_t> inTwf(4, 4), inTtf(4, 4);
    inverseTransformMatrix(Twf, inTwf);
    inverseTransformMatrix(Ttf, inTtf);
    Matrix<calc_t> t06 = inTwf * (Twt * inTtf);
    //DEBUG_STR("T06 matrix")
    //DEBUG_MATRIX(t06)
    
    calc_t tempPos[3]; // Buffer for calculation first 3 joints. This is position of N=5 joint
    tempPos[0] = t06.at(0, 3) - _man.d[5] * t06.at(0, 2);
    tempPos[1] = t06.at(1, 3) - _man.d[5] * t06.at(1, 2);
    tempPos[2] = t06.at(2, 3) - _man.d[5] * t06.at(2, 2);

    // First joint
    Jik[0] = atan2(tempPos[1], tempPos[0]) - atan2(_man.d[2], sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]));

    // Second joint
    Jik[1] = M_PI / 2.0 - acos((_man.r[1] * _man.r[1] + (tempPos[2] - _man.d[0]) * (tempPos[2] - _man.d[0]) + (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0]) * (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0]) - (_man.r[2] * _man.r[2] + _man.d[3] * _man.d[3])) / (2.0 * _man.r[1] * sqrt((tempPos[2] - _man.d[0]) * (tempPos[2] - _man.d[0]) + (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0]) * (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0])))) - atan((tempPos[2] - _man.d[0]) / (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0])); // Jik(2)=pi/2-acos((_man.r(2)^2+(tempPos(3)-d(1))^2+(sqrt(tempPos(1)^2+tempPos(2)^2-d(3)^2)-r(1))^2-(r(3)^2+d(4)^2))/(2*r(2)*sqrt((tempPos(3)-d(1))^2+(sqrt(tempPos(1)^2+tempPos(2)^2-d(3)^2)-r(1))^2)))-atan((tempPos(3)-d(1))/(sqrt(tempPos(1)^2+tempPos(2)^2-d(3)^2)-r(1)));

    // third joint
    Jik[2] = M_PI - acos((_man.r[1] * _man.r[1] + _man.r[2] * _man.r[2] + _man.d[3] * _man.d[3] - (tempPos[2] - _man.d[0]) * (tempPos[2] - _man.d[0]) - (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0]) * (sqrt(tempPos[0] * tempPos[0] + tempPos[1] * tempPos[1] - _man.d[2] * _man.d[2]) - _man.r[0])) / (2 * _man.r[1] * sqrt(_man.r[2] * _man.r[2] + _man.d[3] * _man.d[3]))) - atan2(_man.d[3], _man.r[2]); // Jik(3)=pi-acos((r(2)^2+r(3)^2+d(4)^2-(tempPos(3)-d(1))^2-(sqrt(tempPos(1)^2+tempPos(2)^2-d(3)^2)-r(1))^2)/(2*r(2)*sqrt(r(3)^2+d(4)^2)))-atan(d(4)/r(3));

    // Calculate T36 transoform matrix
    Matrix<calc_t> t_n_to_n1(4, 4), t_n1_to_n2(4, 4), invT03(4, 4);
    // T03 = T01 * T12 * T23, there t_n_to_n1 = T01, t_n1_to_n2 = T12 and T23(overwriting)
    createDHFrameMatrix(_man.theta[0] + Jik[0], _man.alfa[0], _man.r[0], _man.d[0], t_n_to_n1);
    createDHFrameMatrix(_man.theta[1] + Jik[1], _man.alfa[1], _man.r[1], _man.d[1], t_n1_to_n2);
    Matrix<calc_t> t03 = t_n_to_n1 * t_n1_to_n2;
    createDHFrameMatrix(_man.theta[2] + Jik[2], _man.alfa[2], _man.r[2], _man.d[2], t_n1_to_n2);
    t03 = t03 * t_n1_to_n2;
    //DEBUG_STR("T03 matrix")
    //DEBUG_MATRIX(t03)
    inverseTransformMatrix(t03, invT03);
    Matrix<calc_t> t36 = invT03 * t06;
    //DEBUG_STR("T36 matrix")
    //DEBUG_MATRIX(t36)
    // Find last joints
    Jik[3] = atan2(-t36.at(1, 2), -t36.at(0, 2));
    Jik[4] = atan2(sqrt(t36.at(0, 2) * t36.at(0, 2) + t36.at(1, 2) * t36.at(1, 2)), t36.at(2, 2));
    Jik[5] = atan2(-t36.at(2, 1), t36.at(2, 0));

    //DEBUG_STR("Inverse kinematics end")

    return CalculationResult_t::RESULT_SUCCESSFULL;
}

CalculationResult_t KinematicsCalc::forwardKinematics(const std::vector<calc_t> &t, Position_t &out)
{
    // forward kenematics
    //  input: t - joints value for the calculation of the forward kinematics
    //  output: out - pos value for the calculation of the forward kinematics

    Position_t workFrame;
    Position_t toolFrame;

    Matrix<calc_t> workFrameTransformM(4, 4);
    Matrix<calc_t> toolFrameTransformM(4, 4);

    positionToTransformMatrix(workFrame, workFrameTransformM);
    positionToTransformMatrix(toolFrame, toolFrameTransformM);

    // Calculation the transformation matrix - T06 = T01 * T12 * T23 * T34 * T45 * T56
    Matrix<calc_t> t01(4, 4), t12(4, 4), t23(4, 4), t34(4, 4), t45(4, 4), t56(4, 4);
    createDHFrameMatrix(_man.theta[0] + t[0], _man.alfa[0], _man.r[0], _man.d[0], t01);
    createDHFrameMatrix(_man.theta[1] + t[1], _man.alfa[1], _man.r[1], _man.d[1], t12);
    createDHFrameMatrix(_man.theta[2] + t[2], _man.alfa[2], _man.r[2], _man.d[2], t23);
    createDHFrameMatrix(_man.theta[3] + t[3], _man.alfa[3], _man.r[3], _man.d[3], t34);
    createDHFrameMatrix(_man.theta[4] + t[4], _man.alfa[4], _man.r[4], _man.d[4], t45);
    createDHFrameMatrix(_man.theta[5] + t[5], _man.alfa[5], _man.r[5], _man.d[5], t56);

    Matrix<calc_t> Tw1(4, 4), Tw2(4, 4), Tw3(4, 4), Tw4(4, 4), Tw5(4, 4), Tw6(4, 4), Twt(4, 4);
    Tw1 = workFrameTransformM * t01;
    Tw2 = Tw1 * t12;
    Tw3 = Tw2 * t23;
    Tw4 = Tw3 * t34;
    Tw5 = Tw4 * t45;
    Tw6 = Tw5 * t56;
    Twt = Tw6 * toolFrameTransformM;

    transformMatrixToPosition(Twt, out);

    return CalculationResult_t::RESULT_SUCCESSFULL;
}

CalculationResult_t KinematicsCalc::forwardKinematicsOptimized(const std::vector<calc_t> &t, Position_t &out)
{
    Matrix<calc_t> baseT(4, 4);  // Zero position of the base frame
    Matrix<calc_t> workT(4, 4);  // The resulting homogeneous transformation matrix
    Matrix<calc_t> transT(4, 4); // Transformation matrix from N-1 to N frame

    positionToTransformMatrix(Position_t(), baseT); // Get base matrix
    //DEBUG_STR("Base matrix transform");
    //DEBUG_MATRIX(baseT);
    workT = baseT;

    // Calculation the transformation matrix - T06 = T01 * T12 * T23 * T34 * T45 * T56
    for (int i = 0; i < 6; i++)
    {
        createDHFrameMatrix(_man.theta[i] + t[i], _man.alfa[i], _man.r[i], _man.d[i], transT);
        workT = workT * transT;
    }
    baseT = workT * baseT;

    // Get position by calculated matrix
    transformMatrixToPosition(baseT, out);
    return CalculationResult_t::RESULT_SUCCESSFULL;
}

void KinematicsCalc::inverseTransformMatrix(Matrix<calc_t> &m, Matrix<calc_t> &out)
{
    out.at(0, 0) = m.at(0, 0);
    out.at(0, 1) = m.at(1, 0);
    out.at(0, 2) = m.at(2, 0);
    out.at(0, 3) = -m.at(0, 0) * m.at(0, 3) - m.at(1, 0) * m.at(1, 3) - m.at(2, 0) * m.at(2, 3);

    out.at(1, 0) = m.at(0, 1);
    out.at(1, 1) = m.at(1, 1);
    out.at(1, 2) = m.at(2, 1);
    out.at(1, 3) = -m.at(0, 1) * m.at(0, 3) - m.at(1, 1) * m.at(1, 3) - m.at(2, 1) * m.at(2, 3);

    out.at(2, 0) = m.at(0, 2);
    out.at(2, 1) = m.at(1, 2);
    out.at(2, 2) = m.at(2, 2);
    out.at(2, 3) = -m.at(0, 2) * m.at(0, 3) - m.at(1, 2) * m.at(1, 3) - m.at(2, 2) * m.at(2, 3);

    out.at(3, 0) = 0.0;
    out.at(3, 1) = 0.0;
    out.at(3, 2) = 0.0;
    out.at(3, 3) = 1.0;
}

void KinematicsCalc::transformMatrixToPosition(Matrix<calc_t> &m, Position_t &out)
{
    out.x = m.at(0, 3);
    out.y = m.at(1, 3);
    out.z = m.at(2, 3);
    out.wy = atan2(sqrt(m.at(2, 0) * m.at(2, 0) + m.at(2, 1) * m.at(2, 1)), m.at(2, 2));
    out.wx = atan2(m.at(1, 2) / sin(out.wy), m.at(0, 2) / sin(out.wy));
    out.wz = atan2(m.at(2, 1) / sin(out.wy), -m.at(2, 0) / sin(out.wy));
}

void KinematicsCalc::positionToTransformMatrix(const Position_t &pos, Matrix<calc_t> &out)
{
    out.at(0, 0) = cos(pos.wx) * cos(pos.wy) * cos(pos.wz) - sin(pos.wx) * sin(pos.wz);
    out.at(0, 1) = -cos(pos.wx) * cos(pos.wy) * sin(pos.wz) - sin(pos.wx) * cos(pos.wz);
    out.at(0, 2) = cos(pos.wx) * sin(pos.wy);
    out.at(0, 3) = pos.x;

    out.at(1, 0) = sin(pos.wx) * cos(pos.wy) * cos(pos.wz) + cos(pos.wx) * sin(pos.wz);
    out.at(1, 1) = -sin(pos.wx) * cos(pos.wy) * sin(pos.wz) + cos(pos.wx) * cos(pos.wz);
    out.at(1, 2) = sin(pos.wx) * sin(pos.wy);
    out.at(1, 3) = pos.y;

    out.at(2, 0) = -sin(pos.wy) * cos(pos.wz);
    out.at(2, 1) = sin(pos.wy) * sin(pos.wz);
    out.at(2, 2) = cos(pos.wy);
    out.at(2, 3) = pos.z;

    out.at(3, 0) = 0.0;
    out.at(3, 1) = 0.0;
    out.at(3, 2) = 0.0;
    out.at(3, 3) = 1.0;
}

void KinematicsCalc::createDHFrameMatrix(calc_t theta, calc_t alfa, calc_t r, calc_t d, Matrix<calc_t> &out)
{
    out.at(0, 0) = cos(theta);
    out.at(0, 1) = -sin(theta) * cos(alfa);
    out.at(0, 2) = sin(theta) * sin(alfa);
    out.at(0, 3) = r * cos(theta);

    out.at(1, 0) = sin(theta);
    out.at(1, 1) = cos(theta) * cos(alfa);
    out.at(1, 2) = -cos(theta) * sin(alfa);
    out.at(1, 3) = r * sin(theta);

    out.at(2, 0) = 0.0;
    out.at(2, 1) = sin(alfa);
    out.at(2, 2) = cos(alfa);
    out.at(2, 3) = d;

    out.at(3, 0) = 0.0;
    out.at(3, 1) = 0.0;
    out.at(3, 2) = 0.0;
    out.at(3, 3) = 1.0;
}