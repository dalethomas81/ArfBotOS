#include "Kinematics.h"

KinematicsCalc::KinematicsCalc(const Manipulator_t<> &man)
    : _man(man)
{
}

KinematicsCalc::KinematicsCalc(Manipulator_t<> &&man)
    : _man(std::move(man))
{
/*#ifdef DEBUG_DH

    std::cout << "Denavit-Hanterberg parameters\n";
    std::cout << "Theta\tAlfa\tR\tD\t\n";
    for (size_t i = 0; i < DEFAULT_DOF; i++)
    {
        std::cout << _man.theta[i] << '\t' << _man.alfa[i] << '\t' << _man.r[i] << '\t' << _man.d[i] << std::endl;
    }
    std::cout << "===================\n";

#endif*/
}

CalculationResult_t KinematicsCalc::inverseKinematics(const Position_t &Xik, std::vector<calc_t> &Jik)
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

    // Calculate T36 transform matrix
    Matrix<calc_t> t_n_to_n1(4, 4), t_n1_to_n2(4, 4), t_n2_to_n3(4, 4), invT03(4, 4);
    // T03 = T01 * T12 * T23, there t_n_to_n1 = T01, t_n1_to_n2 = T12 and T23(overwriting)
    createDHFrameMatrix(_man.theta[0] + Jik[0], _man.alfa[0], _man.r[0], _man.d[0], t_n_to_n1);
    createDHFrameMatrix(_man.theta[1] + Jik[1], _man.alfa[1], _man.r[1], _man.d[1], t_n1_to_n2);
    createDHFrameMatrix(_man.theta[2] + Jik[2], _man.alfa[2], _man.r[2], _man.d[2], t_n2_to_n3);
    Matrix<calc_t> t03 = t_n_to_n1 * t_n1_to_n2 * t_n2_to_n3;
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

    //
    double Zero = 0.0001;
    for (int _i = 0; _i <= 5; _i++) {
        if (Jik[_i] < Zero && Jik[_i] > -Zero || isnan(Jik[_i])) {
            Jik[_i] = 0.0;
        }
    }

    //DEBUG_STR("Inverse kinematics end")

    return CalculationResult_t::RESULT_SUCCESSFULL;
}

CalculationResult_t KinematicsCalc::forwardKinematics(const std::vector<calc_t> &t, Position_t &out)
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

    //
    //double Zero = 0.0001;
    //if (out.x < Zero && out.x > -Zero || isnan(out.x)) {
    //    out.x = 0.0;
    //}
    //if (out.y < Zero && out.y > -Zero || isnan(out.y)) {
    //    out.y = 0.0;
    //}
    //if (out.z < Zero && out.z > -Zero || isnan(out.z)) {
    //    out.z = 0.0;
    //}
    //if (out.wx < Zero && out.wx > -Zero || isnan(out.wx)) {
    //    out.wx = 0.0;
    //}
    //if (out.wy < Zero && out.wy > -Zero || isnan(out.wy)) {
    //    out.wy = 0.0;
    //}
    //if (out.wz < Zero && out.wz > -Zero || isnan(out.wz)) {
    //    out.wz = 0.0;
    //}

    return CalculationResult_t::RESULT_SUCCESSFULL;
}

void KinematicsCalc::orthogonalize(Matrix<calc_t> &m) {
    // Gram-Schmidt process to orthogonalize the rotation part
    for (int i = 0; i < 3; ++i) {
        // Normalize the i-th column
        double norm = 0.0;
        for (int j = 0; j < 3; ++j) {
            norm += m.at(j, i) * m.at(j, i);
        }
        norm = sqrt(norm);
        for (int j = 0; j < 3; ++j) {
            m.at(j, i) /= norm;
        }

        // Orthogonalize the remaining columns
        for (int k = i + 1; k < 3; ++k) {
            double dot_product = 0.0;
            for (int j = 0; j < 3; ++j) {
                dot_product += m.at(j, i) * m.at(j, k);
            }
            for (int j = 0; j < 3; ++j) {
                m.at(j, k) -= dot_product * m.at(j, i);
            }
        }
    }
}

void KinematicsCalc::inverseTransformMatrix(Matrix<calc_t> &m, Matrix<calc_t> &out)
{
    // Orthogonalize the rotation part
    orthogonalize(m);

    // Transpose the rotation part
    out.at(0, 0) = m.at(0, 0);
    out.at(0, 1) = m.at(1, 0);
    out.at(0, 2) = m.at(2, 0);
    out.at(1, 0) = m.at(0, 1);
    out.at(1, 1) = m.at(1, 1);
    out.at(1, 2) = m.at(2, 1);
    out.at(2, 0) = m.at(0, 2);
    out.at(2, 1) = m.at(1, 2);
    out.at(2, 2) = m.at(2, 2);

    // Negate the translation part
    out.at(0, 3) = -m.at(0, 0) * m.at(0, 3) - m.at(1, 0) * m.at(1, 3) - m.at(2, 0) * m.at(2, 3);
    out.at(1, 3) = -m.at(0, 1) * m.at(0, 3) - m.at(1, 1) * m.at(1, 3) - m.at(2, 1) * m.at(2, 3);
    out.at(2, 3) = -m.at(0, 2) * m.at(0, 3) - m.at(1, 2) * m.at(1, 3) - m.at(2, 2) * m.at(2, 3);

    // Set the homogeneous coordinate
    out.at(3, 0) = 0.0;
    out.at(3, 1) = 0.0;
    out.at(3, 2) = 0.0;
    out.at(3, 3) = 1.0;


    // original
    //out.at(0, 0) = m.at(0, 0);
    //out.at(0, 1) = m.at(1, 0);
    //out.at(0, 2) = m.at(2, 0);
    //out.at(0, 3) = -m.at(0, 0) * m.at(0, 3) - m.at(1, 0) * m.at(1, 3) - m.at(2, 0) * m.at(2, 3);

    //out.at(1, 0) = m.at(0, 1);
    //out.at(1, 1) = m.at(1, 1);
    //out.at(1, 2) = m.at(2, 1);
    //out.at(1, 3) = -m.at(0, 1) * m.at(0, 3) - m.at(1, 1) * m.at(1, 3) - m.at(2, 1) * m.at(2, 3);

    //out.at(2, 0) = m.at(0, 2);
    //out.at(2, 1) = m.at(1, 2);
    //out.at(2, 2) = m.at(2, 2);
    //out.at(2, 3) = -m.at(0, 2) * m.at(0, 3) - m.at(1, 2) * m.at(1, 3) - m.at(2, 2) * m.at(2, 3);

    //out.at(3, 0) = 0.0;
    //out.at(3, 1) = 0.0;
    //out.at(3, 2) = 0.0;
    //out.at(3, 3) = 1.0;


    // copiloyt
    //// Calculate the determinant of the 3x3 rotation part
    //float det = m.at(0, 0) * (m.at(1, 1) * m.at(2, 2) - m.at(2, 1) * m.at(1, 2)) -
    //    m.at(0, 1) * (m.at(1, 0) * m.at(2, 2) - m.at(2, 0) * m.at(1, 2)) +
    //    m.at(0, 2) * (m.at(1, 0) * m.at(2, 1) - m.at(2, 0) * m.at(1, 1));

    //if (det == 0) {
    //    //cerr << "Matrix is singular and cannot be inverted." << endl;
    //    return;
    //}

    //float invDet = 1.0 / det;

    //// Calculate the inverse of the 3x3 rotation part
    //out.at(0, 0) = invDet * (m.at(1, 1) * m.at(2, 2) - m.at(2, 1) * m.at(1, 2));
    //out.at(0, 1) = invDet * (m.at(0, 2) * m.at(2, 1) - m.at(0, 1) * m.at(2, 2));
    //out.at(0, 2) = invDet * (m.at(0, 1) * m.at(1, 2) - m.at(0, 2) * m.at(1, 1));
    //out.at(1, 0) = invDet * (m.at(1, 2) * m.at(2, 0) - m.at(1, 0) * m.at(2, 2));
    //out.at(1, 1) = invDet * (m.at(0, 0) * m.at(2, 2) - m.at(0, 2) * m.at(2, 0));
    //out.at(1, 2) = invDet * (m.at(1, 0) * m.at(0, 2) - m.at(0, 0) * m.at(1, 2));
    //out.at(2, 0) = invDet * (m.at(1, 0) * m.at(2, 1) - m.at(2, 0) * m.at(1, 1));
    //out.at(2, 1) = invDet * (m.at(2, 0) * m.at(0, 1) - m.at(0, 0) * m.at(2, 1));
    //out.at(2, 2) = invDet * (m.at(0, 0) * m.at(1, 1) - m.at(1, 0) * m.at(0, 1));

    //// Calculate the inverse of the translation part
    //out.at(0, 3) = -(out.at(0, 0) * m.at(0, 3) + out.at(0, 1) * m.at(1, 3) + out.at(0, 2) * m.at(2, 3));
    //out.at(1, 3) = -(out.at(1, 0) * m.at(0, 3) + out.at(1, 1) * m.at(1, 3) + out.at(1, 2) * m.at(2, 3));
    //out.at(2, 3) = -(out.at(2, 0) * m.at(0, 3) + out.at(2, 1) * m.at(1, 3) + out.at(2, 2) * m.at(2, 3));

    //// Set the last row of the inverse matrix
    //out.at(3, 0) = 0;
    //out.at(3, 1) = 0;
    //out.at(3, 2) = 0;
    //out.at(3, 3) = 1;

}

void KinematicsCalc::transformMatrixToPosition(Matrix<calc_t> &m, Position_t &out)
{
    // Extract the position
    out.x = m.at(0, 3);
    out.y = m.at(1, 3);
    out.z = m.at(2, 3);

    // original
    out.wy = atan2(sqrt(m.at(2, 0) * m.at(2, 0) + m.at(2, 1) * m.at(2, 1)), m.at(2, 2));
    out.wx = atan2(m.at(1, 2) / sin(out.wy), m.at(0, 2) / sin(out.wy));
    out.wz = atan2(m.at(2, 1) / sin(out.wy), -m.at(2, 0) / sin(out.wy));
    
    //// https://github.com/RobinCPC/Robot_Arm_Kinematics_Lib/blob/master/src/kin/kinematic_chain.cpp#L99
    //double eps = 2.220446049250313e-16;     // to check if close to zero
    //if (fabs(m.at(0, 0)) < eps && fabs(m.at(1, 0)) < eps)
    //{
    //    out.wz = 0;
    //    out.wy = atan2(-m.at(2, 0), m.at(0, 0));
    //    out.wx = atan2(-m.at(1, 2), m.at(1, 1));
    //}
    //else
    //{
    //    out.wz = atan2(m.at(1, 0), m.at(0, 0));
    //    double sr = sin(out.wz);
    //    double cr = cos(out.wz);
    //    out.wy = atan2(-m.at(2, 0), cr * m.at(0, 0) + sr * m.at(1, 0));
    //    out.wx = atan2(sr * m.at(0, 2) - cr * m.at(1, 2), cr * m.at(1, 1) - sr * m.at(0, 1));
    //}

    // copilot
    //// Extract the orientation (roll, pitch, yaw)
    //out.wy = atan2(-m.at(2,0), sqrt(pow(m.at(0,0), 2) + pow(m.at(1,0), 2)));
    //if (cos(out.wy) != 0) {
    //    out.wx = atan2(m.at(2,1), m.at(2,2));
    //    out.wz = atan2(m.at(1,0), m.at(0,0));
    //}
    //else {
    //    out.wx = atan2(-m.at(1,2), m.at(1,1));
    //    out.wz = 0;
    //}
    
}

void KinematicsCalc::positionToTransformMatrix(const Position_t &pos, Matrix<calc_t> &out)
{
    // original
    //out.at(0, 0) = cos(pos.wx) * cos(pos.wy) * cos(pos.wz) - sin(pos.wx) * sin(pos.wz);
    //out.at(0, 1) = -cos(pos.wx) * cos(pos.wy) * sin(pos.wz) - sin(pos.wx) * cos(pos.wz);
    //out.at(0, 2) = cos(pos.wx) * sin(pos.wy);
    //out.at(0, 3) = pos.x;

    //out.at(1, 0) = sin(pos.wx) * cos(pos.wy) * cos(pos.wz) + cos(pos.wx) * sin(pos.wz);
    //out.at(1, 1) = -sin(pos.wx) * cos(pos.wy) * sin(pos.wz) + cos(pos.wx) * cos(pos.wz);
    //out.at(1, 2) = sin(pos.wx) * sin(pos.wy);
    //out.at(1, 3) = pos.y;

    //out.at(2, 0) = -sin(pos.wy) * cos(pos.wz);
    //out.at(2, 1) = sin(pos.wy) * sin(pos.wz);
    //out.at(2, 2) = cos(pos.wy);
    //out.at(2, 3) = pos.z;

    //out.at(3, 0) = 0.0;
    //out.at(3, 1) = 0.0;
    //out.at(3, 2) = 0.0;
    //out.at(3, 3) = 1.0;

    out.at(0, 0) = cos(pos.wy) * cos(pos.wz);
    out.at(0, 1) = -cos(pos.wx) * sin(pos.wz) + sin(pos.wx) * sin(pos.wy) * cos(pos.wz);
    out.at(0, 2) = sin(pos.wx) * sin(pos.wz) + cos(pos.wx) * sin(pos.wy) * cos(pos.wz);
    out.at(0, 3) = pos.x;

    out.at(1, 0) = cos(pos.wy) * sin(pos.wz);
    out.at(1, 1) = cos(pos.wx) * cos(pos.wz) + sin(pos.wx) * sin(pos.wy) * sin(pos.wz);
    out.at(1, 2) = -sin(pos.wx) * cos(pos.wz) + cos(pos.wx) * sin(pos.wy) * sin(pos.wz);
    out.at(1, 3) = pos.y;

    out.at(2, 0) = -sin(pos.wy);
    out.at(2, 1) = sin(pos.wx) * cos(pos.wy);
    out.at(2, 2) = cos(pos.wx) * cos(pos.wy);
    out.at(2, 3) = pos.z;

    out.at(3, 0) = 0.0;
    out.at(3, 1) = 0.0;
    out.at(3, 2) = 0.0;
    out.at(3, 3) = 1.0;

    //// Calculate the rotation matrix components
    //float cosRoll = cos(pos.wx);
    //float sinRoll = sin(pos.wx);
    //float cosPitch = cos(pos.wy);
    //float sinPitch = sin(pos.wy);
    //float cosYaw = cos(pos.wz);
    //float sinYaw = sin(pos.wz);

    //// Fill the transformation matrix
    //out.at(0, 0) = cosYaw * cosPitch;
    //out.at(0, 1) = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    //out.at(0, 2) = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    //out.at(0, 3) = pos.x;

    //out.at(1, 0) = sinYaw * cosPitch;
    //out.at(1, 1) = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
    //out.at(1, 2) = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
    //out.at(1, 3) = pos.y;

    //out.at(2, 0) = -sinPitch;
    //out.at(2, 1) = cosPitch * sinRoll;
    //out.at(2, 2) = cosPitch * cosRoll;
    //out.at(2, 3) = pos.z;

    //out.at(3, 0) = 0;
    //out.at(3, 1) = 0;
    //out.at(3, 2) = 0;
    //out.at(3, 3) = 1;
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