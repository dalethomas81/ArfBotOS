#include "Kinematics.h"

Kinematics::Kinematics(int num_of_joints_) {
    num_of_joints = num_of_joints_;
    num_of_joints_declared = 0;

    mat_utils.zero((double*)initial_end_effector_pose, 4, 4);
    mat_utils.zero((double*)joint_screw_axes, 6, 6);
}

void Kinematics::add_joint_axis(double s1, double s2, double s3, double s4, double s5, double s6) {
    joint_screw_axes[num_of_joints_declared][0] = s1;
    joint_screw_axes[num_of_joints_declared][1] = s2;
    joint_screw_axes[num_of_joints_declared][2] = s3;
    joint_screw_axes[num_of_joints_declared][3] = s4;
    joint_screw_axes[num_of_joints_declared][4] = s5;
    joint_screw_axes[num_of_joints_declared][5] = s6;

    num_of_joints_declared++;
}

void Kinematics::add_initial_end_effector_pose(double m11, double m12, double m13, double m14, double m21, double m22, double m23, double m24, double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44) {
    initial_end_effector_pose[0][0] = m11;
    initial_end_effector_pose[0][1] = m12;
    initial_end_effector_pose[0][2] = m13;
    initial_end_effector_pose[0][3] = m14;

    initial_end_effector_pose[1][0] = m21;
    initial_end_effector_pose[1][1] = m22;
    initial_end_effector_pose[1][2] = m23;
    initial_end_effector_pose[1][3] = m24;

    initial_end_effector_pose[2][0] = m31;
    initial_end_effector_pose[2][1] = m32;
    initial_end_effector_pose[2][2] = m33;
    initial_end_effector_pose[2][3] = m34;
    
    initial_end_effector_pose[3][0] = m41;
    initial_end_effector_pose[3][1] = m42;
    initial_end_effector_pose[3][2] = m43;
    initial_end_effector_pose[3][3] = m44;
}

void Kinematics::forward(double* joint_angles, double* transform) {
    double vec6[6];
    double se3[4][4];
    double exp6[4][4];
    double result[4][4];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform[4 * i + j] = initial_end_effector_pose[i][j];
        }
    }

    for (int i = num_of_joints - 1; i >= 0; i--) {
        mat_utils.mul_scalar(joint_screw_axes[i], joint_angles[i], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (double*)se3);
        mat_utils.exp6((double*)se3, (double*)exp6);
        mat_utils.mul_matrix((double*)exp6, (double*)transform, 4, 4, 4, 4, (double*)result);
        mat_utils.copy_matrix((double*)result, 4, 4, (double*)transform);
    }
}

void Kinematics::inverse(double* transform, double* jac, double* pinv, double* A_t, double* AA_t, double* A_tA, double* initial_joint_angles, double ew, double ev, double max_iterations, double* joint_angles) {
    double Tsb[4][4];
    double Tsb_inv[4][4];
    double Tsb_inv_T[4][4];
    double log6[4][4];
    double vec6[6];
    double adj[6][6];
    double Vs[6];
    double w[3];
    double v[3];
    double pinv_Vs[6];
    bool error;
    int i;

    mat_utils.copy_matrix(initial_joint_angles, 1, num_of_joints, joint_angles);
    forward(joint_angles, (double*)Tsb);
    mat_utils.trn_mat_inverse((double*)Tsb, (double*)Tsb_inv);
    mat_utils.mul_matrix((double*)Tsb_inv, (double*)transform, 4, 4, 4, 4, (double*)Tsb_inv_T);
    mat_utils.log6((double*)Tsb_inv_T, (double*)log6);
    mat_utils.se3_to_vec((double*)log6, vec6);
    mat_utils.adjoint((double*)Tsb, (double*)adj);
    mat_utils.mul_vector((double*)adj, vec6, 6, 6, Vs);

    w[0] = Vs[0];
    w[1] = Vs[1];
    w[2] = Vs[2];

    v[0] = Vs[3];
    v[1] = Vs[4];
    v[2] = Vs[5];

    error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    i = 0;
    
    while (error && i < max_iterations) {
        jacobian(joint_angles, (double*)jac);
        mat_utils.pseudo_inverse((double*)jac, (double*)A_t, (double*)AA_t, (double*)A_tA, 6, num_of_joints, (double*)pinv);
        mat_utils.mul_vector((double*)pinv, Vs, num_of_joints, 6, pinv_Vs);
        mat_utils.add_matrix(joint_angles, pinv_Vs, 1, num_of_joints, joint_angles);

        i = i + 1;

        forward(joint_angles, (double*)Tsb);

        mat_utils.trn_mat_inverse((double*)Tsb, (double*)Tsb_inv);
        mat_utils.mul_matrix((double*)Tsb_inv, (double*)transform, 4, 4, 4, 4, (double*)Tsb_inv_T);
        mat_utils.log6((double*)Tsb_inv_T, (double*)log6);
        mat_utils.se3_to_vec((double*)log6, vec6);
        mat_utils.adjoint((double*)Tsb, (double*)adj);
        mat_utils.mul_vector((double*)adj, vec6, 6, 6, Vs);

        w[0] = Vs[0];
        w[1] = Vs[1];
        w[2] = Vs[2];

        v[0] = Vs[3];
        v[1] = Vs[4];
        v[2] = Vs[5];

        error = (mat_utils.norm(w) > ew) || (mat_utils.norm(v) > ev);
    }
}

void Kinematics::jacobian(double* joint_angles, double* jacobian) {
    double transform[4][4];
    double vec6[6];
    double se3[4][4];
    double exp6[4][4];
    double result[4][4];
    double adj[6][6];
    double jacobian_column[6];

    mat_utils.zero((double*)jacobian, 6, num_of_joints);

    mat_utils.identity((double*)transform, 4);
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < num_of_joints; j++) {
            jacobian[num_of_joints * i + j] = joint_screw_axes[j][i];
        }
    }

    for (int i = 1; i < num_of_joints; i++) {
        mat_utils.mul_scalar(joint_screw_axes[i-1], joint_angles[i-1], 1, 6, vec6);
        mat_utils.vec_to_se3(vec6, (double*)se3);
        mat_utils.exp6((double*)se3, (double*)exp6);
        mat_utils.mul_matrix((double*)transform, (double*)exp6, 4, 4, 4, 4, (double*)result);
        mat_utils.copy_matrix((double*)result, 4, 4, (double*)transform);

        mat_utils.adjoint((double*)transform, (double*)adj);
        mat_utils.mul_vector((double*)adj, joint_screw_axes[i], 6, 6, jacobian_column);
        for (int j = 0; j < 6; j++) {
            jacobian[num_of_joints * j + i] = jacobian_column[j];
        }
    }
}
