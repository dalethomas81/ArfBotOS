/*
    https://github.com/kousheekc/Kinematics

    MIT License

    Copyright (c) 2022 Kousheek Chakraborty

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#include "TcPch.h"
#pragma hdrstop

#include "MatrixUtils.h"

#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

MatrixUtils::MatrixUtils() {
  
}

void MatrixUtils::copy_matrix(double* mat, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j];
        }
    }
}

void MatrixUtils::identity(double* mat, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                mat[n * i + j] = 1;
            }
            else {
                mat[n * i + j] = 0;
            }
        }
    }
}

void MatrixUtils::zero(double* mat, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            mat[c * i + j] = 0;
        }
    }
}

void MatrixUtils::transpose(double* mat, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * j + i] = mat[c * i + j];
        }
    }
}

double MatrixUtils::trace(double* mat, int r) {
    double sum = 0;
    for (int i = 0; i < r; i++) {
        sum += mat[r * i + i]; 
    }
    return sum;
}

int MatrixUtils::inverse(double* A, int n) {
    int pivrow = 0;
    int k, i, j; 
    int pivrows[6];
    double tmp;

    for (k = 0; k < n; k++) {
        tmp = 0;
        for (i = k; i < n; i++) {
            if (abs(A[i * n + k]) >= tmp) {
                tmp = abs(A[i * n + k]);
                pivrow = i;
            }
        }

        if (A[pivrow * n + k] == 0.0f) {
//            Serial.println("Inversion failed due to singular matrix");
            return 0;
        }

        if (pivrow != k) {
            for (j = 0; j < n; j++) {
                tmp = A[k * n + j];
                A[k * n + j] = A[pivrow * n + j];
                A[pivrow * n + j] = tmp;
            }
        }
        pivrows[k] = pivrow; 

        tmp = 1.0f / A[k * n + k];  
        A[k * n + k] = 1.0f;

        for (j = 0; j < n; j++) {
            A[k * n + j] = A[k * n + j] * tmp;
        }

        for (i = 0; i < n; i++) {
            if (i != k) {
                tmp = A[i * n + k];
                A[i * n + k] = 0.0f;
                for (j = 0; j < n; j++) {
                    A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
                }
            }
        }
    }

    for (k = n - 1; k >= 0; k--) {
        if (pivrows[k] != k) {
            for (i = 0; i < n; i++) {
                tmp = A[i * n + k];
                A[i * n + k] = A[i * n + pivrows[k]];
                A[i * n + pivrows[k]] = tmp;
            }
        }
    }
    return 1;
}

void MatrixUtils::pseudo_inverse(double* mat, double* A_t, double* AA_t, double* A_tA, int r, int c, double* result) {
    transpose((double*)mat, r, c, (double*)A_t);
    
    mul_matrix((double*)mat, (double*)A_t, r, c, c, r, (double*)AA_t);
    mul_matrix((double*)A_t, (double*)mat, c, r, r, c, (double*)A_tA);
    
    int AA_t_inv_res = inverse((double*)AA_t, r);
    int A_tA_inv_res = inverse((double*)A_tA, c);

    if (AA_t_inv_res == 1) {
        // A+ = A_t * (A * A_t)^-1
        mul_matrix((double*)A_t, (double*)AA_t, c, r, r, r, (double*)result);
    }
    else {
        mul_matrix((double*)A_tA, (double*)A_t, c, c, c, r, (double*)result);
    }
}

// Transformation matrix methods
void MatrixUtils::get_rot_mat(double* mat, double* rot_mat) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_mat[3 * i + j] = mat[4 * i + j];
        }
    }
}

void MatrixUtils::get_pos_vec(double* mat, double* pos_vec) {
    for (int i = 0; i < 3; i++) {
        pos_vec[i] = mat[4 * i + 3];
    }
}

void MatrixUtils::create_trn_mat(double* rot_mat, double* pos_vec, double* trn_mat) {
    zero((double*)trn_mat, 4, 4);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            trn_mat[4 * i + j] = rot_mat[3 * i + j];
        }
        trn_mat[4 * i + 3] = pos_vec[i];
    }
    trn_mat[4 * 3 + 3] = 1;
}

void MatrixUtils::trn_mat_inverse(double* mat, double* result) {
    double rot_mat[3][3];
    double pos_vec[3];
    double rot_mat_t[3][3];
    double pos_vec_result[3];

    get_rot_mat((double*)mat, (double*)rot_mat);
    get_pos_vec((double*)mat, pos_vec);
    transpose((double*)rot_mat, 3, 3, (double*)rot_mat_t);
    mul_vector((double*)rot_mat_t, pos_vec, 3, 3, pos_vec_result);
    pos_vec_result[0] = -pos_vec_result[0];
    pos_vec_result[1] = -pos_vec_result[1];
    pos_vec_result[2] = -pos_vec_result[2];
    create_trn_mat((double*)rot_mat_t, pos_vec_result, (double*)result);
}

void MatrixUtils::adjoint(double* mat, double* result) {
    double rot_mat[3][3];
    double pos_vec[3];
    double so3[3][3];
    double bottom_left[3][3];

    zero((double*)result, 6, 6);
    get_rot_mat((double*)mat, (double*)rot_mat);
    get_pos_vec((double*)mat, pos_vec);
    vec_to_so3(pos_vec, (double*)so3);

    mul_matrix((double*)so3, (double*)rot_mat, 3, 3, 3, 3, (double*)bottom_left);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * i + j] = rot_mat[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + j] = bottom_left[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + (j + 3)] = rot_mat[i][j];
        }
    }
}

void MatrixUtils::exp3(double* mat, double* result) {
    double w[3];
    double id[3][3];
    double w_mat[3][3];
    double w_mat_sq[3][3];
    double term2[3][3];
    double term3[3][3];

    identity((double*)id, 3);

    so3_to_vec((double*)mat, w);

    if (abs(norm(w)) < 1e-6) {
        copy_matrix((double*)id, 3, 3, (double*)result);
    }
    else {
        double theta = get_angle(w);

        div_scalar((double*)mat, theta, 3, 3, (double*)w_mat);
        mul_matrix((double*)w_mat, (double*)w_mat, 3, 3, 3, 3, (double*)w_mat_sq);


        mul_scalar((double*)w_mat, sin(theta), 3, 3, (double*)term2);
        mul_scalar((double*)w_mat_sq, 1 - cos(theta), 3, 3, (double*)term3);

        add_matrix((double*)id, (double*)term2, 3, 3, result);
        add_matrix((double*)result, (double*)term3, 3, 3, result);
    }
}

void MatrixUtils::exp6(double* mat, double* result) {
    double rot_mat[3][3];
    double pos_vec[3];
    double w[3];
    double id[3][3];
    double w_mat[3][3];
    double w_mat_sq[3][3];
    double result_rot[3][3];
    double result_pos_term1_a[3][3];
    double result_pos_term1_b[3][3];
    double result_pos_term1_c[3][3];
    double result_pos_term1[3][3];
    double result_pos_term2[3];
    double result_pos[3];

    identity((double*)id, 3);

    get_rot_mat((double*)mat, (double*)rot_mat);
    get_pos_vec((double*)mat, pos_vec);
    so3_to_vec((double*)rot_mat, w);

    if (abs(norm(w)) < 1e-6) {
        create_trn_mat((double*)id, pos_vec, (double*)result);
    }
    else {
        double theta = get_angle(w);

        div_scalar((double*)rot_mat, theta, 3, 3, (double*)w_mat);
        exp3((double*)rot_mat, (double*)result_rot);
        mul_matrix((double*)w_mat, (double*)w_mat, 3, 3, 3, 3, (double*)w_mat_sq);

        mul_scalar((double*)id, theta, 3, 3, (double*)result_pos_term1_a);
        mul_scalar((double*)w_mat, 1 - cos(theta), 3, 3, (double*)result_pos_term1_b);
        mul_scalar((double*)w_mat_sq, theta - sin(theta), 3, 3, (double*)result_pos_term1_c);

        add_matrix((double*)result_pos_term1_a, (double*)result_pos_term1_b, 3, 3, (double*)result_pos_term1);
        add_matrix((double*)result_pos_term1, (double*)result_pos_term1_c, 3, 3, (double*)result_pos_term1);

        div_scalar(pos_vec, theta, 1, 3, result_pos_term2);
        mul_vector((double*)result_pos_term1, result_pos_term2, 3, 3, result_pos);

        create_trn_mat((double*)result_rot, result_pos, (double*)result);
    }
}

void MatrixUtils::log3(double* mat, double* result) {
    double acos_input = (trace((double*)mat, 3) - 1.0) / 2.0;

    if (acos_input >= 1) {
        zero((double*)result, 3, 3);
    }
    else if (acos_input <= -1) {
        double w[3];
        double s;
        double so3[3][3];
        
        if (1 + mat[3 * 2 + 2] > 1e-6) {
            w[0] = mat[3 * 0 + 2];
            w[1] = mat[3 * 1 + 2];
            w[2] = 1 + mat[3 * 2 + 2];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 2 + 2]));
            mul_scalar(w, s, 1, 3, w);
        }
        else if (1 + mat[3 * 1 + 1] >= 1e-6) {
            w[0] = mat[3 * 0 + 1];
            w[1] = 1 + mat[3 * 1 + 1];
            w[2] = mat[3 * 2 + 1];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 1 + 1]));
            mul_scalar(w, s, 1, 3, w);
        }
        else {
            w[0] = 1 + mat[3 * 0 + 0];
            w[1] = mat[3 * 1 + 0];
            w[2] = mat[3 * 2 + 0];
            s = 1.0 / sqrt(2.0 * (1 + mat[3 * 0 + 0]));
            mul_scalar(w, s, 1, 3, w);
        }
        mul_scalar(w, M_PI, 1, 3, w);
        vec_to_so3(w, (double*)so3);
        copy_matrix((double*)so3, 3, 3, (double*)result);
    }
    else {
        double mat_t[3][3];
        double term[3][3];
        double theta = acos(acos_input);
        double s = theta / 2.0 / sin(theta);
        transpose((double*)mat, 3, 3, (double*)mat_t);
        sub_matrix((double*)mat, (double*)mat_t, 3, 3, (double*)term);
        mul_scalar((double*)term, s, 3, 3, (double*)result);
    }
}

void MatrixUtils::log6(double* mat, double* result) {
    double rot_mat[3][3];
    double w_mat[3][3];
    bool condition = true;

    get_rot_mat((double*)mat, (double*)rot_mat);
    log3((double*)rot_mat, (double*)w_mat);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (w_mat[i][j] != 0) {
                condition = false;
            }
        }
    }
    if (condition == true) {
        double rot[3][3];
        double vec[3];

        zero((double*)rot, 3, 3);
        vec[0] = mat[4 * 0 + 3];
        vec[1] = mat[4 * 1 + 3];
        vec[2] = mat[4 * 2 + 3];

        create_trn_mat((double*)rot, vec, (double*)result);
        result[4 * 3 + 3] = 0;
    }
    else {
        double theta = acos((trace((double*)rot_mat, 3) - 1.0) / 2.0);
        double w_mat_by_2[3][3];
        double id[3][3];
        double w_mat_sq[3][3];
        double w_mat_sq_by_theta[3][3];
        double s;
        double term1[3][3];
        double term2[3][3];
        double term3[3][3];
        double term4[3];
        double p[3];

        identity((double*)id, 3);
        div_scalar((double*)w_mat, 2.0, 3, 3, (double*)w_mat_by_2);
        mul_matrix((double*)w_mat, (double*)w_mat, 3, 3, 3, 3, (double*)w_mat_sq);
        div_scalar((double*)w_mat_sq, theta, 3, 3, (double*)w_mat_sq_by_theta);
        s = 1.0 / theta - 1.0 / tan(theta / 2.0) / 2.0;
        
        sub_matrix((double*)id, (double*)w_mat_by_2, 3, 3, (double*)term1);
        mul_scalar((double*)w_mat_sq_by_theta, s, 3, 3, (double*)term2);
        add_matrix((double*)term1, (double*)term2, 3, 3, (double*)term3);

        term4[0] = mat[4 * 0 + 3];
        term4[1] = mat[4 * 1 + 3];
        term4[2] = mat[4 * 2 + 3];

        mul_vector((double*)term3, term4, 3, 3, p);

        create_trn_mat((double*)w_mat, p, (double*)result);
        result[4 * 3 + 3] = 0;
    }
}

// Vector methods
double MatrixUtils::norm(double* vec) {
    return sqrt(pow(vec[0],2) + pow(vec[1],2) + pow(vec[2],2));
}

double MatrixUtils::get_angle(double* vec) {
    return norm(vec);
}

// Matrix operators
void MatrixUtils::add_scalar(double* mat, double s, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] + s;
        }
    }
}

void MatrixUtils::sub_scalar(double* mat, double s, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] - s;
        }
    }
}

void MatrixUtils::mul_scalar(double* mat, double s, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] * s;
        }
    }
}

void MatrixUtils::div_scalar(double* mat, double s, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat[c * i + j] / s;
        }
    }
}

void MatrixUtils::add_matrix(double* mat1, double* mat2, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat1[c * i + j] + mat2[c * i + j];
        }
    }
}

void MatrixUtils::sub_matrix(double* mat1, double* mat2, int r, int c, double* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[c * i + j] = mat1[c * i + j] - mat2[c * i + j];
        }
    }
}

void MatrixUtils::mul_matrix(double* mat1, double* mat2, int r1, int c1, int r2, int c2, double* result) {
    for (int i = 0; i < r1; i++) {
        for(int j = 0; j < c2; j++) {
            result[c2 * i + j] = 0;
            for (int k = 0; k < c1; k++) {
                result[c2 * i + j] = result[c2 * i + j] + mat1[c1 * i + k] * mat2[c2 * k + j];
            }
        }
    }
}

void MatrixUtils::mul_vector(double* mat, double* vec, int r, int c, double* result) {
    for (int i = 0; i < c; i++) {
        result[i] = 0;
    }
    
    for(int i = 0; i < r; i++) {
        for(int j = 0; j < c; j++) {
            result[i] = result[i] + (vec[j] * mat[c * i + j]);
        }
    }
}

// Matrix vector methods
void MatrixUtils::vec_to_so3(double* vec, double* result) {
    result[3 * 0 + 0] = 0;
    result[3 * 0 + 1] = -vec[2];
    result[3 * 0 + 2] = vec[1];

    result[3 * 1 + 0] = vec[2];
    result[3 * 1 + 1] = 0;
    result[3 * 1 + 2] = -vec[0];

    result[3 * 2 + 0] = -vec[1];
    result[3 * 2 + 1] = vec[0];
    result[3 * 2 + 2] = 0;
}

void MatrixUtils::so3_to_vec(double* rot_mat, double* result) {
    result[0] = rot_mat[3 * 2 + 1];
    result[1] = rot_mat[3 * 0 + 2];
    result[2] = rot_mat[3 * 1 + 0];
}

void MatrixUtils::se3_to_vec(double* trn_mat, double* result) {
    result[0] = trn_mat[4 * 2 + 1];
    result[1] = trn_mat[4 * 0 + 2];
    result[2] = trn_mat[4 * 1 + 0];
    result[3] = trn_mat[4 * 0 + 3];
    result[4] = trn_mat[4 * 1 + 3];
    result[5] = trn_mat[4 * 2 + 3];
}

void MatrixUtils::vec_to_se3(double* vec, double* result) {
    double v[3] = {vec[0], vec[1], vec[2]};
    double so3_mat[3][3];

    vec_to_so3(v, (double*)so3_mat);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[4 * i + j] = so3_mat[i][j];
        }
    }

    result[4 * 0 + 3] = vec[3];
    result[4 * 1 + 3] = vec[4];
    result[4 * 2 + 3] = vec[5];

    result[4 * 3 + 0] = 0;
    result[4 * 3 + 1] = 0;
    result[4 * 3 + 2] = 0;
    result[4 * 3 + 3] = 0;
}
