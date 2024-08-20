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
#pragma once

#ifndef MATRIXUTILS_h
#define MATRIXUTILS_h

#include <array>
#include <vector>
#include <math.h>

class MatrixUtils {
    private:

    public:
        MatrixUtils();
        
        // General matrix methods
        void copy_matrix(double* mat, int r, int c, double* result);
        void identity(double* mat, int n);
        void zero(double* mat, int r, int c);
        void transpose(double* mat, int r, int c, double* result);
        double trace(double* mat, int r);
        int inverse(double* A, int n);
        void pseudo_inverse(double* mat, double* A_t, double* AA_t, double* A_tA, int r, int c, double* result);

        // Transformation matrix methods
        void get_rot_mat(double* mat, double* rot_mat);
        void get_pos_vec(double* mat, double* pos_vec);
        void create_trn_mat(double* rot_mat, double* pos_vec, double* trn_mat);
        void trn_mat_inverse(double* mat, double* result);
        void adjoint(double* mat, double* result);
        void exp3(double* mat, double* result);
        void exp6(double* mat, double* result);
        void log3(double* mat, double* result);
        void log6(double* mat, double* result);

        // Vector Methods
        double norm(double* vec);
        double get_angle(double* vec);
        
        // Matrix operators
        void add_scalar(double* mat, double s, int r, int c, double* result);
        void sub_scalar(double* mat, double s, int r, int c, double* result);
        void mul_scalar(double* mat, double s, int r, int c, double* result);
        void div_scalar(double* mat, double s, int r, int c, double* result);
        void add_matrix(double* mat1, double* mat2, int r, int c, double* result);
        void sub_matrix(double* mat1, double* mat2, int r, int c, double* result);
        void mul_matrix(double* mat1, double* mat2, int r1, int c1, int r2, int c2, double* result);
        void mul_vector(double* mat1, double* vec, int r, int c, double* result);

        // Matrix vector methods
        void vec_to_so3(double* vec, double* result);
        void so3_to_vec(double* rot_mat, double* result);
        void vec_to_se3(double* vec, double* result);
        void se3_to_vec(double* trn_mat, double* result);
};

#endif