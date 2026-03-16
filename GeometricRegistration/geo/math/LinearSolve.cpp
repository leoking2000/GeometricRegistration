#include <Eigen/Dense>
#include "LinearSolve.h"


namespace geo
{
    // solves A*x=b system that is 6x6
    Vec6 Solve6x6(Mat6 A_in, Vec6 b_in)
    {
        Eigen::Matrix<f32, 6, 6> A;
        Eigen::Matrix<f32, 6, 1> b;

        // Copy std::array to Eigen
        for (int i = 0; i < 6; ++i)
        {
            b(i) = b_in[i];
            for (int j = 0; j < 6; ++j)
                A(i, j) = A_in[i][j];
        }

        // Solve using LDLT (stable for symmetric positive-definite)
        Eigen::Matrix<f32, 6, 1> x = A.ldlt().solve(b);

        // Copy back to std::array
        Vec6 result;
        for (int i = 0; i < 6; ++i)
        {
            result[i] = x(i);
        }

        return result;
    }
}