#include <Eigen/Dense>
#include "LinearSolve.h"


namespace geo
{
	//glm::vec<6, f32> SolvePointToPlaneSystem(const glm::mat<6, 6, f32>& A, const glm::mat<6, 6, f32>& b)
	//{
    //    Eigen::Matrix<double, 6, 6> A_e;
    //    Eigen::Matrix<double, 6, 1> b_e;
//
    //    // Copy glm -> Eigen
    //    for (int r = 0; r < 6; ++r)
    //    {
    //        A_e(r) = A[r];
    //        for (int c = 0; c < 6; ++c)
    //            b_e(r, c) = A[r][c];
    //    }
//
    //    Eigen::Matrix<double, 6, 1> x = A_e.ldlt().solve(b_e);
//
    //    glm::vec<6, float, glm::packed_highp> result{0.0f};
    //    for (int i = 0; i < 6; ++i)
    //    {
    //        result[i] = x(i);
    //    }
//
    //    return result;
    //}
}