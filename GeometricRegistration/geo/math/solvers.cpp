#include <Eigen/Dense>
#include "solvers.h"


namespace geo
{
    namespace detail
    {
        static inline glm::mat3 EigenToGlm(const Eigen::Matrix3f& m)
        {
            glm::mat3 result(0.0f);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result[c][r] = m(r, c); // column-major

            return result;
        }

        static inline Eigen::Matrix3f GlmToEigen(const glm::mat3& m)
        {
            Eigen::Matrix3f result;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    result(r, c) = m[c][r];

            return result;
        }
    }

    // solves A*x=b system that is 6x6, symmetric positive-definite
    Vec6 SolveSymmetric6x6(const Mat6& A_in, const Vec6& b_in)
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
        Eigen::LDLT<Eigen::Matrix<f32, 6, 6>> dec(A);

        // Copy back to std::array
        Vec6 result;
        for (int i = 0; i < 6; ++i)
        {
            result[i] = x(i);
        }

        return result;
    }

    SVDResult SVD(const glm::mat3& A)
    {
        Eigen::Matrix3f A_eigen = detail::GlmToEigen(A);

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(A_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);

        SVDResult result{};
        result.U = detail::EigenToGlm(svd.matrixU());
        result.V = detail::EigenToGlm(svd.matrixV());

        Eigen::Vector3f s = svd.singularValues();
        result.S = glm::vec3(s(0), s(1), s(2));

        return result;
    }

    RigidTransform SolveRigidPointToPoint(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target)
    {
        assert(source.size() == target.size());
        assert(source.size() >= 3);

        const size_t N = source.size();

        // Compute centroids
        glm::vec3 centroidSrc(0.0f);
        glm::vec3 centroidTrg(0.0f);
        for (size_t i = 0; i < N; i++)
        {
            centroidSrc += source[i];
            centroidTrg += target[i];
        }
        centroidSrc = centroidSrc / (f32)N;
        centroidTrg = centroidTrg / (f32)N;

        // Compute covariance matrix
        glm::mat3 H(0.0f);
        for (index_t i = 0; i < N; ++i)
        {
            glm::vec3 p = source[i] - centroidSrc;
            glm::vec3 q = target[i] - centroidTrg;
            H += glm::outerProduct(p, q);
        }

        // SVD
        SVDResult svd = SVD(H);

        glm::mat3 Ut = glm::transpose(svd.U);
        glm::mat3 R = svd.V * Ut;

        // Reflection fix
        if (glm::determinant(R) < 0.0f)
        {
            svd.V[2] *= -1.0f;
            R = svd.V * Ut;
        }

        glm::vec3 t = centroidTrg - R * centroidSrc;

        return { R, t };
    }

    RigidTransform SolveRigidPointToPointWeighted(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target, const std::vector<f32>& weights)
    {
        assert(source.size() == target.size());
        assert(weights.size() == source.size());
        assert(source.size() >= 3);

        const size_t N = source.size();

        f32 sumW = 0.0;
        glm::vec3 centroidSrc(0.0);
        glm::vec3 centroidTgt(0.0);

        for (index_t i = 0; i < N; i++)
        {
            f32 w = weights[i];
            sumW += w;
            centroidSrc += w * source[i];
            centroidTgt += w * target[i];
        }

        if (sumW <= 0.0)
        {
            return { glm::mat3(1.0f), glm::vec3(0.0f) };
        }

        centroidSrc /= sumW;
        centroidTgt /= sumW;

        glm::mat3 H(0.0);

        for (index_t i = 0; i < N; ++i)
        {
            const f32 w = weights[i];

            glm::vec3 p = source[i] - centroidSrc;
            glm::vec3 q = target[i] - centroidTgt;

            H += w * glm::outerProduct(p, q);
        }

        SVDResult svd = SVD(H);
        glm::mat3 Ut = glm::transpose(svd.U);
        glm::mat3 R = svd.V * Ut;

        if (glm::determinant(R) < 0.0f)
        {
            svd.V[2] *= -1.0f;
            R = svd.V * Ut;
        }

        glm::vec3 t = glm::vec3(centroidTgt) - R * glm::vec3(centroidSrc);

        return { R, t };
    }

    static inline glm::mat3 Skew(const glm::vec3& v)
    {
        return glm::mat3{
            { 0.0f, -v.z, v.y },
            { v.z, 0.0f, -v.x },
            { -v.y, v.x, 0.0f }
        };
    }

    static inline glm::mat3 Rodrigues(const glm::vec3& omega)
    {
        float theta = glm::length(omega);
        if (theta < 1e-8f) return glm::mat3(1.0f) + Skew(omega);
        glm::vec3 k = omega / theta;
        glm::mat3 K = Skew(k); // Skew is cross-product matrix
        return glm::mat3(1.0f) + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
    }

    RigidTransform SolveRigidPointToPlane(const std::vector<glm::vec3>& source, const std::vector<glm::vec3>& target, const std::vector<glm::vec3>& normals)
    {
        assert(source.size() == target.size());
        assert(source.size() >= 3);

        const size_t N = source.size();

        // Build 6x6 system
        geo::Mat6 AtA{};
        geo::Vec6 Atb{};

        for (index_t i = 0; i < N; ++i)
        {
            const glm::vec3& p = source[i];
            const glm::vec3& q = target[i];

            // Get normal of target point
            glm::vec3 n = normals[i];

            // Compute b_i = dot(n, q - p)
            f32 bi = glm::dot(n, q - p);

            // Compute rotation part: cross(n, p)
            glm::vec3 crossnp = glm::cross(n, p);

            // Fill AtA and Atb
            std::array<f32, 6> row{ crossnp.x, crossnp.y, crossnp.z, n.x, n.y, n.z };

            // Accumulate AtA = sum row^T * row
            for (int r = 0; r < 6; ++r)
                for (int c = 0; c < 6; ++c)
                    AtA[r][c] += row[r] * row[c];

            // Accumulate Atb = sum row * b_i
            for (int r = 0; r < 6; ++r)
                Atb[r] += row[r] * bi;
        }

        // Solve 6x6 system
        geo::Vec6 x = SolveSymmetric6x6(AtA, Atb);

        glm::vec3 rotVec(x[0], x[1], x[2]);
        glm::vec3 t(x[3], x[4], x[5]);

        // Convert small rotation vector to rotation matrix
        glm::mat3 R = Rodrigues(rotVec);

        return { R, t };
    }

    RigidTransform SolveRigidPointToPlaneShifted(
        const std::vector<glm::vec3>& source,
        const std::vector<glm::vec3>& target,
        const std::vector<glm::vec3>& normals,
        const std::vector<f32>& offsets)
    {
        assert(source.size() == target.size());
        assert(target.size() == normals.size());
        assert(normals.size() == offsets.size());
        assert(source.size() >= 3);

        const size_t N = source.size();

        Mat6 AtA{};
        Vec6 Atb{};

        for (size_t i = 0; i < N; ++i)
        {
            const glm::vec3& p = source[i];
            const glm::vec3& q = target[i];
            const glm::vec3& n = normals[i];
            const f32 c        = offsets[i];

            const glm::vec3 rotJac = glm::cross(n, p);

            const std::array<f32, 6> row{
                rotJac.x, rotJac.y, rotJac.z,
                n.x, n.y, n.z
            };

            const f32 bi = glm::dot(n, q - p) + c;

            for (int r = 0; r < 6; ++r)
            {
                for (int col = 0; col < 6; ++col)
                {
                    AtA[r][col] += row[r] * row[col];
                }
                Atb[r] += row[r] * bi;
            }
        }

        const Vec6 x = SolveSymmetric6x6(AtA, Atb);

        const glm::vec3 omega(x[0], x[1], x[2]);
        const glm::vec3 t(x[3], x[4], x[5]);

        const glm::mat3 R = Rodrigues(omega);
        return { R, t };
    }
}