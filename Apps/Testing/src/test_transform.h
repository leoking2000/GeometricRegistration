#pragma once
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <geo/math/RigidTransform.h>
#include "ExpectNear.h"

glm::mat3 RotationZ(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    return glm::mat3(
        glm::vec3(c, s, 0.0f),
        glm::vec3(-s, c, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f)
    );
}

glm::mat3 RotationX(float radians)
{
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    return glm::mat3(
        glm::vec3(1.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, c, s),
        glm::vec3(0.0f, -s, c)
    );
}


TEST(RigidTransformTests, IdentityTransformPointReturnsSamePoint)
{
    const geo::RigidTransform T = geo::RigidTransform::Identity();
    const glm::vec3 p(1.5f, -2.0f, 3.25f);

    const glm::vec3 result = T.TransformPoint(p);

    ExpectVec3Near(result, p);
}

TEST(RigidTransformTests, IdentityTransformNormalReturnsSameNormal)
{
    const geo::RigidTransform T = geo::RigidTransform::Identity();
    const glm::vec3 n(0.0f, 1.0f, 0.0f);

    const glm::vec3 result = T.TransformNormal(n);

    ExpectVec3Near(result, n);
}

TEST(RigidTransformTests, TransformPointAppliesRotationAndTranslation)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(10.0f, 0.0f, 5.0f);

    const glm::vec3 p(1.0f, 0.0f, 0.0f);
    const glm::vec3 expected(10.0f, 1.0f, 5.0f);

    const glm::vec3 result = T.TransformPoint(p);

    ExpectVec3Near(result, expected);
}

TEST(RigidTransformTests, TransformNormalAppliesRotationOnly)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(100.0f, 200.0f, 300.0f);

    const glm::vec3 n(1.0f, 0.0f, 0.0f);
    const glm::vec3 expected(0.0f, 1.0f, 0.0f);

    const glm::vec3 result = T.TransformNormal(n);

    ExpectVec3Near(result, expected);
}

TEST(RigidTransformTests, InverseUndoesPointTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(0.7f);
    T.translation = glm::vec3(3.0f, -2.0f, 4.0f);

    const glm::vec3 p(1.0f, 2.0f, -1.0f);

    const glm::vec3 transformed = T.TransformPoint(p);
    const glm::vec3 recovered = T.ComputeInverse().TransformPoint(transformed);

    ExpectVec3Near(recovered, p, 1e-4f);
}

TEST(RigidTransformTests, InverseUndoesNormalTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationX(0.35f);
    T.translation = glm::vec3(7.0f, 8.0f, 9.0f);

    const glm::vec3 n = glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));

    const glm::vec3 transformed = T.TransformNormal(n);
    const glm::vec3 recovered = T.ComputeInverse().TransformNormal(transformed);

    ExpectVec3Near(recovered, n, 1e-4f);
}

TEST(RigidTransformTests, ComposeMatchesSequentialApplication)
{
    geo::RigidTransform A;
    A.rotation = RotationZ(0.5f);
    A.translation = glm::vec3(1.0f, 2.0f, 3.0f);

    geo::RigidTransform B;
    B.rotation = RotationX(-0.25f);
    B.translation = glm::vec3(-4.0f, 0.5f, 2.0f);

    const geo::RigidTransform C = geo::RigidTransform::Compose(A, B);

    const glm::vec3 p(2.0f, -1.0f, 0.25f);

    const glm::vec3 sequential = A.TransformPoint(B.TransformPoint(p));
    const glm::vec3 composed = C.TransformPoint(p);

    ExpectVec3Near(composed, sequential, 1e-4f);
}

TEST(RigidTransformTests, InverseOfIdentityIsIdentity)
{
    const geo::RigidTransform I = geo::RigidTransform::Identity();
    const geo::RigidTransform inv = I.ComputeInverse();

    ExpectMat3Near(inv.rotation, glm::mat3(1.0f));
    ExpectVec3Near(inv.translation, glm::vec3(0.0f));
}

TEST(RigidTransformTests, ToMat4MatchesPointTransform)
{
    geo::RigidTransform T;
    T.rotation = RotationZ(glm::half_pi<float>());
    T.translation = glm::vec3(2.0f, 3.0f, 4.0f);

    const glm::vec3 p(1.0f, 0.0f, 5.0f);

    const glm::vec3 expected = T.TransformPoint(p);

    const glm::mat4 M = T.ToMat4();
    const glm::vec4 hp = M * glm::vec4(p, 1.0f);
    const glm::vec3 result(hp.x, hp.y, hp.z);

    ExpectVec3Near(result, expected, 1e-5f);
}


