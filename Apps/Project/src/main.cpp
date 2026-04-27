#include <glm/gtc/matrix_transform.hpp>
#include "Core/ICPTestRun.h"

static geo::Random rng{ 2026 };

int main()
{
    geo::SetLogLevel(geo::LogLevel::INFO);

    // get a mesh
    geo::Mesh bunny = geo::Mesh(RESOURCES_PATH"models/bunny/bunny.obj");
    //geo::Mesh dora_1 = geo::Mesh(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj");
    //geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/fox_skull/fox_skull.obj");

    geo::PointCloud3D rect_pc = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 10000, rng, true);
    geo::PointCloud3D bunny_pc = bunny.ToPointCloud();

    glm::vec3 eulerRot(10.0f, 5.0f, 2.5f);
    glm::vec3 translation(-1.0f, 0.0f, 1.0f);

    glm::mat4 Rx = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.x),
        glm::vec3(1, 0, 0));
    glm::mat4 Ry = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.y),
        glm::vec3(0, 1, 0));
    glm::mat4 Rz = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.z),
        glm::vec3(0, 0, 1));

    glm::mat4 rotation = Ry * Rx * Rz;

    std::vector<test::ICPTestCase> tests = {
        test::KnowedTransform(rect_pc, { glm::mat3(rotation), translation } ,"KnowedTransform Rect"),
        test::KnowedTransform(bunny_pc, { glm::mat3(rotation), translation } ,"KnowedTransform Bunny"),
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.5f; }, "PartialOverlap Bunny 65% overlap (x axis cut)"),
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.3f; }, "PartialOverlap Bunny 50% overlap (x axis cut)"),
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.1f; }, "PartialOverlap Bunny 35% overlap (x axis cut)"),
        test::WithOutliers(bunny_pc, { glm::mat3(rotation), translation }, 14405, 2.0f, "Bunny 20% Outliers")
    };

    for (const auto& test : tests)
    {
        geo::LeastSquaresICPParameters p_ls;
        p_ls.useNormals = true;
        p_ls.maxIterations = 100;
        auto res = test::RunLeastSquaresICP(test, p_ls);
        test::PrintResult(res);

        geo::SparseICPParameters p_spa;
        p_spa.p = 0.4f;
        res = test::RunSparseICPPointToPlane(test);
        test::PrintResult(res);
    }

    return 0;
}
