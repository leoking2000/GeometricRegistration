#include <glm/gtc/matrix_transform.hpp>
#include "Core/ICPTestRun.h"

static geo::Random rng{ 2026 };

static void TestICP()
{
    geo::SetLogLevel(geo::LogLevel::INFO);
    std::cout << "==== ICP Test ====\n";

    //geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/fox_skull/fox_skull.obj");
    geo::PointCloud3D rect_pc = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 10000, rng, true);

    geo::Mesh bunny = geo::Mesh(RESOURCES_PATH"models/bunny/bunny.obj");
    geo::PointCloud3D bunny_pc = bunny.ToPointCloud();

    geo::Mesh dora_1 = geo::Mesh(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj");
    geo::PointCloud3D dora_pc = dora_1.ToPointCloud();

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
        test::WithOutliers(bunny_pc, { glm::mat3(rotation), translation }, 14405, 2.0f, "Bunny 20% Outliers"),
        test::WithOutliers(bunny_pc, { glm::mat3(rotation), translation }, 4000, 50.0f, "Dora_1 Outliers")
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
}

static void TestDF()
{
    geo::SetLogLevel(geo::LogLevel::INFO);
    std::cout << "==== DistanceField Test ====\n";

    // 1. Create input data
    //geo::Mesh bunny = geo::Mesh(RESOURCES_PATH"models/bunny/bunny.obj");
    geo::Mesh dora_3_med = geo::Mesh(RESOURCES_PATH"models/DoraEmbrasure3_med_final/DoraEmbrasure3_med_final.obj");
    geo::PointCloud3D cloud = dora_3_med.ToPointCloud();

    // 2. Build DF
    geo::DistanceField df;

    const geo::u32 resolution = 128;
    const geo::f32 dTrunc = 2.0f;
    const geo::f32 padding = 0.1f;

    geo::TimingStat buildTime;

    geo::TimePoint startBuild = geo::Clock::now();
    df.Build(cloud, resolution, dTrunc, padding);
    geo::TimePoint endBuild = geo::Clock::now();

    std::cout << "Build Time: " << geo::TimeDifferenceMs(endBuild, startBuild) << " ms\n";

    // 3. Query test (1M samples)
    const geo::u32 NUM_QUERIES = 1000000u;

    geo::BBox bbox = cloud.ComputeBoundingBox();
    bbox.ExpandByFactor(padding);


    geo::TimingStat queryStat;
    geo::f32 sum = 0.0f;

    for (int i = 0; i < NUM_QUERIES; ++i)
    {
        geo::ScopedTimer scope(&queryStat);

        // random point in bbox
        glm::vec3 q(rng.Float(bbox.Min().x, bbox.Max().x), 
                    rng.Float(bbox.Min().y, bbox.Max().y), 
                    rng.Float(bbox.Min().z, bbox.Max().z));

        sum += df(q);
    }

    std::cout << "Query Time (1M): " << queryStat.ToString() << "\n";

    // 4. Sanity check
    std::cout << "Zero-ish test: " << df(cloud.GetPoints()[0]) << "\n";

    std::cout << "Far test: " << df(glm::vec3(100, 100, 100)) << "\n";
    std::cout << "Accumulated Value (ignore): " << sum << "\n";

    std::cout << "================================\n";
}

int main()
{
    //TestICP();
    TestDF();


    return 0;
}
