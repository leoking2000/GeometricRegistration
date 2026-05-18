#include <glm/gtc/matrix_transform.hpp>
#include "Core/ICPTestRun.h"

static geo::Random rng{ 2026 };

//#define RUN_LeastSquaresICP
#define RUN_SparseICP
//#define RUN_EfficientICP

//#define PartialOverlap_Tests
//#define Outliers_Tests

static void TestICP()
{
    geo::SetLogLevel(geo::LogLevel::LOG_INFO);
    std::cout << "==== ICP Test ====\n";

    geo::PointCloud3D rect_pc = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 10000, rng, true);

    geo::Mesh bunny = geo::io::LoadGeometry(RESOURCES_PATH"models/bunny/bunny.obj").ToMesh();
    geo::PointCloud3D bunny_pc = bunny.ToPointCloud();

    geo::Mesh dora_1 = geo::io::LoadGeometry(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj").ToMesh();
    geo::PointCloud3D dora_pc = dora_1.ToPointCloud();

    std::cout << "Models Loaded\n";

    glm::vec3 eulerRot(10.0f, 5.0f, 2.5f);
    glm::vec3 translation(-1.0f, 0.0f, 1.0f);

    //glm::vec3 eulerRot(60.0f, -50.0f, 20.5f);
    //glm::vec3 translation(-1.0f, 0.0f, 1.0f);

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
        test::KnowedTransform(bunny_pc, { glm::mat3(rotation), translation } ,"KnowedTransform Bunny")
#ifdef PartialOverlap_Tests
        ,
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.5f; }, "PartialOverlap Bunny 65% overlap (x axis cut)"),
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.3f; }, "PartialOverlap Bunny 50% overlap (x axis cut)"),
        test::PartialOverlap(bunny_pc, { glm::mat3(rotation), translation },
            [](const glm::vec3& v) -> bool { return v.x >= -1.1f; }, "PartialOverlap Bunny 35% overlap (x axis cut)")
#endif
#ifdef Outliers_Tests
        ,
        test::WithOutliers(bunny_pc, { glm::mat3(rotation), translation }, 14405, 2.0f, "Bunny 20% Outliers"),
        test::WithOutliers(bunny_pc, { glm::mat3(rotation), translation }, 4000, 50.0f, "Dora_1 Outliers")
#endif
    };

    std::cout << "Test Cases Created, Running tests\n";

    for (const auto& test : tests)
    {
        test::ICPTestResult res = {};

#ifdef RUN_LeastSquaresICP
        geo::LeastSquaresICPParameters p_ls;
        p_ls.useNormals = true;
        p_ls.maxIterations = 100;
        res = test::RunLeastSquaresICP(test, p_ls);
        test::PrintResult(res);
#endif

#ifdef RUN_SparseICP
        geo::SparseICPParameters p_spa;
        p_spa.maxIterations = 100;
        p_spa.p = 0.4f;
        res = test::RunSparseICPPointToPlane(test, p_spa);
        test::PrintResult(res);
#endif

#ifdef RUN_EfficientICP
        geo::EfficientICPParams p_eff;
        p_eff.esaIterations = 3000;
        p_eff.esaRestarts = 5;
        p_eff.icpParams.maxIterations = 100;
        p_eff.icpParams.p = 0.4f;
        res = test::RunEfficientICPPointToPlane(test, p_eff);
        test::PrintResult(res);
#endif
    }

    std::cout << "\nAll Done\n";
}

static void TestDF()
{
    geo::SetLogLevel(geo::LogLevel::LOG_DEBUG);
    std::cout << "==== DistanceField Test ====\n";

    // 1. Create input data
    std::cout << "Loading Mesh.................\n";

    //geo::Mesh mesh = geo::io::LoadGeometry(RESOURCES_PATH"models/bunny/bunny.obj").ToMesh();
    //geo::Mesh mesh = geo::io::LoadGeometry(RESOURCES_PATH"models/fox_skull/fox_skull.obj").ToMesh();
    geo::Mesh mesh = geo::io::LoadGeometry(RESOURCES_PATH"models/DoraEmbrasure3_med_final/DoraEmbrasure3_med_final.obj").ToMesh();

    //mesh.Flatten();

    std::cout << "Done\n";

    // 2. Build DF

    geo::DistanceFieldParameters params;
    params.bounding_box = mesh.BoundingBox();
    params.resolution = 256;
    // set the maximum radius to be 1/4 the diagonal of the mesh
    params.max_distance = 0.25f * glm::length(params.bounding_box.Max() - params.bounding_box.Min());

    geo::DistanceField df(params);

    std::cout << "Building Distance Field.................";
    geo::TimingStat buildTime;

    geo::TimePoint startBuild = geo::Clock::now();
    df.Build(mesh);
    geo::TimePoint endBuild = geo::Clock::now();
    std::cout << "Done\n\n";

    std::cout << "Mesh Name: " << mesh.FileName() << "\n";
    std::cout << "Number of Points: " << mesh.VertexCount()  << "\n";
    std::cout << "Number of Triangles: " << mesh.TriangleCount() << "\n";
    std::cout << "DF Build Time: " << geo::TimeDifferenceMs(endBuild, startBuild) << " ms\n";

    // 3. Query test (1M samples)
    const geo::u32 NUM_QUERIES = 1000000u;

    geo::TimingStat queryStat;
    geo::f32 sum = 0.0f;

    for (int i = 0; i < NUM_QUERIES; ++i)
    {
        // random point in bbox
        glm::vec3 q(rng.Float(params.bounding_box.Min().x, params.bounding_box.Max().x),
                    rng.Float(params.bounding_box.Min().y, params.bounding_box.Max().y),
                    rng.Float(params.bounding_box.Min().z, params.bounding_box.Max().z));

        geo::ScopedTimer scope(&queryStat);
        sum += df(q);
    }

    std::cout << "Query Time (1M): " << queryStat.ToString() << "\n";

    // 4. Sanity check
    std::cout << "Zero-ish test: " << df(mesh.Vertex(0)) << "\n";

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
