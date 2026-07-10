#include "UnitTests/UnitTests.h"
#include "TestSuite/TestSuitePSA.h"

using namespace tests;

static constexpr u32 SEED = 2026;

//#define RUN_PartialScans
#define RUN_UnitTests
//#define RUN_DFTEST

#define RUN_SparseICP
#define RUN_EfficientICP


static void RunPartialScansAlignmentTests()
{
	std::cout << "====== Partial Scans Alignment Tests ======\n";

	std::cout << "Loading 3D Models...\n";
    Model model = CreateModelFromOBJ(RESOURCES_PATH"models/owl/owl-decimate10pc-textured.ply", 128);
	//Model model = CreateModelFromOBJ(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj", 128);
    //Model model = CreateModelFromOBJ(RESOURCES_PATH"models/fox_skull/fox_skull.obj", 256);
    //Model model = CreateModelFromOBJ(RESOURCES_PATH"models/DoraEmbrasure3_med_final/DoraEmbrasure3_med_final.obj", 256);
	std::cout << "Model Loaded\n";

	std::cout << "Creating Test Suite...";

	glm::vec3 eulerRot(60.0f, -50.0f, 20.5f);
	glm::vec3 translation(-1.0f, 3.0f, 1.0f);

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
    core::RigidTransform gt = { glm::mat3(rotation), translation };

	TestSuitePSA suite;

	suite.AddFullOverlap(&model, gt, SEED);
	suite.AddHalfOverlap(&model, gt, SEED);
	suite.AddQuarterOverlap(&model, gt, SEED);
	suite.AddWithOutliers(&model, 0.1f, gt, SEED);
	suite.AddWithNoise(&model, 0.05f, gt, SEED);
	suite.AddHard(&model, gt, SEED);


	std::cout << "Done\n";

	for (auto& test : suite)
	{
#ifdef RUN_SparseICP
		geo::SparseICPParameters p_spa;
		p_spa.maxIterations = 100;
		p_spa.p = 0.4f;
		TestResult result_spa = RunSparseICPPointToPlane(test, p_spa);

		LogTestResult(result_spa);
#endif // RUN_SparseICP

	std::cout << "=======================================================\n";

#ifdef RUN_EfficientICP
		geo::EfficientICPParams p_eff;
		p_eff.seed = SEED;
		p_eff.esaIterations = 5000;

		p_eff.icpParams.maxIterations = 25;
		p_eff.icpParams.p = 0.4f;
		TestResult result_eff = RunEfficientICPPointToPlane(test, p_eff);

		LogTestResult(result_eff);
#endif // RUN_EfficientICP
	}


	std::cout << "\n";
}

static void TestDF()
{
    core::Random rng{ SEED };

    std::cout << "==== DistanceField Test ====\n";

    // 1. Create input data
    std::cout << "Loading Mesh...\n";

    //geo::Mesh mesh = geo::Mesh::Load(RESOURCES_PATH"models/bunny/bunny.obj");
    //geo::Mesh mesh = geo::Mesh::Load(RESOURCES_PATH"models/fox_skull/fox_skull.obj");
    geo::Mesh mesh = geo::Mesh::Load(RESOURCES_PATH"models/DoraEmbrasure3_med_final/DoraEmbrasure3_med_final.obj");

    //mesh.Flatten();

    std::cout << "Done\n";

    // 2. Build DF

    geo::DistanceFieldParameters params;
    params.bounding_box = mesh.BoundingBox();
    params.resolution = 128;
    // set the maximum radius to be 1/4 the diagonal of the mesh
    params.max_distance = 0.25f * glm::length(params.bounding_box.Max() - params.bounding_box.Min());

    //geo::f32 cellSize = mesh.BoundingBox().MaxSize() / params.resolution;
    //params.max_distance = cellSize * 5.0f;

    geo::DistanceField df;

    std::cout << "Building Distance Field......\n";
    core::TimingStat buildTime;

    core::TimePoint startBuild = core::Clock::now();
    df.Build(params, mesh);
    core::TimePoint endBuild = core::Clock::now();
    std::cout << "Done\n\n";

    std::cout << "Mesh Name: " << mesh.FileName() << "\n";
    std::cout << "Number of Points: " << mesh.VertexCount() << "\n";
    std::cout << "Number of Triangles: " << mesh.TriangleCount() << "\n";
    std::cout << "DF Resolution: " << params.resolution << "\n";
    std::cout << "DF Max Distance: " << params.max_distance << "\n";
    std::cout << "DF Build Time: " << core::TimeDifferenceMs(endBuild, startBuild) << " ms\n\n";

    df.Save(RESOURCES_PATH"models/sdf_cache.gsdf");
    geo::DistanceField::Load(RESOURCES_PATH"models/sdf_cache.gsdf", df);

    // 3. Query test (1M samples)
    const u32 NUM_QUERIES = 1000000u;

    core::TimingStat queryStat;
    f32 sum = 0.0f;

    for (int i = 0; i < NUM_QUERIES; ++i)
    {
        // random point in bbox
        glm::vec3 q(rng.Float(params.bounding_box.Min().x, params.bounding_box.Max().x),
            rng.Float(params.bounding_box.Min().y, params.bounding_box.Max().y),
            rng.Float(params.bounding_box.Min().z, params.bounding_box.Max().z));

        core::ScopedTimer scope(&queryStat);
        sum += df(q);
    }

    std::cout << "Query Time (1M): " << queryStat.ToString() << "\n";

    // 4. Sanity check
    std::cout << "Zero-ish test: " << df(mesh.Vertex(0) + 0.1f * mesh.Normal(0)) << "\n";

    std::cout << "Far test: " << df(glm::vec3(100, 100, 100)) << "\n";
    std::cout << "Accumulated Value (ignore): " << sum << "\n";

    std::cout << "================================\n";
}

int main(int argc, char** argv)
{
    core::SetLogLevel(core::LogLevel::LOG_ERROR);
	int r = 0;

#ifdef RUN_PartialScans
	RunPartialScansAlignmentTests();
#endif // RUN_PartialScans

#ifdef RUN_DFTEST
    TestDF();
#endif // RUN_DFTEST

#ifdef RUN_UnitTests
	r = RunUnitTests(argc, argv);
#endif // RUN_UnitTests

	return r;
}
