#include <omp.h>
#include "UnitTests/UnitTests.h"
#include "TestSuite/TestSuitePSA.h"

using namespace tests;

static constexpr u32 SEED = 2026;

#define RUN_PartialScans

//#define RUN_LeastSquaresICP
#define RUN_SparseICP
#define RUN_EfficientICP

//#define PartialOverlap_Tests
//#define Outliers_Tests

//#define RUN_UnitTests

static void RunPartialScansAlignmentTests()
{
	geo::SetLogLevel(geo::LogLevel::LOG_ERROR);
	std::cout << "====== Partial Scans Alignment Tests ======\n";

	std::cout << "Loading 3D Models...\n";
	Model bunny = CreateModelFromOBJ(RESOURCES_PATH"models/bunny/bunny.obj", 128);
	Model dora_1 = CreateModelFromOBJ(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj", 128);
	std::cout << "Models Loaded\n";

	std::cout << "Creating Test Suite...";

	//glm::vec3 eulerRot(10.0f, 5.0f, 2.5f);
	//glm::vec3 translation(-1.0f, 0.0f, 1.0f);

	glm::vec3 eulerRot(60.0f, -50.0f, 20.5f);
	glm::vec3 translation(0.0f, 0.0f, 0.0f);

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
	geo::RigidTransform gt = { glm::mat3(rotation), translation };

	TestSuitePSA suite;

	suite.AddHalfOverlap(&bunny, gt, SEED);

	std::cout << "Done\n";

#ifdef RUN_SparseICP
	geo::SparseICPParameters p_spa;
	p_spa.maxIterations = 100;
	p_spa.p = 0.4f;
	TestResult result_spa = RunSparseICPPointToPlane(suite.Cases()[0], p_spa);

	LogTestResult(result_spa);
#endif // RUN_SparseICP

	std::cout << "=======================================================\n";

#ifdef RUN_EfficientICP
	geo::EfficientICPParams p_eff;
	p_eff.esaIterations = 1000;
	p_eff.icpParams.maxIterations = 100;
	p_eff.icpParams.p = 0.4f;
	TestResult result_eff = RunEfficientICPPointToPlane(suite.Cases()[0], p_eff);

	LogTestResult(result_eff);
#endif // RUN_EfficientICP

	std::cout << "\n";
}

int main(int argc, char** argv)
{
//#pragma omp parallel
//	{
//		int id = omp_get_thread_num();
//#pragma omp critical
//		{
//			std::cout << "Thread " << id << std::endl;
//		}
//	}

	int r = 0;

#ifdef RUN_PartialScans
	RunPartialScansAlignmentTests();
#endif // RUN_PartialScans

#ifdef RUN_UnitTests
	r = RunUnitTests(argc, argv);
#endif // RUN_UnitTests

	return r;
}
