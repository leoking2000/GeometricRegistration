#include "UnitTests/UnitTests.h"
#include "TestSuite/TestSuitePSA.h"

using namespace tests;

#define RUN_PartialScans
#define RUN_UnitTests

static void RunPartialScansAlignmentTests()
{
	std::cout << "====== Partial Scans Alignment Tests ======\n";

	Model bunny = CreateModelFromOBJ(RESOURCES_PATH"models/bunny/bunny.obj", 64);

	GEOLOGINFO(bunny.name << " " <<
		bunny.mesh.VertexCount() << " " << 
		bunny.cloud.Size() << " " << 
		bunny.kdTree.Size() << " " << bunny.sdf( bunny.cloud.Point(0) ) );

	std::cout << "\n";
}

int main(int argc, char** argv)
{
	int r = 0;

#ifdef RUN_PartialScans
	RunPartialScansAlignmentTests();
#endif // RUN_PartialScans

#ifdef RUN_UnitTests
	r = RunUnitTests(argc, argv);
#endif // RUN_UnitTests

	return r;
}
