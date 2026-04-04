#include "test_pointcloud.h"
#include "test_mesh.h"
#include "test_kdtree.h"
#include "test_naiveicp.h"
#include "test_transform.h"

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}