#include "test_pointcloud.h"
#include "test_horn.h"

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}