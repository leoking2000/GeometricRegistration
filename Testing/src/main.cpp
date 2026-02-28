#include "test_pointcloud.h"

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int r = RUN_ALL_TESTS();

	return r;
}