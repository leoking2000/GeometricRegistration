#include "TestSuitePSA.h"

namespace tests
{
	void TestSuitePSA::Add(const std::string& name, Model* target, geo::RigidTransform groundTruth, geo::f32 sampleRatio, geo::f32 overlapRatio, geo::f32 outlierRatio, geo::f32 noiseStdDev, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddFullOverlap(Model * target, const geo::RigidTransform & gt, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddHalfOverlap(Model * target, const geo::RigidTransform & gt, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddQuarterOverlap(Model* target, const geo::RigidTransform& gt, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddWithOutliers(Model * target, geo::f32 outlierRatio, const geo::RigidTransform & gt, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddWithNoise(Model * target, geo::f32 noiseStdDev, const geo::RigidTransform & gt, geo::u32 seed)
	{

	}

	void TestSuitePSA::AddHard(Model * target, const geo::RigidTransform & gt, geo::u32 seed)
	{

	}
}