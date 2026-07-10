#pragma once
#include <vector>
#include <memory>
#include "TestCase.h"

namespace tests
{
	// Test Suite for Partial Scans Alignment
	class TestSuitePSA
	{
	public:
		TestSuitePSA() = default;
	public:
		// Access
		std::vector<TestCase>& Cases() { return m_cases; }
		std::size_t Size() const { return m_cases.size(); }
	public:
		void Add(const std::string& name, Model* target, core::RigidTransform groundTruth, 
			// sampleRatio * vertex_coint = points to sample from the target mesh to build the the synthetic source scan.
			// sampleRatio = -1 -> use the target point cloud as is, it does not sample the mesh.
			f32 sampleRatio = -1.0f, 
			f32 overlapRatio = 1.0f, f32 outlierRatio = 0.0f, f32 noiseStdDev = 0.0f,
			u32 seed = 2026);
	public:
		// Named scenario shortcuts — express intent at the call site
		void AddFullOverlap(Model* target, const core::RigidTransform& gt, u32 seed = 2026);
		void AddHalfOverlap(Model* target, const core::RigidTransform& gt, u32 seed = 2026);
		void AddQuarterOverlap(Model* target, const core::RigidTransform& gt, u32 seed = 2026);
		void AddWithOutliers(Model* target, f32 outlierRatio, const core::RigidTransform& gt, u32 seed = 2026);
		void AddWithNoise(Model* target, f32 noiseStdDev, const core::RigidTransform& gt, u32 seed = 2026);
		void AddHard(Model* target, const core::RigidTransform& gt, u32 seed = 2026);
	public:
		// Iterator support for range-based for loops (read-only)
		auto begin() const { return m_cases.cbegin(); }
		auto end() const { return m_cases.cend(); }
	private:
		// unique_ptr gives stable addresses, TestCase pointers remain valid even as m_sourceModels grows.
		std::vector<std::unique_ptr<Model>> m_sourceModels; // we own the source models
		std::vector<TestCase>               m_cases;
	};
}
