#include "TestSuitePSA.h"

// Internal helpers //

// Gaussian noise via Box-Muller
static inline glm::vec3 GaussianNoise(const glm::vec3& p, f32 stdDev, core::Random& rng)
{
    auto sample = [&]() -> f32
        {
            f32 u1 = rng.Float(1e-6f, 1.0f);
            f32 u2 = rng.Float(0.0f, 1.0f);
            return std::sqrt(-2.0f * std::log(u1)) * std::cos(glm::two_pi<f32>() * u2);
        };
    return p + glm::vec3(sample(), sample(), sample()) * stdDev;
}

// Partial overlap — keep overlapRatio fraction of points from
// the "front" of the object (+X half-space in local frame).
// The rest are back-face points with no target correspondence.
static inline void ApplyPartialOverlap(
    std::vector<glm::vec3>& pts,
    std::vector<glm::vec3>& nrm,
    f32 overlapRatio)
{
    if (overlapRatio >= 1.0f || pts.empty()) return;

    const bool hasNormals = (nrm.size() == pts.size());

    std::vector<f32> proj(pts.size());
    for (size_t i = 0; i < pts.size(); i++)
        proj[i] = pts[i].x;

    std::vector<f32> sorted = proj;
    std::sort(sorted.begin(), sorted.end());

    const size_t keepCount =
        std::max(size_t(1), size_t(std::round(pts.size() * overlapRatio)));
    const f32 threshold = sorted[keepCount - 1];

    std::vector<glm::vec3> front, back, frontN, backN;
    for (size_t i = 0; i < pts.size(); i++)
    {
        if (proj[i] <= threshold) {
            front.push_back(pts[i]);
            if (hasNormals) frontN.push_back(nrm[i]);
        }
        else {
            back.push_back(pts[i]);
            if (hasNormals) backN.push_back(nrm[i]);
        }
    }

    pts.clear(); nrm.clear();
    pts.insert(pts.end(), front.begin(), front.end());
    pts.insert(pts.end(), back.begin(), back.end());
    if (hasNormals) {
        nrm.insert(nrm.end(), frontN.begin(), frontN.end());
        nrm.insert(nrm.end(), backN.begin(), backN.end());
    }
}

// Outliers — replace tail points with random noise in bbox
static inline void ApplyOutliers(
    std::vector<glm::vec3>& pts,
    std::vector<glm::vec3>& nrm,
    f32 outlierRatio,
    const core::BBox& bbox,
    core::Random& rng)
{
    if (outlierRatio <= 0.0f || pts.empty()) return;

    const bool hasNormals = (nrm.size() == pts.size());
    const glm::vec3 expand = bbox.Size() * 0.2f;
    const glm::vec3 bmin = bbox.Min() - expand;
    const glm::vec3 bmax = bbox.Max() + expand;

    const size_t count = size_t(std::round(pts.size() * outlierRatio));
    const size_t startIdx = pts.size() - count;

    for (size_t i = startIdx; i < pts.size(); i++) {
        pts[i] = rng.Float3(0.0f, 1.0f) * (bmax - bmin) + bmin;
        if (hasNormals) nrm[i] = rng.Dir3D(1.0f);
    }
}

static inline std::unique_ptr<tests::Model> BuildSourceModel(
    const geo::Mesh& targetMesh, core::RigidTransform groundTruth,
    // sampleRatio * vertex_coint = points to sample from the target mesh to build the the synthetic source scan.
    // sampleRatio = -1 -> use the target point cloud as is, it does not sample the mesh.
    f32 sampleRatio,
    f32 overlapRatio, f32 outlierRatio, f32 noiseStdDev,
    u32 seed)
{
    core::Random rng(seed);

    geo::PointCloud3D raw;
    if (sampleRatio <= 0.0f) 
    {
        raw = targetMesh.ToPointCloud();
    }
    else
    {
        raw = targetMesh.SamplePointsUniform((u32)glm::ceil(sampleRatio * targetMesh.VertexCount()), rng, true);
    }

    std::vector<glm::vec3> pts(raw.GetPoints());
    std::vector<glm::vec3> nrm(raw.GetNormals());

    // Degrade in LOCAL frame before applying ground truth transform.
    // This keeps conditions independent of where the object is placed.
    ApplyPartialOverlap(pts, nrm, overlapRatio);
    ApplyOutliers(pts, nrm, outlierRatio, targetMesh.BoundingBox(), rng);

    // add Gaussian noise
    if (noiseStdDev > 0.0f) {
        for (glm::vec3& p : pts) { 
            p = GaussianNoise(p, noiseStdDev, rng);
        }
    }

    // Apply ground truth, source is now in world frame
    geo::PointCloud3D cloud(std::move(pts), std::move(nrm));
    cloud.Transform(groundTruth.ComputeInverse()); // groundTruth is source -> target, so we apply the Inverse

    return std::make_unique<tests::Model>(targetMesh.FileName() + ".Test", cloud);
}

namespace tests
{
	void TestSuitePSA::Add(const std::string& name, Model* target, 
        core::RigidTransform groundTruth,
        f32 sampleRatio, f32 overlapRatio, f32 outlierRatio, f32 noiseStdDev, u32 seed)
	{
        // Build source model
        std::unique_ptr<Model> sourceModel = BuildSourceModel(target->mesh, groundTruth, 
            sampleRatio, overlapRatio, outlierRatio, noiseStdDev, seed);

        Model* sourcePtr = sourceModel.get();
        m_sourceModels.emplace_back(std::move(sourceModel));

        // build the test case
        TestCase tc;
        tc.name = name;

        tc.target = target;
        tc.source = sourcePtr;
        tc.groundTruth  = groundTruth;

        tc.sampleRatio  = sampleRatio;
        tc.overlapRatio = overlapRatio;
        tc.outlierRatio = outlierRatio;
        tc.noiseStdDev  = noiseStdDev;

        m_cases.emplace_back(std::move(tc));
	}

	void TestSuitePSA::AddFullOverlap(Model * target, const core::RigidTransform & gt, u32 seed)
	{
        Add("FullOverlap", target, gt, -1.0f, 1.0f, 0.0f, 0.0f, seed);
	}

	void TestSuitePSA::AddHalfOverlap(Model * target, const core::RigidTransform & gt, u32 seed)
	{
        Add("HalfOverlap", target, gt, -1.0f, 0.5f, 0.0f, 0.0f, seed);
	}

	void TestSuitePSA::AddQuarterOverlap(Model* target, const core::RigidTransform& gt, u32 seed)
	{
        Add("QuarterOverlap", target, gt, -1.0f, 0.25f, 0.0f, 0.0f, seed);
	}

	void TestSuitePSA::AddWithOutliers(Model * target, 
        f32 outlierRatio, const core::RigidTransform & gt, u32 seed)
	{
        Add("Outliers_" + std::to_string(int(outlierRatio * 100)) + "pct", 
            target, gt, -1.0f, 1.0f, outlierRatio, 0.0f, seed);
	}

	void TestSuitePSA::AddWithNoise(Model * target, f32 noiseStdDev, const core::RigidTransform & gt, u32 seed)
	{
        Add("Noisy_std" + std::to_string(noiseStdDev),
            target, gt, -1.0f, 1.0f, 0.0f, noiseStdDev, seed);
	}

	void TestSuitePSA::AddHard(Model * target, const core::RigidTransform & gt, u32 seed)
	{
        Add("Hard_sampled_ov25_out1", target, gt, 1.0f, 0.25f, 0.01f, 0.005f, seed);
	}
}
