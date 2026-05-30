#include <string>
#include <functional>
#include <geo/GeometricRegistration.h>

namespace test
{
    struct Model
    {
        std::string        name;
        geo::Mesh          mesh;
        geo::PointCloud3D  cloud;
        geo::KDTree        kdTree;
        geo::DistanceField sdf;

        Model(const Model&) = delete;
        Model& operator=(const Model&) = delete;

        Model(Model&&) = default;
        Model& operator=(Model&&) = default;
    };

    Model CreateModelFromOBJ(const std::filesystem::path& filePath, const geo::DistanceFieldParameters& df_params);

    struct TestCase
    {
        std::string name = "unnamed";

        test::Model* targte = nullptr;
        test::Model* source = nullptr;

        geo::RigidTransform groundTruth = {};

        // metadata about this test case.
        geo::f32 overlapRatio = 1.0f;
        geo::f32 outlierRatio = 0.0f;
        geo::f32 noiseStdDev  = 0.0f;
    };

    struct TestCaseParams
    {
        std::string name = "unnamed";

        geo::f32 sampleRatio  = 1.0f;  // [0,1]: fraction of the number of points in the source
        geo::f32 overlapRatio = 1.0f;  // [0,1]: fraction of source with target correspondence
        geo::f32 outlierRatio = 0.0f;  // [0,1]: fraction of source replaced with random noise
        geo::f32 noiseStdDev  = 0.0f;  // Gaussian noise on source points (world units)

        geo::RigidTransform groundTruth = {};

        geo::u32 seed = 42;
    };

    class TestSuite
    {
    public:
        // General factory — generates a synthetic source Model from
        // target->mesh and wires up a TestCase referencing both.
        void Add(Model* target, const TestCaseParams& params);
    public:
        // Named scenario shortcuts — express intent at the call site
        void AddFullOverlap(Model* target, const geo::RigidTransform& gt, geo::u32 seed = 42);
        void AddHalfOverlap(Model* target, const geo::RigidTransform& gt, geo::u32 seed = 42);
        void AddQuarterOverlap(Model* target, const geo::RigidTransform& gt, geo::u32 seed = 42);
        void AddWithOutliers(Model* target, geo::f32 outlierRatio, const geo::RigidTransform& gt, geo::u32 seed = 42);
        void AddWithNoise(Model* target, geo::f32 noiseStdDev, const geo::RigidTransform& gt, geo::u32 seed = 42);
        void AddHard(Model* target, const geo::RigidTransform& gt, geo::u32 seed = 42);
    public:
        // Access
        const std::vector<TestCase>& Cases() const { return m_cases; }
        std::size_t Size() const { return m_cases.size(); }
    private:
        std::vector<std::unique_ptr<Model>> m_sourceModels; // created by the class, target Models live outside
        std::vector<TestCase> m_cases;
    };


}
