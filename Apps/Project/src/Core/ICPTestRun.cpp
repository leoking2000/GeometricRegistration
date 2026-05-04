#include "ICPTestRun.h"

namespace test
{
    static inline geo::f32 RotationError(const glm::mat3& R1, const glm::mat3& R2)
    {
        glm::mat3 R = R1 * glm::transpose(R2);
        return geo::RotationAngle(R);
    }

    static inline geo::f32 TranslationError(const glm::vec3& t1, const glm::vec3& t2)
    {
        return glm::length(t1 - t2);
    }

    void PrintResult(const ICPTestResult& r)
    {
        std::cout << "==== " << r.methodName << " | " << r.testName << " ====\n";

        std::cout << "Number of Source Points: " << r.sourceCount << '\n';
        std::cout << "Number of Target Points: " << r.targetCount << '\n';

        std::cout << "--------------------------------\n";

        std::cout << "RMSE: " << r.result.rmse << '\n';
        std::cout << "Iterations: " << r.result.iterations << '\n';
        std::cout << "Converged: " << r.result.converged << '\n';

        std::cout << "--------------------------------\n";

        std::cout << "Rotation Error (rad): " << r.rotationError << '\n';
        std::cout << "Translation Error: " << r.translationError << '\n';

        std::cout << "--------------------------------\n";

        std::cout << "Total Time [" << r.result.totalIterationTime.ToString() << "]\n";
        std::cout << "Correspondence Search Time [" << r.result.correspondenceSearchTime.ToString() << "]\n";
        std::cout << "Alignment Solve Time [" << r.result.alignmentSolveTime.ToString() << "]\n\n\n";
    }

	ICPTestResult RunLeastSquaresICP(const ICPTestCase& test, const geo::LeastSquaresICPParameters& params)
	{
        geo::PointCloud3D sourceCopy = test.source;

        geo::KDTree nn(test.target.GetPoints());

        auto result = geo::LeastSquaresICP(test.target, sourceCopy, nn, params);

        geo::RigidTransform gtInv = test.groundTruth.ComputeInverse();

        ICPTestResult out;
        out.methodName = "LeastSquaresICP " + ((params.useNormals) ? std::string("PointToPlane") : std::string("PointToPoint"));
        out.testName = test.name;

        out.result = result;

        out.rotationError = RotationError(result.transform.rotation, gtInv.rotation);
        out.translationError = TranslationError(result.transform.translation, gtInv.translation);

        out.sourceCount = test.source.Size();
        out.targetCount = test.target.Size();

        return out;
	}


    ICPTestResult RunSparseICPPointToPoint(const ICPTestCase& test, const geo::SparseICPParameters& params)
    {
        geo::PointCloud3D sourceCopy = test.source;

        geo::KDTree nn(test.target.GetPoints());

        auto result = geo::SparseICPPointToPoint(test.target, sourceCopy, nn, params);

        geo::RigidTransform gtInv = test.groundTruth.ComputeInverse();

        ICPTestResult out;
        out.methodName = "SparseICP PointToPoint ";
        out.testName = test.name;

        out.result = result;

        out.rotationError = RotationError(result.transform.rotation, gtInv.rotation);
        out.translationError = TranslationError(result.transform.translation, gtInv.translation);

        out.sourceCount = test.source.Size();
        out.targetCount = test.target.Size();

        return out;
    }

    ICPTestResult RunSparseICPPointToPlane(const ICPTestCase& test, const geo::SparseICPParameters& params)
    {
        geo::PointCloud3D sourceCopy = test.source;

        geo::KDTree nn(test.target.GetPoints());

        auto result = geo::SparseICPPointToPlane(test.target, sourceCopy, nn, params);

        geo::RigidTransform gtInv = test.groundTruth.ComputeInverse();

        ICPTestResult out;
        out.methodName = "SparseICP PointToPlane";
        out.testName = test.name;

        out.result = result;

        out.rotationError = RotationError(result.transform.rotation, gtInv.rotation);
        out.translationError = TranslationError(result.transform.translation, gtInv.translation);

        out.sourceCount = test.source.Size();
        out.targetCount = test.target.Size();

        return out;
    }

    ICPTestResult RunEfficientICPPointToPlane(const ICPTestCase& test, const geo::EfficientICPParams& params)
    {
        geo::PointCloud3D sourceCopy = test.source;

        geo::KDTree nn(test.target.GetPoints());

        auto result = geo::EfficientICP(test.target, sourceCopy, nn, test.df, params);

        geo::RigidTransform gtInv = test.groundTruth.ComputeInverse();

        ICPTestResult out;
        out.methodName = "EfficientICP PointToPlane";
        out.testName = test.name;

        out.result = result;

        out.rotationError = RotationError(result.transform.rotation, gtInv.rotation);
        out.translationError = TranslationError(result.transform.translation, gtInv.translation);

        out.sourceCount = test.source.Size();
        out.targetCount = test.target.Size();

        return out;
    }

}