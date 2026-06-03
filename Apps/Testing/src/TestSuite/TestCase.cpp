#include <iostream>
#include <iomanip>
#include <string>
#include "TestCase.h"

namespace tests
{
	TestResult RunLeastSquaresICP(const TestCase& test, const geo::LeastSquaresICPParameters& params)
	{
		std::string methodName = 
			"LeastSquaresICP " + ((params.useNormals) ? std::string("PointToPlane") : std::string("PointToPoint"));
		geo::ICPResult result = geo::LeastSquaresICP(test.target->cloud, test.source->cloud, test.target->kdTree, params);

		return { test, result.transform, methodName, result, false, {} };
	}

	TestResult RunSparseICPPointToPoint(const TestCase& test, const geo::SparseICPParameters& params)
	{
		std::string methodName = "SparseICP PointToPoint (p=" + std::to_string(params.p) + ")";
		geo::ICPResult result = 
			geo::SparseICPPointToPoint(test.target->cloud, test.source->cloud, test.target->kdTree, params);

		return { test, result.transform, methodName, result, false, {} };
	}

	TestResult RunSparseICPPointToPlane(const TestCase& test, const geo::SparseICPParameters& params)
	{
		std::string methodName = "SparseICP PointToPlane (p=" + std::to_string(params.p) + ")";
		geo::ICPResult result =
			geo::SparseICPPointToPlane(test.target->cloud, test.source->cloud, test.target->kdTree, params);

		return { test, result.transform, methodName, result, false, {} };
	}

	TestResult RunEfficientICPPointToPlane(const TestCase& test, const geo::EfficientICPParams& params)
	{
		std::string methodName = "ESA+SparseICP PointToPlane (p=" + std::to_string(params.icpParams.p) + ")";

		geo::u32 targetCount = (geo::u32)glm::ceil(glm::min(2000.0f, 0.05f * test.source->cloud.Size()));

		geo::EfficientICPResult result = 
			geo::EfficientICP(test.target->cloud, test.source->cloud, 
				test.source->cloud.UniformSubsample(targetCount, params.seed),
				test.target->kdTree, test.target->sdf, params);

		return { test, result.transform, methodName, result.icp_result, true, result.esa_result };
	}

	// ====================================================================

	// ============================================================
	// Qualitative thresholds
	//
	// Rotation error thresholds are absolute (degrees) — universal.
	// Translation error thresholds are relative (% of mesh diagonal)
	// so they scale with object size automatically.
	// ============================================================

	static inline const char* RotationLabel(geo::f32 degrees)
	{
		if (degrees < 1.0f)  return "EXCELLENT";
		if (degrees < 5.0f)  return "GOOD";
		if (degrees < 15.0f) return "ACCEPTABLE";
		return "POOR";
	}

	static inline const char* TranslationLabel(geo::f32 percentOfDiagonal)
	{
		if (percentOfDiagonal < 0.1f) return "EXCELLENT";
		if (percentOfDiagonal < 0.5f) return "GOOD";
		if (percentOfDiagonal < 2.0f) return "ACCEPTABLE";
		return "POOR";
	}

	void LogTestResult(const TestResult& result)
	{
		constexpr int W = 56;
		const std::string divider(W, '-');
		const std::string header(W, '=');

		// --------------------------------------------------------
		// Header
		// --------------------------------------------------------
		std::cout << "\n" << header << "\n";
		std::cout << "  Method:  " << result.methodName << "\n";
		std::cout << "  Case:    " << result.testCase.name << "\n";
		std::cout << header << "\n";

		std::cout << std::fixed;

		// --------------------------------------------------------
		// Test Conditions
		// What degradation was applied to the source scan
		// --------------------------------------------------------
		std::cout << "\n  [Conditions]\n";
		std::cout << "    Overlap:   " << std::setprecision(0)
			<< (result.testCase.overlapRatio * 100.0f) << "%\n";
		std::cout << "    Outliers:  " << std::setprecision(0)
			<< (result.testCase.outlierRatio * 100.0f) << "%\n";
		std::cout << "    Noise std: " << std::setprecision(5)
			<< result.testCase.noiseStdDev << "\n";

		// --------------------------------------------------------
		// Models
		// Names and point counts for both target and source
		// --------------------------------------------------------
		std::cout << "\n" << divider << "\n";
		std::cout << "  [Models]\n";
		if (result.testCase.target)
			std::cout << "    Target: " << result.testCase.target->name
			<< "  (" << result.testCase.target->cloud.Size() << " points)\n";
		if (result.testCase.source)
			std::cout << "    Source: " << result.testCase.source->name
			<< "  (" << result.testCase.source->cloud.Size() << " points)\n";

		// --------------------------------------------------------
		// ESA Initialization (only when ESA was used)
		// Shows how well the global search initialized ICP.
		// A high ESA RMSE that ICP brings down to near zero means
		// ICP did most of the work — ESA just got it in the basin.
		// --------------------------------------------------------
		if (result.usedESA)
		{
			std::cout << "\n" << divider << "\n";
			std::cout << "  [ESA Initialization]\n";
			std::cout << "    Cost:  " << std::setprecision(10) << result.esa_result.cost << "\n";
			std::cout << "    Time:  " << std::setprecision(1) << result.esa_result.totalTime << " ms\n";
		}

		// --------------------------------------------------------
		// Convergence
		// Did the algorithm reach a stable solution?
		// --------------------------------------------------------
		std::cout << "\n" << divider << "\n";
		std::cout << "  [Convergence]\n";
		std::cout << "    Converged:  " << (result.icp_result.converged ? "YES" : "NO") << "\n";
		std::cout << "    Iterations: " << result.icp_result.iterations << "\n";
		std::cout << "    RMSE:       " << std::setprecision(10) << result.icp_result.rmse << "\n";

		// --------------------------------------------------------
		// Transform error vs ground truth
		//
		// Rotation error: geodesic angle between estimated and true rotation.
		// Translation error: Euclidean distance, also expressed as % of mesh
		// diagonal so it's meaningful regardless of object scale.
		// --------------------------------------------------------
		geo::f32 rotErr, transErr;
		geo::Distance(result.transform, result.testCase.groundTruth, rotErr, transErr);
		const geo::f32 rotErrDeg = glm::degrees(rotErr);

		// Diagonal for scale-relative translation error
		geo::f32 diagonal = 0.0f;
		geo::f32 transErrPct = 0.0f;
		const bool hasMesh = result.testCase.target && result.testCase.target->mesh.TriangleCount() > 0;
		if (hasMesh)
		{
			const geo::BBox& bb = result.testCase.target->mesh.BoundingBox();
			diagonal = glm::length(bb.Max() - bb.Min());
			transErrPct = (diagonal > 0.0f) ? (transErr / diagonal) * 100.0f : 0.0f;
		}

		std::cout << "\n" << divider << "\n";
		std::cout << "  [Transform Error vs Ground Truth]\n";

		std::cout << "    Rotation:    " << std::setprecision(10) << rotErrDeg << " deg"
			<< "  [" << RotationLabel(rotErrDeg) << "]\n";

		if (hasMesh)
		{
			std::cout << "    Translation: " << std::setprecision(10) << transErr
				<< "  (" << std::setprecision(10) << transErrPct << "% of diagonal)"
				<< "  [" << TranslationLabel(transErrPct) << "]\n";
		}
		else
		{
			std::cout << "    Translation: " << std::setprecision(10) << transErr << "\n";
		}

		// Overall verdict — worst of rotation and translation
		const bool rotGood = rotErrDeg < 5.0f;
		const bool transGood = !hasMesh || transErrPct < 0.5f;
		const char* verdict = (rotGood && transGood) ? "PASS" : "FAIL";
		std::cout << "\n    Overall: " << verdict << "\n";

		// --------------------------------------------------------
		// Timing breakdown
		// --------------------------------------------------------
		std::cout << "\n" << divider << "\n";
		std::cout << "  [Timing]\n";
		std::cout << "    Total:          " << std::setprecision(1) 
			<< result.icp_result.totalTimeMs + result.esa_result.totalTime << " ms\n";

		if (result.usedESA)
		{
			std::cout << "    Total ESA Time:          " << std::setprecision(1)
				<< result.esa_result.totalTime << " ms\n";
		}

		std::cout << "    Total ICP Time:          " << std::setprecision(1)
			<< result.icp_result.totalTimeMs << " ms\n";

		if (!result.icp_result.totalIterationTime.IsEmpty())
			std::cout << "    Per iteration:  " << std::setprecision(2)
			<< result.icp_result.totalIterationTime.AverageMs() << " ms avg"
			<< "  (" << result.icp_result.totalIterationTime.Count() << " samples)\n";

		if (!result.icp_result.correspondenceSearchTime.IsEmpty())
			std::cout << "    Correspondence: " << std::setprecision(2)
			<< result.icp_result.correspondenceSearchTime.AverageMs() << " ms avg\n";

		if (!result.icp_result.alignmentSolveTime.IsEmpty())
			std::cout << "    Solve:          " << std::setprecision(2)
			<< result.icp_result.alignmentSolveTime.AverageMs() << " ms avg\n";

		std::cout << header << "\n\n";
	}

	void LogComparison(const TestResult& r1, const TestResult& r2)
	{
		constexpr int W = 56;
		const std::string header(W, '=');

		TestCase tc = r1.testCase; // both TestResult should have the same test case!!!

		geo::f32 rot1, trans1, rot2, trans2;
		geo::Distance(r1.icp_result.transform, tc.groundTruth, rot1, trans1);
		geo::Distance(r2.icp_result.transform, tc.groundTruth, rot2, trans2);

		const geo::f32 rot1Deg = glm::degrees(rot1);
		const geo::f32 rot2Deg = glm::degrees(rot2);

		const bool hasMesh = tc.target && tc.target->mesh.TriangleCount() > 0;
		geo::f32 diagonal = 0.0f;
		if (hasMesh)
		{
			const geo::BBox& bb = tc.target->mesh.BoundingBox();
			diagonal = glm::length(bb.Max() - bb.Min());
		}

		std::cout << "\n" << header << "\n";
		std::cout << "  Comparison: " << tc.name << "\n";
		std::cout << header << "\n";
		std::cout << std::fixed;

		// Column headers
		std::cout << "\n"
			<< std::setw(20) << std::left << ""
			<< std::setw(18) << std::left << r1.methodName
			<< std::setw(18) << std::left << r2.methodName
			<< "\n"
			<< std::string(W, '-') << "\n";

		// RMSE
		std::cout << std::setw(20) << std::left << "  RMSE"
			<< std::setw(18) << std::left << (std::to_string(r1.icp_result.rmse).substr(0, 8))
			<< std::setw(18) << std::left << (std::to_string(r2.icp_result.rmse).substr(0, 8))
			<< "\n";

		// Rotation error
		std::cout << std::setw(20) << std::left << "  Rotation (deg)"
			<< std::setprecision(3)
			<< std::setw(18) << std::left << rot1Deg
			<< std::setw(18) << std::left << rot2Deg
			<< "\n";

		// Translation error
		if (hasMesh && diagonal > 0.0f)
		{
			geo::f32 t1Pct = (trans1 / diagonal) * 100.0f;
			geo::f32 t2Pct = (trans2 / diagonal) * 100.0f;
			std::cout << std::setw(20) << std::left << "  Trans (% diag)"
				<< std::setprecision(3)
				<< std::setw(18) << std::left << t1Pct
				<< std::setw(18) << std::left << t2Pct
				<< "\n";
		}

		// Iterations
		std::cout << std::setw(20) << std::left << "  Iterations"
			<< std::setw(18) << std::left << r1.icp_result.iterations
			<< std::setw(18) << std::left << r2.icp_result.iterations
			<< "\n";

		// Time
		std::cout << std::setw(20) << std::left << "  Time (ms)"
			<< std::setprecision(1)
			<< std::setw(18) << std::left << r1.icp_result.totalTimeMs
			<< std::setw(18) << std::left << r2.icp_result.totalTimeMs
			<< "\n";

		// Converged
		std::cout << std::setw(20) << std::left << "  Converged"
			<< std::setw(18) << std::left << (r1.icp_result.converged ? "YES" : "NO")
			<< std::setw(18) << std::left << (r2.icp_result.converged ? "YES" : "NO")
			<< "\n";

		// ESA rows — only shown when at least one method used ESA.
		// "N/A" for the method that ran without ESA so the table stays aligned.
		if (r1.usedESA || r2.usedESA)
		{
			auto esaRmse = [](const TestResult& r) -> std::string
				{
					return r.usedESA
						? std::to_string(r.esa_result.cost).substr(0, 8)
						: "N/A";
				};
			auto esaTime = [](const TestResult& r) -> std::string
				{
					return r.usedESA
						? std::to_string(static_cast<int>(r.esa_result.totalTime)) + " ms"
						: "N/A";
				};

			std::cout << std::string(W, '-') << "\n";
			std::cout << std::setw(20) << std::left << "  ESA Cost"
				<< std::setw(18) << std::left << esaRmse(r1)
				<< std::setw(18) << std::left << esaRmse(r2) << "\n";
			std::cout << std::setw(20) << std::left << "  ESA Time (ms)"
				<< std::setw(18) << std::left << esaTime(r1)
				<< std::setw(18) << std::left << esaTime(r2) << "\n";
		}

		std::cout << header << "\n\n";
	}
}