#include <memory>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <filesystem>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>

#include <core/Rand.h>
#include <core/Time.h>
#include <core/math/RigidTransform.h>
#include <core/math/BBox.h>
#include <core/logging/Log.h>

#include "ExperimentRunner.h"
#include "TestSuitePSA.h"

namespace tests
{
    // ============================================================
    // Configuration
    // ============================================================

    // Placeholders — point these at wherever the downloaded meshes actually
    // land. sampleRatio is chosen so a full-overlap synthetic scan lands
    // roughly in the few-thousand-to-~20K point range regardless of the
    // source mesh's native density (SparseICP's per-iteration cost scales
    // with source point count, and we want that cost comparable across
    // models of very different native resolution). sampleRatio = -1 means
    // "use every vertex, no subsampling" (fine for Bunny's ~36K).
    struct ModelSpec
    {
        const char* label;
        const char* relPath;
        u32         dfResolution;
        f32         sampleRatio;
    };

    static const ModelSpec kModels[] =
    {
        { "Bunny",     "models/test/bunny/bunny.obj", 128, -1.0f  },
        { "Armadillo", "models/test/armadillo/armadillo.ply", 128, 0.03f },
        { "Buddha",    "models/test/buddha/buddha.obj", 128, 0.02f  },
    };

    struct MisalignLevel
    {
        const char* label;
        f32 rotMinDeg, rotMaxDeg;
        f32 transMinPct, transMaxPct; // % of target bbox diagonal
    };

    static const MisalignLevel kMisalignLevels[] =
    {
        { "Small",  10.0f,  20.0f,  3.0f,  8.0f },
        { "Medium", 45.0f,  90.0f, 15.0f, 30.0f },
        { "Large", 120.0f, 180.0f, 40.0f, 70.0f },
    };

    static const f32 kOverlapLevels[] = { 1.0f, 0.75f, 0.5f, 0.25f };

    static constexpr u32 kRepeatsPerCell = 20;
    static constexpr u32 kBaseSeed = 2026u;
    static constexpr u32 kEsaIterations = 5000;

    // Canonical bbox diagonal every target mesh is rescaled to on load.
    static constexpr f32 kTargetDiagonal = 2.0f;

    static geo::Mesh LoadAndNormalizeMesh(const std::filesystem::path& path, f32 targetDiagonal)
    {
        geo::Mesh raw = geo::Mesh::Load(path);

        const core::BBox& bb = raw.BoundingBox();
        const f32 currentDiagonal = glm::length(bb.Max() - bb.Min());
        const f32 scale = (currentDiagonal > 1e-8f) ? (targetDiagonal / currentDiagonal) : 1.0f;

        std::vector<glm::vec3> points = raw.GetVertices();
        for (glm::vec3& p : points) p *= scale;

        std::vector<glm::uvec3> triangles;
        triangles.reserve(raw.TriangleCount());
        for (index_t t = 0; t < raw.TriangleCount(); t++)
            triangles.push_back(raw.Triangle(t).vertexIndices);

        return geo::Mesh(raw.FileName(), std::move(points), std::move(triangles), raw.GetNormals());
    }

    // ============================================================
    // Ground truth generation
    // ============================================================

    // Builds a rigid ground-truth transform (source -> target): a rotation of
    // magnitude drawn uniformly from [rotMinDeg, rotMaxDeg] about a uniformly
    // random axis, and a translation of magnitude drawn uniformly from
    // [transMinPct, transMaxPct]% of targetDiagonal in a uniformly random
    // direction. Built directly via axis-angle (glm::rotate), not through the
    // Euler-array RigidTransform constructor, so this has no dependence on
    // ESA's ZYX parameterization — verified separately that ESA's own search
    // domain (x,y full range, z in [-90,90]) covers all of SO(3) regardless.
    static core::RigidTransform GenerateGroundTruth(
        core::Random& rng, const MisalignLevel& level, f32 targetDiagonal)
    {
        const glm::vec3 axis = rng.Dir3D(1.0f);
        const f32       angle = glm::radians(rng.Float(level.rotMinDeg, level.rotMaxDeg));

        const glm::mat4 R4 = glm::rotate(glm::mat4(1.0f), angle, axis);
        const glm::mat3 R = glm::mat3(R4);

        const glm::vec3 dir = rng.Dir3D(1.0f);
        const f32       transPct = rng.Float(level.transMinPct, level.transMaxPct);
        const glm::vec3 t = dir * (transPct * 0.01f) * targetDiagonal;

        return core::RigidTransform(R, t);
    }

    // ============================================================
    // Grid construction
    // ============================================================

    // Metadata not carried by TestCase itself (misalignment bucket label,
    // repeat index, seed) — built in lockstep with suite.Add() calls and
    // zipped back against suite.Cases() by index at output time. Deliberately
    // kept outside TestCase.h rather than adding fields there.
    struct CellMeta
    {
        std::string modelLabel;
        f32         overlapRatio = 1.0f;
        std::string misalignLabel;
        u32         repeat = 0;
        u32         seed = 0;
    };

    static void BuildGrid(
        Model& model, u32 modelIdx, const char* modelLabel, f32 sampleRatio,
        TestSuitePSA& suite, std::vector<CellMeta>& meta)
    {
        const core::BBox& bb = model.mesh.BoundingBox();
        const f32          diagonal = glm::length(bb.Max() - bb.Min());

        for (u32 overlapIdx = 0; overlapIdx < (u32)std::size(kOverlapLevels); overlapIdx++)
        {
            const f32 overlap = kOverlapLevels[overlapIdx];

            for (u32 misalignIdx = 0; misalignIdx < (u32)std::size(kMisalignLevels); misalignIdx++)
            {
                const MisalignLevel& level = kMisalignLevels[misalignIdx];

                for (u32 rep = 0; rep < kRepeatsPerCell; rep++)
                {
                    const u32 seed = kBaseSeed
                        + modelIdx * 100000u
                        + overlapIdx * 10000u
                        + misalignIdx * 1000u
                        + rep;

                    core::Random rng(seed);
                    const core::RigidTransform gt = GenerateGroundTruth(rng, level, diagonal);

                    const std::string name =
                        std::string(modelLabel) + "_ov" + std::to_string((int)(overlap * 100.0f))
                        + "_" + level.label + "_r" + std::to_string(rep);

                    suite.Add(name, &model, gt, sampleRatio, overlap, /*outlierRatio*/0.0f, /*noiseStdDev*/0.0f, seed);
                    meta.push_back({ modelLabel, overlap, level.label, rep, seed });
                }
            }
        }

        // Identity sanity case — deterministic floor-level check.
        {
            const u32 seed = kBaseSeed + modelIdx * 100000u + 999999u;
            const std::string name = std::string(modelLabel) + "_Identity";
            suite.Add(name, &model, core::RigidTransform::Identity(), sampleRatio, 1.0f, 0.0f, 0.0f, seed);
            meta.push_back({ modelLabel, 1.0f, "Identity", 0, seed });
        }
    }

    // ============================================================
    // CSV output
    // ============================================================

    static void WriteCsvHeader(std::ofstream& csv)
    {
        csv <<
            "model,overlap_ratio,misalign_level,repeat,seed,"
            "gt_rotation_deg,gt_translation_pct_diag,source_point_count,"
            "method,rotation_error_deg,translation_error_raw,translation_error_pct_diag,"
            "rmse,converged,iterations,"
            "esa_cost,esa_time_ms,icp_total_time_ms,total_time_ms,"
            "corr_avg_ms,corr_total_ms,solve_avg_ms,solve_total_ms,"
            "iter_avg_ms,iter_total_ms\n";
    }

    static void WriteCsvRow(std::ofstream& csv, const CellMeta& m, const TestResult& r)
    {
        const TestCase& tc = r.testCase;

        // Ground-truth magnitude recomputed from the transform itself (not
        // carried from GenerateGroundTruth), so this file has no hidden
        // dependency on how the pose was sampled.
        f32 gtRot, gtTrans;
        core::Distance(tc.groundTruth, core::RigidTransform::Identity(), gtRot, gtTrans);
        const f32 gtRotDeg = glm::degrees(gtRot);

        f32 diagonal = 0.0f, gtTransPct = 0.0f;
        const bool hasMesh = tc.target && tc.target->mesh.TriangleCount() > 0;
        if (hasMesh)
        {
            const core::BBox& bb = tc.target->mesh.BoundingBox();
            diagonal = glm::length(bb.Max() - bb.Min());
            gtTransPct = (diagonal > 0.0f) ? (gtTrans / diagonal) * 100.0f : 0.0f;
        }

        f32 rotErr, transErr;
        core::Distance(r.transform, tc.groundTruth, rotErr, transErr);
        const f32 rotErrDeg = glm::degrees(rotErr);
        const f32 transErrPct = (diagonal > 0.0f) ? (transErr / diagonal) * 100.0f : 0.0f;

        const auto& icp = r.icp_result;
        const auto& esa = r.esa_result;

        csv << '"' << m.modelLabel << "\"," << m.overlapRatio << ",\"" << m.misalignLabel << "\","
            << m.repeat << ',' << m.seed << ','
            << gtRotDeg << ',' << gtTransPct << ',' << (tc.source ? tc.source->cloud.Size() : 0) << ','
            << '"' << r.methodName << "\","
            << rotErrDeg << ',' << transErr << ',' << transErrPct << ','
            << icp.rmse << ',' << (icp.converged ? 1 : 0) << ',' << icp.iterations << ','
            << esa.cost << ',' << esa.totalTime << ','
            << icp.totalTimeMs << ',' << (icp.totalTimeMs + esa.totalTime) << ','
            << icp.correspondenceSearchTime.AverageMs() << ',' << icp.correspondenceSearchTime.TotalMs() << ','
            << icp.alignmentSolveTime.AverageMs() << ',' << icp.alignmentSolveTime.TotalMs() << ','
            << icp.totalIterationTime.AverageMs() << ',' << icp.totalIterationTime.TotalMs()
            << '\n';
    }

    // ============================================================
    // Driver
    // ============================================================

    void RunChapter5Experiments()
    {
        std::cout << "====== Chapter 5 Experiment Grid (Stanford Repo) ======\n";

        // 1. Load models. reserve() up front is load-bearing: TestCase/Model
        //    pointers into this vector must stay valid for the whole run, so
        //    the vector must never reallocate after grid-building begins.
        std::vector<std::unique_ptr<Model>> models;
        models.reserve(std::size(kModels));

        for (const ModelSpec& spec : kModels)
        {
            std::cout << "Loading " << spec.label << "...\n";
            models.push_back(std::make_unique<Model>(
                spec.label,
                LoadAndNormalizeMesh(RESOURCES_PATH + std::string(spec.relPath), kTargetDiagonal),
                spec.dfResolution));
        }

        // 2. Build the test grid.
        TestSuitePSA suite;
        std::vector<CellMeta> meta;

        for (u32 m = 0; m < (u32)models.size(); m++)
            BuildGrid(*models[m], m, kModels[m].label, kModels[m].sampleRatio, suite, meta);

        std::cout << "Built " << suite.Size() << " test cases ("
            << (suite.Size() * 2) << " registration runs across both methods)\n";

        // 3. Open output CSV.
        const std::filesystem::path outPath = RESOURCES_PATH "results/chapter5_results.csv";
        std::filesystem::create_directories(outPath.parent_path());

        std::ofstream csv(outPath);
        if (!csv.is_open())
        {
            LOGERROR("Could not open output CSV at " << outPath.string());
            return;
        }
        WriteCsvHeader(csv);

        // 4. Run every test case with both methods.
        geo::SparseICPParameters spa;
        spa.maxIterations = 100; // matches main.cpp's standalone-SparseICP precedent
        spa.p = 0.4f;

        geo::EfficientICPParams eff;
        eff.esaIterations = kEsaIterations;
        eff.icpParams.maxIterations = 25; // matches main.cpp's post-ESA refinement precedent
        eff.icpParams.p = 0.4f;

        for (std::size_t i = 0; i < suite.Size(); i++)
        {
            const TestCase& tc = suite.Cases()[i];
            eff.seed = meta[i].seed;

            std::cout << "[" << (i + 1) << "/" << suite.Size() << "] " << tc.name << "\n";

            const TestResult r_eff = RunEfficientICPPointToPlane(tc, eff);
            WriteCsvRow(csv, meta[i], r_eff);

            const TestResult r_spa = RunSparseICPPointToPlane(tc, spa);
            WriteCsvRow(csv, meta[i], r_spa);
        }

        csv.close();
        std::cout << "Done. Results written to " << outPath.string() << "\n";
    }
}
