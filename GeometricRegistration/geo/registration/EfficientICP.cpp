#include <array>
#include <glm/gtc/constants.hpp>
#include <geo/utils/logging/LogMacros.h>
#include <geo/math/ESA.h>
#include "EfficientICP.h"

namespace geo
{
    struct ESACostContext
    {
        const geo::DistanceField* df = nullptr;
        const std::vector<glm::vec3>* points = nullptr;
    };

    static ESACostContext g_ctx;

    static inline RigidTransform VectorToTransform(const float* x)
    {
        const glm::vec3 t(x[0], x[1], x[2]);
        const glm::vec3 r(x[3], x[4], x[5]); // radians

        glm::mat4 Rx = glm::rotate(glm::mat4(1.0f), r.x, glm::vec3(1, 0, 0));
        glm::mat4 Ry = glm::rotate(glm::mat4(1.0f), r.y, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1.0f), r.z, glm::vec3(0, 0, 1));

        glm::mat4 R = Ry * Rx * Rz;

        return {glm::mat3(R), t};
    }

    static float ESACostFunction(float* x)
    {
        const RigidTransform T = VectorToTransform(x);

        const auto& points = *g_ctx.points;
        const auto& df = *g_ctx.df;

        const u32 stride = 20;

        float cost = 0.0f;
        u32 count = 0;

        #pragma omp parallel for reduction(+:cost, count)
        for (int i = 0; i < (int)points.size(); i += stride)
        {
            const glm::vec3& p = points[i];

            glm::vec3 tp = T.TransformPoint(p);
            float d = df(tp);

            cost += d * d;
            count += 1;
        }

        return (count > 0) ? cost / (float)count : df.m_grid.GetDefaultValue();
    }
     
    ICPResult EfficientICP(const PointCloud3D& target, PointCloud3D& source, const INearestNeighbor& nn,
        const DistanceField& df, const EfficientICPParams& params)
    {
        assert(source.Size() >= 3);
        assert(target.Size() == nn.Size());
        assert(params.icpParams.maxIterations >= 1);
        assert(params.icpParams.tolerance > 0.0f);
        assert(params.icpParams.p > 0.0f && params.icpParams.p < 1.0f);
        assert(params.icpParams.mu > 0.0f);
        assert(params.icpParams.admmIterations >= 1);

        TimePoint startTotal = Clock::now();

        // 1. Build ESACostContext and ESA Variables
        g_ctx.df = &df;
        g_ctx.points = &source.GetPoints();

        float bestCost = FLT_MAX;
        float x_best[6] = {0.0f};

        BBox targetBBox = target.ComputeBoundingBox();
        BBox sourceBBox = source.ComputeBoundingBox();
        glm::vec3 tMin = targetBBox.Min() - sourceBBox.Max();
        glm::vec3 tMax = targetBBox.Max() - sourceBBox.Min();

        float margin = 0.1f * targetBBox.MaxSize();

        tMin -= glm::vec3(margin);
        tMax += glm::vec3(margin);

        float x_init[6] = { 0.0f };
        float x_min[6];
        float x_max[6];
        float step_fraction[6];
        long  wraparound[6];

        // --- translation ---
        x_min[0] = tMin.x; x_max[0] = tMax.x;
        x_min[1] = tMin.y; x_max[1] = tMax.y;
        x_min[2] = tMin.z; x_max[2] = tMax.z;

        // --- rotation ---
        const float PI = glm::pi<float>();
        x_min[3] = -PI * 0.5f; x_max[3] = PI * 0.5f;
        x_min[4] = -PI; x_max[4] = PI;
        x_min[5] = -PI; x_max[5] = PI;

        // --- step size ---
        step_fraction[0] = 0.1f; step_fraction[1] = 0.1f; step_fraction[2] = 0.1f;
        step_fraction[3] = 0.05f; step_fraction[4] = 0.05f; step_fraction[5] = 0.05f;

        // --- wrap ---
        wraparound[0] = 0; wraparound[1] = 0; wraparound[2] = 0;
        wraparound[3] = 1; wraparound[4] = 1; wraparound[5] = 1;


        // 2. Run ESA → get best x (6D vector)
        TimingStat totalESATime;
        for (u32 r = 0; r < params.esaRestarts; r++)
        {
            float x_candidate[6];

            TimePoint startESA = Clock::now();

            float cost = EnhancedSimulatedAnnealingPlus(
                6,                   // the dimensionality of the search space. 
                1,                   // the dimensionality of the partitioning subspace (subdim <= dim).
                NULL,                // x_init, random initial starting point.
                x_candidate,         // the returned variable vector.
                x_min,               // the minimum x values vector.
                x_max,               // the maximum x values vector. 
                step_fraction,       // a vector defining the maximum ( normalized ) jump for each variable.
                wraparound,          // a flag vector, ( 1=wrap, 0=truncate )
                ESACostFunction,     // pointer the cost function to be minimized.
                // pointer a monitoring function.
                [](float* x, float e) {  
                    GEOLOGVERBOSE("cost: " << e << " [" << x[0] << ", " << x[1] << ", " << x[2] << ", " << 
                                                           x[3] << ", " << x[4] << ", " << x[5] << ", ");
                },             
                params.esaIterations // maximum itaerations.
            );

            TimePoint endESA = Clock::now();

            totalESATime.AddSample(TimeDifferenceMs(endESA, startESA));

            if (cost < bestCost)
            {
                bestCost = cost;
                memcpy(x_best, x_candidate, sizeof(float) * 6);
            }
        }

        // 3. Convert x → transform T0
        RigidTransform T = VectorToTransform(x_best);

        // 4. Apply T0 to source
        source.Transform(T);

        // 5. Run Sparse ICP starting from T0
        ICPResult result = geo::SparseICPPointToPlane(target, source, nn, params.icpParams);

        // 6. Combine results
        result.transform = RigidTransform::Compose(result.transform, T);

        // 7. Return result
        TimePoint endTotal = Clock::now();
        result.totalTime = TimeDifferenceMs(endTotal, startTotal);
        result.totalESATime = totalESATime;

        return result;
    }

}