#pragma once
#include <array>
#include <functional>
#include <geo/utils/GeoRand.h>
#include <geo/utils/GeoTime.h>
#include <geo/spatial/DistanceField.h>
#include <geo/geometry/PointCloud3D.h>
#include "RigidTransform.h"
#include "BBox.h"

// The Enhanced Simulated Annealing method as presented by P. Siarry, G. Berthiau,
// F. Durbin and J. Haussy in "Enhanced Simulated Annealing for Globally Minimizing
// Functions of Many-Continuous Variables", ACM Trans on Mathematical Software, 23(2),
// June 1997, pp.209-228.
// ARGUMENTS: 
//
// dim		         : the dimensionality of the search space. 
// subdim		     : the dimensionality of the partitioning subspace (subdim <= dim).
// x_init            : an optional initial starting point. x_init is randomly selected if
//				       this argument is NULL. 
// x_best            : the returned variable vector.
// x_min             : the minimum x values vector.
// x_max             : the maximum x values vector. 
// step_fraction     : a vector defining the maximum ( normalized ) jump for each variable.
// wraparound        : a flag vector (of size dim) specifying if the i-th variable values 
//				       must wrap around the limits ( 1=wrap, 0=truncate )  
// e(x)              : pointer the cost function to be minimized.
// monitor(x,e)      : pointer a monitoring function.
// iterations_max    : maximum itaerations.
float EnhancedSimulatedAnnealingPlus
(long dim, long subdim, float* x_init, float* x_best,
    float* x_min, float* x_max, float* step_fraction,
    long* wraparound, std::function<float(float*)> e,
    void (*monitor)(float*, float), long iterations_max,
    geo::Random& rng);

namespace geo
{
    // Layout:
    // x[0..2] = translation
    // x[3..5] = Euler angles (radians)
    using ESARigidTransform = std::array<f32, 6>;

    // Converts 6D ESA parameter vector into a rigid transform
    RigidTransform ConvertToRigidTransform(const ESARigidTransform& x);

    // Search space definition for ESA optimization
    struct ESASearchSpace
    {
        glm::vec3 translationMin{};         // the minimum of the translation vector.
        glm::vec3 translationMax{};         // the maximum of the translation vector.

        // Euler angle limits (radians) in ZYX convention.
        // Z is half-range [-pi/2, pi/2] to avoid gimbal singularity at poles.
        glm::vec3 rotationMin = { -glm::pi<f32>(), -glm::pi<f32>(), -glm::half_pi<f32>() };
        glm::vec3 rotationMax = { glm::pi<f32>(),  glm::pi<f32>(),  glm::half_pi<f32>() };

        // Normalized step size: fraction of the search range perturbed per move.
        // Translation and rotation are tuned separately.
        f32 translationStep = 0.1f;
        f32 rotationStep = 0.05f;

        // Full rotation search — use when you have no prior on orientation
        static ESASearchSpace FullRotation(const BBox& box)
        {
            ESASearchSpace s;
            glm::vec3 size = box.Size();
            s.translationMin = -size;
            s.translationMax = size;
            // rotationMin/Max already default to full range
            return s;
        }
    };

    // ESA configuration parameters
    struct ESAParameters
    {        
        u32 seed          = 2026u;                  // RNG seed — change for multi-run experiments
        u8  subDim        = 1u;                     // the dimensionality of the partitioning subspace (subDim <= 6).
        u32 maxIterations = 2000;                   // maximum itaerations.

        ESARigidTransform initialTransform{0.0f};   // initial starting point/transform — identity by default.
        ESASearchSpace searchSpace;                 // the search space.
    };

    // ESA result container
    struct ESAResult
    {
        RigidTransform transform = {};
        f32 cost = 0.0;

        f64 totalTime = 0.0;
    };

    // Runs ESA optimization for rigid transform estimation
    ESAResult RunESA(const ESAParameters& params, std::function<float(float*)> costfn);

    // Runs multiple ESA configurations in parallel and selects best result
    ESAResult MultiConfigESA(const std::vector<ESAParameters>& configs, std::function<float(float*)> costdf);

    // Multi-start ESA with randomized seeds
    ESAResult MultiStartESA(const ESAParameters& params, std::function<float(float*)> costfn, u32 numRuns);
}
