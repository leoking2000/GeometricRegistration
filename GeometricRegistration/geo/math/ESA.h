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
    // the RigidTransform define as [0..2]:translation | [3..5]:euler angles radians
    using ESARigidTransform = std::array<f32, 6>;

    RigidTransform ConvertToRigidTransform(const ESARigidTransform& x);

    struct ESAResult
    {
        RigidTransform transform = {};
        f32  rmse = 0.0;

        f64 totalTime = 0.0f;
    };

    struct ESASearchSpace
    {
        glm::vec3 translationMin;         // the minimum of the translation vector.
        glm::vec3 translationMax;         // the maximum of the translation vector.

        // the minimum of the rotation vector.
        glm::vec3 rotationMin = { -glm::pi<f32>(), -glm::pi<f32>(), -glm::half_pi<f32>() };

        // the maximum of the rotation vector.
        glm::vec3 rotationMax = { glm::pi<f32>(),  glm::pi<f32>(),  glm::half_pi<f32>() };

        f32 translationStep = 0.1f;       // defining the maximum ( normalized ) jump for translation
        f32 rotationStep = 0.05f;         // defining the maximum ( normalized ) jump for rotation
    };

    struct ESAParameters
    {        
        u32 seed          = 0u;           // the Random seed used
        u8  subDim        = 1u;           // the dimensionality of the partitioning subspace (subDim <= 6).
        u32 maxIterations = 2000;         // maximum itaerations.

        ESARigidTransform init{0.0f};     // initial starting point.
        ESASearchSpace searchSpace;       // the search space.
    };

    ESAResult RunESA(ESAParameters& params, const PointCloud3D& src, const geo::DistanceField& df);
}
