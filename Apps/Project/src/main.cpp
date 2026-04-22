#include "Graphics/RaylibRenderer.h"
#include "Core/ICPSystem.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <functional>

#include <geo/GeometricRegistration.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

static geo::Random rng{ 2026 };

static geo::PointCloud3D CropPointCloud(
    const geo::PointCloud3D& cloud,
    const std::function<bool(const glm::vec3&)>& keep)
{
    std::vector<glm::vec3> pointsOut;
    std::vector<glm::vec3> normalsOut;

    pointsOut.reserve(cloud.Size());
    if (cloud.HasNormals())
    {
        normalsOut.reserve(cloud.Size());
    }

    for (geo::index_t i = 0; i < cloud.Size(); ++i)
    {
        const glm::vec3& p = cloud.Point(i);
        if (!keep(p))
        {
            continue;
        }

        pointsOut.push_back(p);

        if (cloud.HasNormals())
        {
            normalsOut.push_back(cloud.Normal(i));
        }
    }

    if (cloud.HasNormals())
    {
        return geo::PointCloud3D(std::move(pointsOut), std::move(normalsOut));
    }

    return geo::PointCloud3D(std::move(pointsOut));
}

static void RunProjectWithWindow(ICPSystem& system, const std::vector<glm::vec3>& ground_truth)
{
    geo::ICPResult result;

    // ---------------- Create the Renderer ----------------
    RaylibRenderer renderer;
    renderer.Init();

    // ---------------- Main Loop ----------------
    while (!renderer.ShouldClose())
    {
        //UpdateCamera(&camera, CAMERA_FREE);


        // Run one ICP iteration on SPACE
        //if (IsKeyDown(KEY_SPACE))
        if(IsKeyPressed(KEY_SPACE))
        {
            result = system.Solve(1);
        }

        // ---------------- Rendering ----------------
        renderer.BeginFrame();

        renderer.RenderGrid();

        renderer.RenderPointCloud(system.GetTarget(), RED);
        renderer.RenderPointCloud(system.GetSource(), BLUE);

        // UI
        std::stringstream ss;
        ss << "FPS:" << GetFPS() << "\n";
        ss << "Press SPACE to run 1 ICP iteration\n";
        ss << "Current Truth RMSE: " << std::fixed << std::setprecision(6) 
            << geo::PointToPointRMSE(system.GetSource().GetPoints(), ground_truth) << "\n";
        ss << "Total Time: "          << result.totalIterationTime.totalMs << "ms\n";
        ss << "Correspondence Time: " << result.correspondenceSearchTime.totalMs << "ms\n";
        ss << "Solver Time: "         << result.alignmentSolveTime.totalMs << "ms\n";

        renderer.RenderText(ss.str(), 20, 20, 25, DARKGRAY);

        renderer.EndFrame();
    }
}

static void RunProjectInConsole(ICPSystem& system, const std::vector<glm::vec3>& ground_truth)
{
    geo::ICPResult result = system.Solve(500);

    std::stringstream ss;
    ss << "Number of Target Points: " << system.GetTarget().Size() << "\n";
    ss << "Number of Source Points: " << system.GetSource().Size() << "\n";
    ss << "Iterations: " << result.iterations << "\n";
    ss << "Total Time: " << result.totalIterationTime.totalMs << "ms\n";
    ss << "Avg Correspondence Time: " << result.correspondenceSearchTime.AverageMs() << "ms\n";
    ss << "Avg Solver Time: " << result.alignmentSolveTime.AverageMs() << "ms\n";
    ss << "converged: " << result.converged << "\n";
    ss << "Current Truth RMSE: " << std::fixed << std::setprecision(6)
        << geo::PointToPointRMSE(system.GetSource().GetPoints(), ground_truth) << "\n";

    std::cout << ss.str();
}

int main()
{
    // get a mesh
    geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/bunny/bunny.obj");
    //geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/fox_skull/fox_skull.obj");
    //geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj");

    // the ground_truth
    std::vector<glm::vec3> sourceTruth = mesh.Vertices();

    //geo::PointCloud3D source = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 10000, rng, true);
    geo::PointCloud3D source = mesh.ToPointCloud();
 
    // make the target be a part, this way the "outliers are in source cloud"
    const float splitZ = source.Centroid().z;
    geo::PointCloud3D target = CropPointCloud(
        source,
        [splitZ](const glm::vec3& p)
        {
            return p.z > splitZ;
        });

    geo::SetLogLevel(geo::LogLevel::VERBOSE);

    GEOLOGDEBUG("Number of points: " << source.Size() + target.Size());
    GEOLOGDEBUG("Number of source points: " << source.Size());
    GEOLOGDEBUG("Number of target points: " << target.Size());
    GEOLOGDEBUG("Has Normals: " << target.HasNormals());
    
    // Apply known transform to source
    glm::vec3 eulerRot(10.0f, 5.0f, 2.5f);
    glm::vec3 translation(-1.0f, 0.0f, 1.0f);

    glm::mat4 Rx = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.x),
        glm::vec3(1, 0, 0));
    glm::mat4 Ry = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.y),
        glm::vec3(0, 1, 0));
    glm::mat4 Rz = glm::rotate(glm::mat4(1.0f),
        glm::radians(eulerRot.z),
        glm::vec3(0, 0, 1));

    glm::mat4 rotation = Ry * Rx * Rz;

    source.Transform({ rotation, translation });

    //std::cout << "<<Point to Point>>\n\n";
    //ICPSystem system_ptp(ICPMethod::NAIVE, target, source);
    //RunProjectInConsole(system_ptp, sourceTruth);
    
    std::cout << "\n<<Point to Plane>>\n\n";
    ICPSystem system_ptpl(ICPMethod::NAIVE_PLANE, target, source);
    RunProjectInConsole(system_ptpl, sourceTruth);

    //std::cout << "\n<<Sparse Point to Point>>\n\n";
    //ICPSystem system_sparse(ICPMethod::SPARSE, target, source);
    //RunProjectInConsole(system_sparse, sourceTruth);

    std::cout << "\n<<Sparse Point to Plane>>\n\n";
    ICPSystem system_sparse_p(ICPMethod::SPARSE_PLANE, target, source);
    RunProjectInConsole(system_sparse_p, sourceTruth);

    //RunProjectWithWindow(system_sparse_p, sourceTruth);

    return 0;
}
