#include "Graphics/RaylibRenderer.h"
#include "Core/ICPSystem.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include <geo/GeometricRegistration.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

static geo::Random rng{ 2026 }; 

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
            result = system.Solve(500);
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
            << geo::RMSE(system.GetSource().GetPoints(), ground_truth) << "\n";
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
        << geo::RMSE(system.GetSource().GetPoints(), ground_truth) << "\n";

    std::cout << ss.str();
}

int main()
{
    // get a mesh
    geo::Mesh mesh = geo::Mesh(RESOURCES_PATH"models/bunny/bunny.obj");
    std::vector<glm::vec3> vertices = mesh.Vertices();
    for (size_t i = 0; i < vertices.size(); i++)
    {
        glm::vec4 scaled =  glm::scale(glm::mat4(1.0f), glm::vec3(5.0f)) * glm::vec4(vertices[i], 1.0f);
        vertices[i] = glm::vec3(scaled);
    }

    //geo::PointCloud3D source = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 10000, rng, true);
    geo::PointCloud3D source = geo::PointCloud3D(vertices);

    // get some part of the bunny
    std::vector<glm::vec3> points = source.GetPoints();
    points.erase(
        std::remove_if(points.begin(), points.end(),
            [](const glm::vec3& p) {
                return p.x < -3;
            }),
        points.end()
    );
    
    // make the target be a part, this way the "outliers are in source cloud"
    geo::PointCloud3D target = geo::PointCloud3D(points);

    geo::SetLogLevel(geo::VERBOSE);

    GEOLOGDEBUG("Number of points: " << source.Size() + target.Size());
    GEOLOGDEBUG("Number of source points: " << source.Size());
    GEOLOGDEBUG("Number of target points: " << target.Size());
    
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

    std::cout << "<<Point to Point>>\n\n";
    ICPSystem system_ptp(ICPMethod::NAIVE, target, source);
    RunProjectInConsole(system_ptp, vertices);
    
    //std::cout << "\n<<Point to Plane>>\n\n";
    //ICPSystem system_ptpl(ICPMethod::NAIVE_PLANE, target, source);
    //RunProjectInConsole(system_ptpl);

    std::cout << "\n<<Sparse Point to Point>>\n\n";
    ICPSystem system_sparse(ICPMethod::SPARSE, target, source);
    RunProjectInConsole(system_sparse, vertices);


    //RunProjectWithWindow(system_ptp, vertices);

    return 0;
}
