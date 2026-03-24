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

static void RunProjectWithWindow(ICPSystem& system)
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
        if (IsKeyPressed(KEY_SPACE))
        {
            result = system.Step();
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
        ss << "Current RMS: " << std::fixed << std::setprecision(6) << result.rms << "\n";
        ss << "Total Time: " << std::fixed << std::setprecision(2) << result.totalElapsed_ms << "ms\n";
        ss << "Correspondence Time: " << std::fixed << std::setprecision(2) << result.avgCorrespondenceTime_ms << "ms\n";
        ss << "Solver Time: " << std::fixed << std::setprecision(2) << result.avgSolverTime_ms << "ms\n";

        renderer.RenderText(ss.str(), 20, 20, 25, DARKGRAY);

        renderer.EndFrame();
    }
}

static void RunProjectInConsole(ICPSystem& system)
{
    geo::ICPResult result = system.Solve();

    std::stringstream ss;
    ss << "Number of Points: " << system.GetTarget().Size() << "\n";
    ss << "Iterations: " << result.iterations << "\n";
    ss << "converged: " << result.converged << "\n";
    ss << "RMS: " << std::fixed << std::setprecision(6) << result.rms << "\n";
    ss << "Total Time: " << std::fixed << std::setprecision(2) << result.totalElapsed_ms << "ms\n";
    ss << "Avg Correspondence Time: " << std::fixed << std::setprecision(2) << result.avgCorrespondenceTime_ms << "ms\n";
    ss << "Avg Solver Time: " << std::fixed << std::setprecision(2) << result.avgSolverTime_ms << "ms\n";

    std::cout << ss.str();
}

int main()
{
    geo::PointCloud3D target = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 1000, rng);
    geo::PointCloud3D source = geo::GenerateRandomPointCloudRect(glm::vec3(0.0f), 10.0f, 10.0f, 10.0f, 1000, rng); //target;
    LOGDEBUG("RandomPointCloud genearated!!!");


    // Apply known transform to source
    glm::vec3 eulerRot(20.0f, 40.0f, 10.0f);
    glm::vec3 translation(5.0f, 3.0f, -4.0f);

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

    source.Transform(rotation, translation);

    std::cout << "<<Point to Point>>\n\n";
    ICPSystem system_ptp(ICPMethod::NAIVE, target, source);
    RunProjectInConsole(system_ptp);
    

    std::cout << "\n<<Point to Plane>>\n\n";
    ICPSystem system_ptpl(ICPMethod::NAIVE_PLANE, target, source);
    RunProjectInConsole(system_ptpl);


    //RunProjectWithWindow(system_ptpl);

    return 0;
}
