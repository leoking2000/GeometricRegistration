#include <raylib.h>
#include <raymath.h>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

#include <geo/GeometricRegistration.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Core/LeoRand.h"

#include "rlights.h"


static leo::Random rng{2026};

static geo::PointCloud3D GenerateRandomCloud(leo::u32 count, leo::f32 min, leo::f32 max)
{
	std::vector<glm::vec3> points;
	points.reserve(count);

	for (unsigned int i = 0; i < count; i++)
	{
		points.emplace_back(rng.Float3(min, max));
	}

	return geo::PointCloud3D(std::move(points));
}

static void PrintCloudPreview(const geo::PointCloud3D& cloud)
{
	std::cout << "Number of points: " << cloud.Count() << std::endl;
	glm::vec3 center = cloud.Centroid();
	std::cout << "Centroid: " << "(" << center.x << ", " << center.y << ", " << center.z << ")\n";

	int i = 0;
	for (const auto& p : cloud)
	{
		std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")\n";
		i++;
		if (i > 10) {
			break;
		}
	}
}

static void PrintRigidTransform(const geo::RigidTransform& tf)
{
	std::cout << std::fixed << std::setprecision(6);

	std::cout << "Rotation (R):\n";
	for (int r = 0; r < 3; ++r)
	{
		std::cout << "  [ ";
		for (int c = 0; c < 3; ++c)
		{
			std::cout << std::setw(10) << tf.rot[c][r] << " ";
		}
		std::cout << "]\n";
	}

	std::cout << "\nTranslation (t):\n";
	std::cout << "  [ "
		<< tf.t.x << ", "
		<< tf.t.y << ", "
		<< tf.t.z << " ]\n";

	std::cout << "\nDeterminant(R): "
		<< glm::determinant(tf.rot)
		<< "\n\n";
}

// ------------------------------------------------------------
// Utility: Convert geo::PointCloud3D -> instance transforms
// ------------------------------------------------------------
static void BuildInstanceTransforms(const geo::PointCloud3D& cloud, std::vector<Matrix>& transforms, float scale = 0.1f)
{
    for (size_t i = 0; i < cloud.Count(); i++)
    {
        glm::vec3 p = cloud[i];

        Matrix scaleMatrix = MatrixScale(scale, scale, scale);
        Matrix translationMatrix = MatrixTranslate(p.x, p.y, p.z);

        transforms[i] = MatrixMultiply(scaleMatrix, translationMatrix); // <-- weird something fishy going on!
    }
}

int main()
{
    // ---------------- Generate Test Clouds ----------------
    geo::PointCloud3D target = GenerateRandomCloud(1000, -10.0f, 10.0f);
    geo::PointCloud3D source = target;

    // Apply known transform to source
    glm::vec3 eulerRot(20.0f, 40.0f, 10.0f);

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
    glm::vec3 translation(5.0f, 3.0f, -4.0f);

    source.Transform(rotation, translation);

    float currentRMS = -1.0f;


    // ---------------- Create the Window ----------------
    InitWindow(1280, 800, "ICP Visualization");
    SetTargetFPS(60);

    // ---------------- Camera ----------------
    Camera3D camera = { 0 };
    camera.position = { 20.0f, 20.0f, 20.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // ---------------- Shader --------------------

    Shader shader = LoadShader(RESOURCES_PATH"shaders/lighting.vs", RESOURCES_PATH"shaders/lighting.fs");

    // Get shader locations
    shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(shader, "mvp");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");

    // Set shader value: ambient light level
    int ambientLoc = GetShaderLocation(shader, "ambient");
    const float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue(shader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

    // Create one light
    const Vector3 light_pos = { 50.0f, 50.0f, 10.0f };
    CreateLight(LIGHT_DIRECTIONAL, light_pos, Vector3Zero(), WHITE, shader);

    // ---------------- Sphere Mesh ----------------
    Mesh mesh = GenMeshCube(1.0f, 1.0f, 1.0f);

    Material mat = LoadMaterialDefault();
    mat.shader = shader;
    mat.maps[MATERIAL_MAP_DIFFUSE].color = RED;

    // ---------------- Instances Transforms -----------------

    std::vector<Matrix> targetTransforms;
    std::vector<Matrix> sourceTransforms;
    targetTransforms.resize(target.Count());
    sourceTransforms.resize(source.Count());

    BuildInstanceTransforms(target, targetTransforms);
    BuildInstanceTransforms(source, sourceTransforms);

    // ---------------- Main Loop ----------------
    while (!WindowShouldClose())
    {
        //UpdateCamera(&camera, CAMERA_FREE);

        // Run one ICP iteration on SPACE
        if (IsKeyPressed(KEY_SPACE))
        {
            currentRMS = geo::NaiveICP(target, source, 1);
        }

        // Build GPU instance matrices
        BuildInstanceTransforms(target, targetTransforms);
        BuildInstanceTransforms(source, sourceTransforms);

        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        // ---------------- Rendering ----------------
        BeginDrawing();
        ClearBackground(SKYBLUE);

        BeginMode3D(camera);

        DrawGrid(100, 1.0f);

        // Draw Target RED
        mat.maps[MATERIAL_MAP_DIFFUSE].color = RED;
        DrawMeshInstanced(mesh, mat, targetTransforms.data(), targetTransforms.size());

        // Draw Target RED
        mat.maps[MATERIAL_MAP_DIFFUSE].color = BLUE;
        DrawMeshInstanced(mesh, mat, sourceTransforms.data(), sourceTransforms.size());

        EndMode3D();
        
        // UI
        std::stringstream ss;
        ss << "FPS:" << GetFPS() << "\n";
        ss << "Press SPACE to run 1 ICP iteration\n";
        ss << "Current RMS: " << std::fixed << std::setprecision(6) << currentRMS;

        DrawText(ss.str().c_str(), 20, 20, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
