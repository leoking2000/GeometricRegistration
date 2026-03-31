#include "RaylibRenderer.h"
#include <raymath.h>
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

// Utility: Convert geo::PointCloud3D -> instance transforms
static void BuildInstanceTransforms(const geo::PointCloud3D& cloud, std::vector<Matrix>& transforms, float scale = 0.1f)
{
	for (size_t i = 0; i < cloud.Size(); i++)
	{
		glm::vec3 p = cloud[i];

		Matrix scaleMatrix = MatrixScale(scale, scale, scale);
		Matrix translationMatrix = MatrixTranslate(p.x, p.y, p.z);

		transforms[i] = MatrixMultiply(scaleMatrix, translationMatrix); // <-- weird something fishy going on!
	}
}

RaylibRenderer::~RaylibRenderer()
{
	CloseWindow();
}

Camera3D& RaylibRenderer::GetCamera()
{
	return m_camera;
}

void RaylibRenderer::SetClearColor(Color color)
{
	m_clearColor = color;
}

void RaylibRenderer::Init()
{
	// ---------------- Create the Window ----------------
	InitWindow(1600, 900, "ICP Visualization");
	SetTargetFPS(60);

	// ---------------- Camera ----------------
	m_camera = { 0 };
	m_camera.position = { 15.0f, 15.0f, 15.0f };
	m_camera.target = { 0.0f, 0.0f, 0.0f };
	m_camera.up = { 0.0f, 1.0f, 0.0f };
	m_camera.fovy = 45.0f;
	m_camera.projection = CAMERA_PERSPECTIVE;

	// ---------------- Shader --------------------

	m_shader = LoadShader(RESOURCES_PATH"shaders/lighting.vs", RESOURCES_PATH"shaders/lighting.fs");

	// Get shader locations
	m_shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(m_shader, "mvp");
	m_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(m_shader, "viewPos");
	m_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(m_shader, "instanceTransform");

	// Set shader value: ambient light level
	int ambientLoc = GetShaderLocation(m_shader, "ambient");
	const float ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	SetShaderValue(m_shader, ambientLoc, ambient, SHADER_UNIFORM_VEC4);

	// Create one light
	const Vector3 light_pos = { 50.0f, 50.0f, 10.0f };
	CreateLight(LIGHT_DIRECTIONAL, light_pos, Vector3Zero(), WHITE, m_shader);

	// ---------------- Cube Mesh ----------------
	m_mesh = GenMeshCube(1.0f, 1.0f, 1.0f);

	m_material = LoadMaterialDefault();
	m_material.shader = m_shader;
	m_material.maps[MATERIAL_MAP_DIFFUSE].color = RED;

}

bool RaylibRenderer::ShouldClose() const
{
	return WindowShouldClose();
}

void RaylibRenderer::BeginFrame() const
{
	float cameraPos[3] = { m_camera.position.x, m_camera.position.y, m_camera.position.z };
	SetShaderValue(m_shader, m_shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

	BeginDrawing();
	ClearBackground(SKYBLUE);
}


void RaylibRenderer::RenderGrid() const
{
	BeginMode3D(m_camera);
	DrawGrid(100, 1.0f);
	EndMode3D();
}

void RaylibRenderer::RenderPointCloud(const geo::PointCloud3D& cloud, Color color) const
{
	BeginMode3D(m_camera);

	// Instances Transforms
	static std::vector<Matrix> transforms;
	transforms.resize(cloud.Size());
	BuildInstanceTransforms(cloud, transforms, 0.1f);

	// Draw Cloud
	m_material.maps[MATERIAL_MAP_DIFFUSE].color = color;
	DrawMeshInstanced(m_mesh, m_material, transforms.data(), (int)transforms.size());

	EndMode3D();
}

void RaylibRenderer::RenderText(const std::string& str, int posX, int posY, int fontSize, Color color) const
{
	DrawText(str.c_str(), posX, posY, fontSize, color);
}

void RaylibRenderer::EndFrame() const
{
	EndDrawing();
}


