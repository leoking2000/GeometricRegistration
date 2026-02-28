#pragma once
#include <string>
#include <geo/GeometricRegistration.h>
#include <raylib.h>


class RaylibRenderer
{
public:
	RaylibRenderer() = default;
	~RaylibRenderer();
public:
	Camera3D& GetCamera();
	void SetClearColor(Color color);
public:
	void Init(); // TODO: add a Specification as parameter
	bool ShouldClose() const;
	void BeginFrame() const;
	void EndFrame() const;
public:
	void RenderGrid() const;
	void RenderPointCloud(const geo::PointCloud3D& cloud, Color color) const;
	void RenderText(const std::string& str, int posX, int posY, int fontSize, Color color) const;
private:
	Camera3D m_camera = {};
	Color m_clearColor = SKYBLUE;
	Shader m_shader;
	Mesh m_mesh;
	Material m_material;
};



