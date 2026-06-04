#pragma once
#include "ViewerCamera.h"
#include "GLWindow.h"

namespace gl
{
	class ViewerController
	{
	public:
		void Attach(gl::Window& window, ViewerCamera& camera);

		void OnMouseMove(geo::f32 x, geo::f32 y);
		void OnMouseButton(geo::i32 button, geo::i32 action);
		void OnScroll(geo::f32 yOffset);
	private:
		ViewerCamera* m_camera = nullptr;
		glm::vec2 m_lastMouse = glm::vec2(0.0f);
		bool m_leftDown = false;
		bool m_midDown = false;
	};



}
