#pragma once
#include <Platform/GLWindow.h>
#include "ViewerCamera.h"

namespace gl
{
	class ViewerController
	{
	public:
		void Attach(gl::Window& window, ViewerCamera& camera);

		void OnMouseMove(f32 x, f32 y);
		void OnMouseButton(i32 button, i32 action);
		void OnScroll(f32 yOffset);
	private:
		ViewerCamera* m_camera = nullptr;
		glm::vec2 m_lastMouse = glm::vec2(0.0f);
		bool m_leftDown = false;
		bool m_midDown = false;
	};



}
