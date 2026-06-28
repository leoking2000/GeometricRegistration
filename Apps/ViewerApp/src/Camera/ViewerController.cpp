#include "ViewerController.h"

namespace gl
{
	void ViewerController::Attach(gl::Window& window, ViewerCamera& camera)
	{
		m_camera = &camera;

		window.SetMouseMoveCallback(
			[this](float x, float y)
			{
				OnMouseMove(x, y);
			});

		window.SetMouseButtonCallback(
			[this](int button, int action)
			{
				OnMouseButton(button, action);
			});

		window.SetScrollCallback(
			[this](float yOffset)
			{
				OnScroll(yOffset);
			}
		);
	}

	void ViewerController::OnMouseMove(geo::f32 x, geo::f32 y)
	{
		if (!m_camera) return;

		glm::vec2 current(x, y);
		glm::vec2 delta = current - m_lastMouse;
		m_lastMouse = current;

		const float sensitivity = 0.005f;

		if (m_leftDown)
		{
			m_camera->Orbit(-delta.x * sensitivity, -delta.y * sensitivity);
		}

		if (m_midDown)
		{
			float panSpeed = 0.001f * m_camera->m_distance;

			m_camera->Pan(-m_camera->Right() * delta.x * panSpeed + m_camera->Up() * delta.y * panSpeed);
		}
	}

	void ViewerController::OnMouseButton(geo::i32 button, geo::i32 action)
	{
		if (button == MOUSE_BUTTON_LEFT)
			m_leftDown = (action == KEY_PRESS);

		if (button == MOUSE_BUTTON_MIDDLE)
			m_midDown = (action == KEY_PRESS);
	}

	void ViewerController::OnScroll(geo::f32 yOffset)
	{
		if (!m_camera) return;

		m_camera->Zoom(-yOffset * 0.5f);
	}
}
