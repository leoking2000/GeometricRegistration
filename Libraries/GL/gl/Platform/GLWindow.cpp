#include <GLFW/glfw3.h> 
#include <cassert>
#include <stdio.h>
#include "GLWindow.h"

namespace gl
{
	static bool g_glfwInnit = false;

	static void glfw_error_callback(int code, const char* description)
	{
		printf("[ERROR] GLFW error %i : %s", code, description);
	}

	void WINInitialization()
	{
		assert(g_glfwInnit == false && "WIN system is already Initialized.");

		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			assert(false && "GLFW init failed.");
			std::exit(-1);
		}

		// Hits
		// OpenGL version 4.6
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, 0);

		g_glfwInnit = true;
	}

	void WINTerminate()
	{
		assert(g_glfwInnit == true && "WIN system has not been Initialized.");
		g_glfwInnit = false;
		glfwTerminate();
	}

	//-----------------------------------------------------------

	static void glfw_window_size_callback(GLFWwindow* glfw_window, int width, int height)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));

		data->params.width = (unsigned int)width;
		data->params.height = (unsigned int)height;

		if (data->windowResizeCallback)
		{
			data->windowResizeCallback(width, height);
		}
	}

	static void glfw_key_callback(GLFWwindow* glfw_window, int key, int scancode, int action, int mods)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));

		if (data->keyboardCallback)
		{
			data->keyboardCallback(key, action);
		}
	}

	static void glfw_mouse_button_callback(GLFWwindow* glfw_window, int button, int action, int mods)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));
		if (data->mouseKeyCallback)
		{
			data->mouseKeyCallback(button, action);
		}
	}

	static void glfw_cursor_position_callback(GLFWwindow* glfw_window, double xpos, double ypos)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));
		if (data->mouseMoveCallback)
		{
			data->mouseMoveCallback((float)xpos, (float)ypos);
		}
	}

	static void glfw_scroll_callback(GLFWwindow* glfw_window, double xoffset, double yoffset)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));
		if (data->mouseMoveCallback)
		{
			data->mouseScrollCallBack((float)yoffset);
		}
	}

	static void glfw_drop_callback(GLFWwindow* glfw_window, int count, const char** paths)
	{
		Window::WinData* data = reinterpret_cast<Window::WinData*>(glfwGetWindowUserPointer(glfw_window));

		if (!data->windowDropCallback) {
			return;
		}

		for (int i = 0; i < count; i++) {
			data->windowDropCallback(paths[i]);
		}
	}

	//-----------------------------------------------------------

	Window::Window(WindowParameters win_params, bool create)
	{
		m_data.params = win_params;

		if (create) {
			Create();
		}
	}

	Window::Window(u32 width, u32 height, const std::string& title, u32 flags, bool create)
		:
		Window(gl::WindowParameters{ title.c_str(), width, height, flags }, create)
	{
	}

	Window::~Window()
	{
		Destroy();
	}

	//-----------------------------------------------------------

	void Window::Create()
	{
		assert(g_glfwInnit == true && "WIN system needs to be Initialized to create window call LEO::WINInitialization()");

		if (m_window != nullptr)
		{
			glfwDestroyWindow(m_window);
		}

		// window hits
		glfwWindowHint(GLFW_RESIZABLE, m_data.params.init_flags & WIN_FLAG_RESIZABLE ? GL_TRUE : GL_FALSE);
		glfwWindowHint(GLFW_SAMPLES, 4);

		// TODO: add fullscreen support

		// create the window
		m_window = glfwCreateWindow(m_data.params.width, m_data.params.height, m_data.params.title.c_str(), NULL, NULL);

		if (m_window == nullptr)
		{
			printf("[Error] Window creation failed.");
		}

		// set user pointer (used by the callbacks)
		glfwSetWindowUserPointer(m_window, &m_data);

		// set the glfw callbacks
		glfwSetWindowSizeCallback(m_window, glfw_window_size_callback);
		glfwSetKeyCallback(m_window, glfw_key_callback);
		glfwSetMouseButtonCallback(m_window, glfw_mouse_button_callback);
		glfwSetCursorPosCallback(m_window, glfw_cursor_position_callback);
		glfwSetScrollCallback(m_window, glfw_scroll_callback);
		glfwSetDropCallback(m_window, glfw_drop_callback);

		glfwMakeContextCurrent(m_window);
		glfwSwapInterval(m_data.params.init_flags & WIN_FLAG_VSYNC ? 1 : 0);
	}

	void Window::Destroy()
	{
		if (m_window != nullptr)
		{
			glfwDestroyWindow(m_window);
			m_window = nullptr;
		}
	}

	bool Window::IsCreated() const
	{
		return m_window != nullptr;
	}

	//-----------------------------------------------------------

	bool Window::ShouldClose() const
	{
		return glfwWindowShouldClose(m_window);
	}

	void Window::Close()
	{
		glfwSetWindowShouldClose(m_window, true);
	}

	void Window::PollEvents()
	{
		glfwPollEvents();

		if (m_data.params.init_flags & WIN_FLAG_ESC_CLOSE)
		{
			if (glfwGetKey(m_window, KEY_ESCAPE) == GLFW_PRESS)
			{
				Close(); // close window
			}
		}
	}

	void Window::SwapBuffers()
	{
		glfwSwapBuffers(m_window);
	}

	//-----------------------------------------------------------

	void Window::SetTitle(const std::string& title)
	{
		glfwSetWindowTitle(m_window, title.c_str());
		m_data.params.title = title;
	}

	void Window::SetVsync(bool vsync)
	{
		glfwSwapInterval(vsync ? 1 : 0);
	}

	//-----------------------------------------------------------

	glm::uvec2 Window::Size() const
	{
		return glm::uvec2(m_data.params.width, m_data.params.height);
	}

	glm::uvec2 Window::HalfSize() const
	{
		return glm::uvec2(m_data.params.width / 2, m_data.params.height / 2);
	}

	//-----------------------------------------------------------

	glm::vec2 Window::MousePos() const
	{
		double xpos, ypos;
		glfwGetCursorPos(m_window, &xpos, &ypos);

		return glm::vec2((float)xpos, (float)ypos);
	}

	void Window::SetMouseVisibility(bool visible)
	{
		glfwSetInputMode(m_window, GLFW_CURSOR, visible ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
	}

	f64 Window::GetTime()
	{
		return glfwGetTime();
	}

	//-----------------------------------------------------------

	void Window::SetResizeCallback(WindowResizeCallback resize_callback)
	{
		m_data.windowResizeCallback = resize_callback;
	}

	void Window::SetKeyboardCallback(ButtonEventCallback key_callback)
	{
		m_data.keyboardCallback = key_callback;
	}

	void Window::SetMouseButtonCallback(ButtonEventCallback mouse_key_callback)
	{
		m_data.mouseKeyCallback = mouse_key_callback;
	}

	void Window::SetMouseMoveCallback(MouseMoveCallback mouse_move_callback)
	{
		m_data.mouseMoveCallback = mouse_move_callback;
	}

	void Window::SetScrollCallback(ScrollCallback mouse_scroll_callback)
	{
		m_data.mouseScrollCallBack = mouse_scroll_callback;
	}

	void Window::SetDropCallback(DropCallback cb)
	{
		m_data.windowDropCallback = cb;
	}

	void* Window::NativeHandle()
	{
		return m_window;
	}
}
