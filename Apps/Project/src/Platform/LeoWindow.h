#pragma once
#include <geo/utils/GeoTypes.h>
#include <glm/glm.hpp>
#include <functional>
#include <string>
#include "Keys.h"

namespace gl
{
	typedef enum : geo::u32
	{
		WIN_FLAG_DEFAULT    = 0,
		WIN_FLAG_RESIZABLE  = (1 << 0),
		WIN_FLAG_VSYNC      = (1 << 1),
		WIN_FLAG_ESC_CLOSE  = (1 << 3),
	} ConfigFlags;

	struct WindowsParameters
	{
		std::string        title       = "Leonidas Engine";
		geo::u32           width       = 1600;
		geo::u32           height      = 900;
		geo::u32           init_flags  = WIN_FLAG_DEFAULT;
	};

	void WINInitialization();
	void WINTerminate();

	class Window final
	{
	public:
		Window(WindowsParameters win_params, bool create = true);
		Window(geo::u32 width, geo::u32 height, 
			const std::string& title, geo::u32 flags = WIN_FLAG_DEFAULT, bool create = true);

		Window(const Window&) = delete;
		Window& operator=(const Window&) = delete;

		~Window();
	public:
		void Create();
		void Destroy();
	public:
		// Check if window should close (KEY_ESCAPE pressed or windows close icon clicked)
		bool ShouldClose() const;
		// Polls Events
		void PollEvents();
		// Swap buffers (double buffering)
		void SwapBuffers();
		// Close window
		void Close();
	public:
		// Set title for window
		void SetTitle(const std::string& title);
		// Enable/disable Vsync
		void SetVsync(bool vsync);
	public:
		// Get current window Size (width, height) in pixels
		glm::uvec2 Size() const;
		// Get current window half Size (width/2, height/2) in pixels
		glm::uvec2 HalfSize() const;
		// Get mouse position XY
		glm::vec2  MousePos()   const;
		// mouse cursur visibility
		void SetMouseVisibility(bool visible);
	public:
		using WindowResizeCallback = std::function<void(int, int)>;
		void SetResizeCallback(WindowResizeCallback resize_callback);

		// first input is the keycode, second is the action(press, relesed, repeat)
		using ButtonEventCallback = std::function<void(int, int)>;
		void SetKeyboardCallback(ButtonEventCallback key_callback);
		void SetMouseButtonCallBack(ButtonEventCallback mouse_key_callback);

		using MouseMoveCallback = std::function<void(float, float)>;
		void SetMouseMoveCallback(MouseMoveCallback mouse_move_callback);
	public:
		struct WinData
		{
			WindowsParameters    params;
			ButtonEventCallback  keyboardCallback;
			ButtonEventCallback  mouseKeyCallback;
			MouseMoveCallback    mouseMoveCallback;
			WindowResizeCallback windowResizeCallback;
		};
	private:
		struct GLFWwindow* m_window = nullptr;
		WinData m_data = {};
	};
}
