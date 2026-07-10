#pragma once
#include <functional>
#include <string>
#include <filesystem>
#include <glm/glm.hpp>
#include <core/Types.h>
#include "Keys.h"

namespace gl
{
	// Configuration flags used during window creation.
	enum ConfigFlags : u32
	{
		WIN_FLAG_DEFAULT   = 0,
		WIN_FLAG_RESIZABLE = (1 << 0),
		WIN_FLAG_VSYNC     = (1 << 1),
		WIN_FLAG_ESC_CLOSE = (1 << 3),
	};

	// Parameters used when creating a window.
	struct WindowParameters
	{
		std::string        title       = "Leonidas Engine";
		u32                width       = 1600;
		u32                height      = 900;
		u32                init_flags  = WIN_FLAG_DEFAULT;
	};

	// Initializes the underlying windowing system (GLFW).
	// Must be called before creating any Window.
	void WINInitialization();

	// Shuts down the windowing system.
	void WINTerminate();

	// Wrapper around a native GLFW window.
	class Window final
	{
	public:
		// Constructs a window using the supplied parameters.
		// If create is true, Create() is called automatically.
		Window(WindowParameters win_params, bool create = true);
		Window(u32 width, u32 height, const std::string& title, 
			u32 flags = WIN_FLAG_DEFAULT, bool create = true);

		Window(const Window&) = delete;
		Window& operator=(const Window&) = delete;

		// Destroys the native window if it exists.
		~Window();
	public:
		// Creates the native window using the stored parameters.
		void Create();
		// Releases all native window resources.
		void Destroy();
		// Checks if the Window has been created.
		bool IsCreated() const;
	public:
		// Check if window should close.
		bool ShouldClose() const;
		// Processes pending window and input events.
		void PollEvents();
		// Swaps the front/back buffers.
		void SwapBuffers();
		// Requests the window to close.
		void Close();
	public:
		// Set title for window.
		void SetTitle(const std::string& title);
		// Enable/disable Vsync.
		void SetVsync(bool vsync);
	public:
		// Get current window Size (width, height) in pixels.
		glm::uvec2 Size() const;
		// Get current window half Size (width/2, height/2) in pixels.
		glm::uvec2 HalfSize() const;
		// Get mouse position XY.
		glm::vec2  MousePos()   const;
		// mouse cursur visibility.
		void SetMouseVisibility(bool visible);
		// time elapsed since GLFW was initialized(WINInitialization) in seconds
		static f64 GetTime();
	public:
		using WindowResizeCallback = std::function<void(int, int)>;
		using ButtonEventCallback  = std::function<void(int, int)>;
		using MouseMoveCallback    = std::function<void(float, float)>;
		using ScrollCallback       = std::function<void(float)>;
		using DropCallback         = std::function<void(const std::filesystem::path&)>;
	public:
		// Invoked whenever the window size changes.
		void SetResizeCallback(WindowResizeCallback resize_callback);

		// Invoked for keyboard events.
		// Parameters: key, action. action is (press, relesed, repeat).
		void SetKeyboardCallback(ButtonEventCallback key_callback);

		// Invoked for mouse button events.
		// Parameters: button, action.
		void SetMouseButtonCallback(ButtonEventCallback mouse_key_callback);

		// Invoked whenever the mouse moves.
		// Parameters: x, y.
		void SetMouseMoveCallback(MouseMoveCallback mouse_move_callback);

		// Invoked for mouse wheel scrolling.
		void SetScrollCallback(ScrollCallback cb);

		void SetDropCallback(DropCallback cb);
	public:
		// Returns Native GLFW window handle.
		void* NativeHandle();
	public:
		struct WinData
		{
			WindowParameters     params;
			ButtonEventCallback  keyboardCallback;
			ButtonEventCallback  mouseKeyCallback;
			MouseMoveCallback    mouseMoveCallback;
			ScrollCallback       mouseScrollCallBack;
			WindowResizeCallback windowResizeCallback;
			DropCallback         windowDropCallback;
		};
	private:
		struct GLFWwindow* m_window = nullptr;
		WinData m_data = {};
	};
}
