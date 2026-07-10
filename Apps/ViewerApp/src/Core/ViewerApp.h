#include <string>
#include <Platform/GLWindow.h>


struct ViewerAppConfig
{
    u32 width         = 1600;
    u32 height        = 900;
    std::string title = "Geo Viewer";
};

// Viewer Application
class ViewerApp final
{
public:
    ViewerApp(const ViewerAppConfig& config);

    ViewerApp(const ViewerApp&) = delete;
    ViewerApp& operator=(const ViewerApp&) = delete;

    ~ViewerApp();
public:
    // Main loop entry point.
    void Run();
    // Request shutdown.
    void Stop();
private:
    void Update(f32 dt);
    void Render();
    void RenderUI();
private:
    gl::Window m_window;
};

