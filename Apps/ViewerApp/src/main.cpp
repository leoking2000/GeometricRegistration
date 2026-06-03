#include <geo/GeometricRegistration.h>
#include <Graphics/LeoGraphics.h>
#include <Platform/LeoWindow.h>

int main()
{
    geo::SetLogLevel(geo::LogLevel::LOG_VERBOSE);

    gl::WINInitialization();
    {
        gl::Window win(800, 600, "Geometry!!!", gl::WIN_FLAG_VSYNC);
        gl::GraphicsInitialization();

        while (!win.ShouldClose())
        {
            win.PollEvents();




            win.SwapBuffers();
        }


    }
    gl::WINTerminate();

    return 0;
}
