#include <Core/ViewerApp.h>

int main()
{
    ViewerAppConfig config;
    config.width = 1600;
    config.height = 900;
    config.title = "Geometry!!!";

    {
        ViewerApp app(config);
        app.Run();
    }

    return 0;
}
