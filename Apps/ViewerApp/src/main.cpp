#include <geo/GeometricRegistration.h>
#include <Graphics/LeoGraphics.h>
#include <Platform/GLWindow.h>
#include <Platform/ViewerController.h>
#include <Render/MeshDrawable.h>

int main()
{
    geo::SetLogLevel(geo::LogLevel::LOG_INFO);

    geo::Mesh cpu_mesh = geo::io::LoadOBJ(RESOURCES_PATH"models/DoraColumnBase/DoraColumnBase1_low.obj").ToMesh();
    geo::BBox box = cpu_mesh.BoundingBox();

    gl::WINInitialization();
    {
        gl::Window win(1600, 900, "Geometry!!!", gl::WIN_FLAG_VSYNC);
        gl::GraphicsInitialization();

        gl::MeshDrawable mesh;
        mesh.Upload(cpu_mesh);

        gl::ViewerCamera camera;
        camera.m_target = box.Center();
        camera.m_distance = 250.0f;

        gl::ViewerController controller;

        controller.Attach(win, camera);

        gl::ShaderProgram shader(RESOURCES_PATH"shaders/viewer/MeshPhong");

        glm::mat4 proj = camera.ProjectionMatrix((geo::f32)win.Size().x / (geo::f32)win.Size().y);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        while (!win.ShouldClose())
        {
            win.PollEvents();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mesh.Draw(
                shader, proj * camera.ViewMatrix(),
                glm::mat4(1.0f), glm::vec3(1.0f),
                camera.Position()
            );

            win.SwapBuffers();
        }


    }
    gl::WINTerminate();

    return 0;
}
