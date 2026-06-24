#include <geo/GeometricRegistration.h>
#include <Graphics/LeoGraphics.h>
#include <Platform/GLWindow.h>
#include <Platform/ViewerController.h>
#include <Render/MeshDrawable.h>
#include <Render/PointCloudDrawable.h>

#include <glm/gtc/matrix_transform.hpp>

static glm::mat4 AlignScans(const geo::Mesh& target, const geo::Mesh& source)
{
    geo::Random rng{ 8888 };

    geo::DistanceField df;
    geo::DistanceFieldParameters df_parmas;
    df_parmas.bounding_box = target.BoundingBox();
    df_parmas.max_distance = 0.25f * df_parmas.bounding_box.Radius();
    df_parmas.cell_size = 4.0f;
    df.Build(df_parmas, target);

    geo::PointCloud3D target_cloud = target.SamplePointsUniform(30000, rng, true);
    geo::PointCloud3D source_cloud = source.SamplePointsUniform(30000, rng, true);
    geo::PointCloud3D subSource = source.SamplePointsUniform(1000, rng, false);

    geo::KDTree tree(target_cloud.GetPoints());

    geo::EfficientICPParams params;
    params.seed = 8888;
    params.esaIterations = 2000;

    params.icpParams.maxIterations = 500;
    params.icpParams.p = 0.1f;

    //geo::LeastSquaresICPParameters ls_params;
    //ls_params.useNormals = true;
    //geo::ICPResult result = geo::LeastSquaresICP(target_cloud, source_cloud, tree, ls_params);

    //geo::ICPResult result = geo::SparseICPPointToPlane(target_cloud, source_cloud, tree, params.icpParams);
    geo::EfficientICPResult result = geo::EfficientICP(target_cloud, source_cloud, subSource, tree, df, params);

    glm::mat4 matrix = result.transform.ToMat4();

    return matrix;
}

int main()
{
    geo::SetLogLevel(geo::LogLevel::LOG_INFO);

    std::cout << "Loading Models\n";
    geo::Mesh target = geo::Mesh::Load(RESOURCES_PATH"models/scans/ravenna/ravenna001.ply");
    geo::Mesh source = geo::Mesh::Load(RESOURCES_PATH"models/scans/ravenna/ravenna002.ply");
    std::cout << "Done\n============================\n\n";

    glm::mat4 matrix = AlignScans(target, source);
    
    gl::WINInitialization();
    {
        gl::Window win(1600, 900, "Geometry!!!", gl::WIN_FLAG_VSYNC | gl::WIN_FLAG_ESC_CLOSE);
        gl::GraphicsInitialization();

        gl::MeshDrawable mesh_target;
        mesh_target.Upload(target);

        gl::MeshDrawable mesh_source;
        mesh_source.Upload(source);

        gl::ViewerCamera camera;
        geo::BBox box = target.BoundingBox();
        camera.m_target = box.Center();
        camera.m_distance = 0.5f * box.MaxSize();

        gl::ViewerController controller;
        controller.Attach(win, camera);

        gl::ShaderProgram shader(RESOURCES_PATH"shaders/viewer/MeshPhong");
        //gl::ShaderProgram shader(RESOURCES_PATH"shaders/viewer/PointShader");

        glm::mat4 proj = camera.ProjectionMatrix((geo::f32)win.Size().x / (geo::f32)win.Size().y);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        while (!win.ShouldClose())
        {
            win.PollEvents();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            //cloud.Draw(shader, proj * camera.ViewMatrix(), glm::vec3(1.0f), 0.1f);

            mesh_target.Draw(
                shader, proj * camera.ViewMatrix(),
                glm::mat4(1.0f), glm::vec3(0.5f, 0.0f, 0.0f), // RED
                camera.Position()
            );

            mesh_source.Draw(
                shader, proj * camera.ViewMatrix() * matrix,
                matrix, glm::vec3(0.0f, 0.5f, 0.0f), // GREEN
                camera.Position()
            );

            //mesh_source.Draw(
            //    shader, proj * camera.ViewMatrix(),
            //    glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.5f), // BLUE
            //    camera.Position()
            //);

            win.SwapBuffers();
        }


    }
    gl::WINTerminate();

    return 0;
}
