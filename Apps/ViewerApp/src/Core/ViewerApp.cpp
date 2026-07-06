#include <geo/GeometricRegistration.h>
#include <Graphics/LeoGraphics.h>
#include <Platform/GLWindow.h>
#include <Camera/ViewerController.h>
#include <Drawable/MeshDrawable.h>
#include <Drawable/PointCloudDrawable.h>
#include <glm/gtc/matrix_transform.hpp>
#include "ViewerApp.h"

static glm::mat4 AlignScans(const geo::Mesh& target, const geo::Mesh& source)
{
    geo::Random rng{ 8888 };

    geo::DistanceField df;
    geo::DistanceFieldParameters df_parmas;
    df_parmas.bounding_box = target.BoundingBox();
    df_parmas.max_distance = 0.25f * df_parmas.bounding_box.Radius();
    df_parmas.cell_size = 3.0f;
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


ViewerApp::ViewerApp(const ViewerAppConfig& config)
	:
	m_window(config.width, config.height, config.title, gl::WIN_FLAG_VSYNC | gl::WIN_FLAG_ESC_CLOSE, false)
{
	gl::WINInitialization();
	m_window.Create();

	gl::GraphicsInitialization();
}

ViewerApp::~ViewerApp()
{
	m_window.Destroy();
	gl::WINTerminate();
}

void ViewerApp::Run()
{
    geo::SetLogLevel(geo::LogLevel::LOG_INFO);

    //std::cout << "Loading Models\n";
    
    geo::Mesh target = geo::Mesh::Load(RESOURCES_PATH"models/fragments/Tombstone/Tombstone1_med.obj");
    //geo::Mesh source = geo::Mesh::Load(RESOURCES_PATH"models/fragments/Tombstone/Tombstone1_med.obj");

    //geo::PointCloud3D cpu_cloud = geo::PointCloud3D::Load(RESOURCES_PATH"models/scans/owl/owl-decimate10pc-textured.ply");
    //gl::PointCloudDrawable gpu_cloud(cpu_cloud.GetPoints());

    //glm::vec3 eulerRot(60.0f, -50.0f, 20.5f);
    //glm::vec3 translation(-1.0f, 3.0f, 1.0f);
    //glm::mat4 Rx = glm::rotate(glm::mat4(1.0f),
    //    glm::radians(eulerRot.x),
    //    glm::vec3(1, 0, 0));
    //glm::mat4 Ry = glm::rotate(glm::mat4(1.0f),
    //    glm::radians(eulerRot.y),
    //    glm::vec3(0, 1, 0));
    //glm::mat4 Rz = glm::rotate(glm::mat4(1.0f),
    //    glm::radians(eulerRot.z),
    //    glm::vec3(0, 0, 1));
    //glm::mat4 rotation = Ry * Rx * Rz;
    //geo::RigidTransform gt = { glm::mat3(rotation), translation };
    //source.Transform(gt);

    //std::cout << "Done\n============================\n\n";

    //glm::mat4 matrix = AlignScans(target, source);

    gl::MeshDrawable mesh_target;
    std::vector<glm::uvec3> indeces;
    indeces.reserve(target.TriangleCount());
    for (geo::index_t i = 0; i < target.TriangleCount(); i++)
    {
        indeces.emplace_back(target.Triangle(i).vertexIndices);
    }

    mesh_target.Upload(target.GetVertices(), target.GetNormals(), std::move(indeces));

    //gl::MeshDrawable mesh_source;
    // mesh_source.Upload(source);

    gl::ViewerCamera camera;
    geo::BBox box = target.BoundingBox();
    camera.m_target = box.Center();
    camera.m_distance = 0.5f * box.MaxSize();

    gl::ViewerController controller;
    controller.Attach(m_window, camera);

    gl::ShaderProgram shader(RESOURCES_PATH"shaders/viewer/MeshPhong");
    //gl::ShaderProgram shader(RESOURCES_PATH"shaders/viewer/PointShader");

    glm::mat4 proj = camera.ProjectionMatrix((geo::f32)m_window.Size().x / (geo::f32)m_window.Size().y);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    while (!m_window.ShouldClose())
    {
        m_window.PollEvents();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //gpu_cloud.Draw(shader, proj * camera.ViewMatrix(), glm::vec3(1.0f), 0.1f);

        mesh_target.Draw(
            shader, proj * camera.ViewMatrix(), 
            glm::vec3(1.0f, 1.0f, 1.0f), // RED
            camera.Position()
        );

        //mesh_source.Draw(
        //    shader, proj * camera.ViewMatrix() * matrix,
        //    matrix, glm::vec3(0.0f, 0.5f, 0.0f), // GREEN
        //    camera.Position()
        //);

        //mesh_source.Draw(
        //    shader, proj * camera.ViewMatrix(),
        //    glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.5f), // BLUE
        //    camera.Position()
        //);

        m_window.SwapBuffers();
    }

}

void ViewerApp::Stop()
{
    m_window.Close();
}

void ViewerApp::Update(geo::f32 dt)
{

}

void ViewerApp::Render()
{

}

void ViewerApp::RenderUI()
{

}
