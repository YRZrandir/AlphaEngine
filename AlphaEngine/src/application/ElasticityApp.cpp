#include "ElasticityApp.h"
#include <memory>
#include <fstream>
#include <imgui/imgui.h>
#include <tinyxml2.h>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>

#include "input/Input.h"
#include "lighting/Light.h"
#include "model/CHalfEdgeMesh.h"
#include "model/TetraMesh.h"
#include "model/HalfEdgeMesh.h"
#include "model/ModelLoader.h"
#include "model/Terrain.h"
#include "model/RigidStatic.h"
#include "model/RigidSDF.h"
#include "model/SphereMesh.h"
#include "PBD/PBDTetraModel.h"
#include "PBD/MetaballModel.h"
#include "PBD/MetaballCutter.h"
#include "PBD/MetaballCuttingModel.h"
#include "PBD/SphereTreeHelper.h"
#include "PBD/ScalpelRect.h"
#include "PBD/PBDGPUTetraModel.h"
#include "PD/PDMetaballModel.h"
#include "PD/PDGPUMetaballModel.h"
#include "PD/PDTetraModel.h"
#include "PD/PDMetaballModelFC.h"
#include "FEM/FEMSolver.h"
#include "ui/ImGuiFileDialog.h"
#include "util/Scene.h"
#include "util/PBDScene.h"
#include "util/SceneObject.h"
#include "util/Camera.h"
#include "util/util.h"
#include "util/Shader.h"
#include "util/Intersection.h"
#include "util/Instrumentor.h"
#include "util/GlobalTimer.h"
#include "CVT/WeightedCVT.h"
#include "haptic/Haptic.h"

namespace
{
void UpdateHaptics()
{
    static glm::vec3 sPos( -FLT_MAX );

    Haptic& haptic = Haptic::Get();
    glm::vec3 pos = ToGLM( haptic.GetPos() );
    pos = pos * 1.f / (float)haptic.GetInfo().m_workspaceRadius;
    pos = glm::vec3( pos.y, pos.z, pos.x );

    auto ball = Scene::active->GetChild<CHalfEdgeMesh<ItemBase>>( "haptic" );
    ball->mTransform.SetPos( pos );
    ball->mTransform.SetScale( glm::vec3( 0.5f ) );

    if (sPos[0] == -FLT_MAX)
        sPos = pos;

    auto model = Scene::active->GetChild<PBD::PBDTetraModel>();
    auto mesh = model->Mesh();
    const auto& points = model->Points();

    glm::vec3 force( 0.f );
    for (int i = 0; i < points.size(); i++)
    {
        float dist = glm::distance( points[i], pos );
        if (glm::distance( points[i], pos ) < 0.5f)
        {
            glm::vec3 n = glm::normalize( pos - points[i] );
            float d = 0.5f - dist;
            force += 3.f * d * n * (float)haptic.GetInfo().m_maxLinearForce;
            model->AddExternalForce( i, -d * n * 300.f );
        }
    }

    haptic.AddForce( Vector3( force.z, force.x, force.y ) );
}

void UpdateHaptics2()
{
    auto model = Scene::active->GetChild<PBD::PBDTetraModel>();
    auto mesh = model->Mesh();
    auto& points = model->Points();
    Haptic& haptic = Haptic::Get();
    glm::vec3 pos = ToGLM( haptic.GetPos() );
    pos = pos * 2.f / (float)haptic.GetInfo().m_workspaceRadius;
    pos = glm::vec3( pos.y, pos.z, pos.x );

    auto ball = Scene::active->GetChild<CHalfEdgeMesh<ItemBase, CGAL::Simple_cartesian<float>>>( "haptic" );
    ball->mTransform.SetPos( pos );
    ball->mTransform.SetScale( glm::vec3( 0.5f ) );

    glm::vec3 force( 0.f );
    float max_d = -1.f;
    for (int hf = 0; hf < mesh->GetBorderFaceNum(); hf++)
    {
        const auto& face = mesh->mBorderFaces[hf];
        glm::vec3 p0 = points[face.a];
        glm::vec3 p1 = points[face.b];
        glm::vec3 p2 = points[face.c];
        int topo = -1;
        glm::vec3 proj;
        float d = glm::MinDistToTriangle( pos, p0, p1, p2, &topo, &proj );
        glm::vec3 f;
        if (d < 0.25f)
        {
            glm::vec3 normal = glm::normalize( pos - proj );
            f = 2.f * (0.25f - d) * normal;
            max_d = std::max( 0.25f - d, max_d );
            //std::cout << f << std::endl;
            model->AddExternalForce( face.a, -0.1f * f );
            model->AddExternalForce( face.b, -0.1f * f );
            model->AddExternalForce( face.c, -0.1f * f );

            force += f * (float)haptic.GetInfo().m_maxLinearForce;
        }
    }

    force = glm::min( 5.f * max_d, 1.f ) * (float)haptic.GetInfo().m_maxLinearForce * glm::normalize( force );
    haptic.AddForce( Vector3( force.z, force.x, force.y ) );
}
}

ElasticityApp::ElasticityApp( std::string title, size_t width, size_t height )
    :ApplicationBase( title, width, height )
{
}

ElasticityApp::~ElasticityApp()
{

    //Haptic::Disconnect();
    Instrumentor::Get().EndSession();
}

void ElasticityApp::Init()
{
    glViewport( 0, 0, static_cast<GLsizei>(_width), static_cast<GLsizei>(_height) );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
    //glEnable( GL_MULTISAMPLE );
    glEnable( GL_TEXTURE_2D );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );

    Scene::active = std::make_unique<PBDScene>( true );

    //Camera
    auto cam = Scene::active->AddChild( std::make_unique<FreeCamera>( "free", glm::vec3( 0, 0.1, -2 ), 0.02f ) );
    Camera::main = cam;
    Camera::current = Camera::main;
    cam->mTransform.SetPos( glm::vec3( -0.42, 0.397, -0.537 ) );
    cam->_yaw = 28.4f;
    cam->_pitch = 35.4f;
    //cam->mTransform.SetPos( glm::vec3( 0.582, -1.04, -3.09 ) );
    //cam->_yaw = -12.6;
    //cam->_pitch = -20.4;
    /*
    0.582, -1.04, -3.09
    -12.6
    -20.4
    */

    //Light
    //Scene::active->AddChild( std::make_unique<PointLight>(
    //    "uPointLights[0]",
    //    glm::vec3( -1.4, 1.4, 2.5 ),
    //    glm::vec3( 1, 1, 1 ),
    //    70.0f,
    //    1.0f, 0.2f, 0.1f ) );

    auto light = Scene::active->AddChild( std::make_unique<DirLight>( "dirlight", glm::vec3( 0.3, -1, 0 ), glm::vec3( 0.f ), glm::vec3( 1.f ), 5.0f, glm::vec3( 1.f ) ) );

    //Shaders
    const std::string SHADER_PATH = "res/shaders/";
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "depth.glsl" ), "depth" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "depth_pd.glsl" ), "depth_pd" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "tetra.glsl" ), "tetra" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model.glsl" ), "model" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_ball.glsl" ), "model_ball" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_no_normal.glsl" ), "model_no_normal" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_vtx_color.glsl" ), "model_vtx_color" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_pbd.glsl" ), "model_pbd" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_pd.glsl" ), "model_pd" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "model_vtx_color_pbd.glsl" ), "model_vtx_color_pbd" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "normal_visualization.glsl" ), "normal_visualization" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "GLLineSegments.glsl" ), "GLLineSegments" );
    Shader::Add( std::make_unique<Shader>( SHADER_PATH + "GLPoints.glsl" ), "GLPoints" );

    Shader::Find( "model" )->BuildShaderInfo();

#ifdef EXAMPLE_ARMA
    cam->mTransform.SetPos( glm::vec3( 1.4, 0.550, 0.530 ) );
    cam->_yaw = 250.0f;
    cam->_pitch = 25.0f;
    LoadSceneFile( "D:/models/arma/model.xml" );

#endif

#ifdef EXAMPLE_DUCK
    cam->mTransform.SetPos( glm::vec3( 1.4, 0.550, 0.530 ) );
    cam->_yaw = 250.0f;
    cam->_pitch = 25.0f;
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/duck/duck_coarse.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "D:/models/duck/duck_coarse-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.06f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 100.0f;
    cfg._nb_points = 2400;
    cfg._dt = 0.01f;
    cfg._nb_solve = 10;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return v[1] < -0.3f || v[1] > 0.3f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( ModelLoader::LoadPDMetaballHalfEdgeMesh( "D:/models/duck/ducks.obj" ) );
    surface->mLayer = 1;
    auto pdmetaball = Scene::active->AddChild( std::make_unique<PD::PDGPUMetaballModel>( cfg, surface ) );
    pdmetaball->mLayer = 1;

    auto* cylinder = Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( "D:/models/duck_example/cylinders.obj" ) );
    cylinder->_material_main->SetDiffuseColor( 0.0, 0.4, 0.0 );
    cylinder->mTransform.SetPos( glm::vec3( 0, -0.2, 0 ) );
    cylinder->mTransform.SetScale( glm::vec3( 0.5f, 0.5f, 0.7f ) );
    cylinder->mName = "cy1";
    cylinder->_no_normal = true;
    auto* cylinder2 = Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( "D:/models/duck_example/cylinders.obj" ) );
    cylinder2->mTransform.SetRotation( glm::vec3( 0, glm::radians( 90.f ), 0 ) );
    cylinder2->mTransform.SetPos( glm::vec3( 0, -0.2, 0 ) );
    cylinder2->mTransform.SetScale( glm::vec3( 0.5f, 0.5f, 0.7f ) );
    cylinder2->mName = "cy2";
    cylinder2->_no_normal = true;

    auto* bar = Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( "D:/models/duck_example/bar.obj" ) );
    bar->_use_face_normal = true;
    bar->UpdateNormal();
    bar->UpdateAttrBuffer();
    bar->_material_main->mAlpha = 0.4f;
    bar->mLayer = 2;
    auto* bar2 = Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( "D:/models/duck_example/bar2.obj" ) );
    bar2->_material_main->mAlpha = bar->_material_main->mAlpha;
    bar2->mLayer = 0;
#endif

#ifdef EXAMPLE_SKIN
    cam->mTransform.SetPos( glm::vec3( 0, 0, -2 ) );
    cam->_yaw = 0;
    cam->_pitch = 0;
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/arma/armadillo_coarse.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "res/models/liver/liver-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.03f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 500.0f;
    cfg._nb_points = 3000;
    cfg._dt = 0.01f;
    cfg._nb_solve = 8;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return v[0] > 0.3f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( ModelLoader::LoadPDMetaballHalfEdgeMesh( "D:/models/arma/armadilloss.obj" ) );
    surface->_gpu_skinning = true;
    Scene::active->AddChild( std::make_unique<PD::PDGPUMetaballModel>( cfg, surface ) );
#endif

#ifdef EXAMPLE_BAR_BALL
    cam->mTransform.SetPos( glm::vec3( -0.42, 0.397, -0.537 ) );
    cam->_yaw = 28.4;
    cam->_pitch = 35.4;
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/bar.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "D:/models/bar-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.01f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 100.0f;
    cfg._nb_points = 951;
    cfg._dt = 0.03f;
    cfg._nb_solve = 10;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return v[0] < -0.3f || v[0] > 0.3f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( ModelLoader::LoadPDMetaballHalfEdgeMesh( "D:/models/bars.obj" ) );
    Scene::active->AddChild( std::make_unique<PD::PDGPUMetaballModel>( cfg, surface ) );
#endif
#ifdef EXAMPLE_BAR_PBD
    Scene::active->AddChild( std::make_unique<PBD::PBDTetraModel>( "D:/models/bar.obj", "D:/models/bars.obj", 100.f ) );
#endif

#ifdef EXAMPLE_BAR_PD
    //Scene::active->AddChild( std::make_unique<PD::PDTetraModel>( "D:/models/bunny/bunnys.obj", "D:/models/bunny.obj", 1.f,
    //    []( glm::vec3 v )->bool { return v[1] > 0.2f; } ) );
#endif

#ifdef EXAMPLE_BAR_0
    cam->mTransform.SetPos( glm::vec3( 0.0, 0.0, -1.0 ) );
    cam->_yaw = 0;
    cam->_pitch = 0;
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/bar.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "res/models/liver/liver-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.03f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 500.0f;
    cfg._nb_points = 951;
    cfg._dt = 0.03f;
    cfg._nb_solve = 10;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return v[0] > 0.3f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( ModelLoader::LoadPDMetaballHalfEdgeMesh( "D:/models/bars.obj" ) );
    Scene::active->AddChild( std::make_unique<PD::PDMetaballModel>( cfg, surface ) );
#endif
#ifdef EXAMPLE_BAR_1
    FEMConfig femconfig;
    femconfig.path = "D:/models/bar2.obj";
    femconfig.surface_path = "D:/models/bar2.obj";
    femconfig.density = 1.0;
    femconfig.mu = 1750.f;
    femconfig.lambda = 400.f;
    femconfig.substep = 100;
    femconfig.dt = 0.03f;
    Scene::active->AddChild( std::make_unique<FEMSolver>( femconfig ) );
#endif
#ifdef EXAMPLE_BAR_2
    Scene::active->AddChild( ModelLoader::LoadMetaballModel( "D:/models/bar-medial.sph", "D:/models/bars.obj", 1.0f ) );
#endif

#ifdef EXAMPLE_SPHERETREE
    cam->mTransform.SetPos( glm::vec3( 0, 0.45, 1.6 ) );
    cam->_yaw = 180;
    cam->_pitch = 25.0f;
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 2;
    cfg._coarse_surface = "D:/models/duck/duck_coarse.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "D:/models/duck/duck_coarse-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.06f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 100.0f;
    cfg._nb_points = 485;
    cfg._dt = 0.01f;
    cfg._nb_solve = 4;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return v[1] < -0.2f || v[1] > 0.1f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( ModelLoader::LoadPDMetaballHalfEdgeMesh( "D:/models/duck/ducks.obj" ) );
    surface->mLayer = 1;
    auto pdmetaball = Scene::active->AddChild( std::make_unique<PD::PDMetaballModel>( cfg, surface ) );
    pdmetaball->mLayer = 1;

    auto* cylinder = Scene::active->AddChild( ModelLoader::LoadHalfEdgeMeshFromObj( "D:/models/duck_example/cylinders.obj" ) );
    cylinder->_material_main->SetDiffuseColor( 0.0, 0.4, 0.0 );
    cylinder->mTransform.SetPos( glm::vec3( 0, -0.2, 0 ) );
    cylinder->mTransform.SetScale( glm::vec3( 0.5f, 0.5f, 0.7f ) );
    cylinder->mName = "cy1";
    cylinder->_no_normal = true;
    auto* cylinder2 = Scene::active->AddChild( ModelLoader::LoadHalfEdgeMeshFromObj( "D:/models/duck_example/cylinders.obj" ) );
    cylinder2->mTransform.SetRotation( glm::vec3( 0, glm::radians( 90.f ), 0 ) );
    cylinder2->mTransform.SetPos( glm::vec3( 0, -0.2, 0 ) );
    cylinder2->mTransform.SetScale( glm::vec3( 0.5f, 0.5f, 0.7f ) );
    cylinder2->mName = "cy2";
    cylinder2->_no_normal = true;

    auto* bar = Scene::active->AddChild( ModelLoader::LoadHalfEdgeMeshFromObj( "D:/models/duck_example/bar.obj" ) );
    bar->_use_face_normal = true;
    bar->UpdateNormal();
    bar->UpdateAttrBuffer();
    bar->_material_main->mAlpha = 0.4f;
    bar->mLayer = 2;
    auto* bar2 = Scene::active->AddChild( ModelLoader::LoadHalfEdgeMeshFromObj( "D:/models/duck_example/bar2.obj" ) );
    bar2->_material_main->mAlpha = bar->_material_main->mAlpha;
    bar2->mLayer = 0;
#endif

#ifdef EXAMPLE_PERFORMANCE
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/arma/armadillo_coarse.obj";
    cfg._fine_surface = "res/models/cactus.obj";
    cfg._metaball_path = "D:/models/duck/duck_coarse-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.06f;
    cfg._nb_lloyd = 20;
    cfg._k_attach = 1000.0f;
    cfg._k_stiff = 500.0f;
    cfg._nb_points = 1000;
    cfg._dt = 0.01f;
    cfg._nb_solve = 10;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return  v[1] > 0.3f && v[0] > 0.3f; };
    cfg._displacement = glm::vec3( 0, 0, 0 );

    auto surface = Scene::active->AddChild( std::make_unique<PDMetaballHalfEdgeMesh>( "D:/models/arma/armadillo.obj" ) );
    surface->mLayer = 1;
    auto pdmetaball = Scene::active->AddChild( std::make_unique<PD::PDGPUMetaballModel>( cfg, surface ) );
    pdmetaball->mName = "model0";
#endif

#ifdef EXAMPLE_BUNNYS
    PD::PDMetaballModelConfig cfg{};
    cfg._method = 0;
    cfg._coarse_surface = "D:/models/bunny/bunny.obj";
    cfg._fine_surface = "D:/models/bunny/bunnys.obj";
    cfg._metaball_path = "D:/models/duck/duck_coarse-medial.sph";
    cfg._density = 1.0f;
    cfg._sample_dx = 0.03f;
    cfg._nb_lloyd = 1;
    cfg._k_attach = 200.0f;
    cfg._k_stiff = 50.0f;
    cfg._nb_points = 48;
    cfg._dt = 0.005f;
    cfg._nb_solve = 5;
    cfg._physical_step = 1;
    cfg._const_type = 0;
    cfg._attach_filter = []( glm::vec3 v )->bool { return false; };
    cfg._displacement = glm::vec3( 0, 0, 0 );
    auto surface = Scene::active->AddChild( std::make_unique<PDMetaballHalfEdgeMesh>( "D:/models/bunny/bunnys.obj" ) );

    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            for (int k = 2; k <= 4; k++)
            {
                cfg._displacement = glm::vec3( i, k, j ) * 0.5f;
                auto pdmetaball = Scene::active->AddChild( std::make_unique<PD::PDMetaballModelFC>( cfg, surface ) );
            }
        }
    }

    Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( "D:/models/floor.obj" ) );
#endif
    //auto rigid_box = Scene::active->AddChild( std::make_unique<RigidStatic>( "D:/models/cylinder.obj" ) );
    //rigid_box->mTransform.Translate( { 0,-1, 0 } );
    //rigid_box->UpdateTransPos();

    //auto rigid_ball = Scene::active->AddChild( std::make_unique<RigidBall>( glm::vec3( 0, -1.5, 0 ), 2.0f ) );
    //rigid_ball->mName = "rigid_ball";

     //Scene::active->AddChild( std::make_unique<RigidSDF>( "D:/models/cylinder.obj", 0.02f ) );
     //Scene::active->AddChild( std::make_unique<PBD::GPUTetraModel>( "D:/models/arma/armadillo_coarse.obj", "D:/models/arma/armadillo_coarse.obj", 10000.f ) );

    //auto ball = Scene::active->AddChild( std::make_unique<CHalfEdgeMesh<ItemBase>>( "res/models/ball960.obj" ) );
    //ball->mName = "haptic";
    //Haptic::Init( UpdateHaptics );
    GlobalTimer::Start();
}

void ElasticityApp::PreDraw()
{
    Scene::active->Update();
    //UpdateHaptics();
    GlobalTimer::Update();
    //Scene::active->GetChild<PointLight>()->mTransform.SetPos( Camera::current->mTransform.GetPosition() );
    Scene::active->GetChild<DirLight>()->mTransform.SetPos( Camera::current->mTransform.GetPosition() );

#ifdef EXAMPLE_DUCK
    if (Input::IsKeyHeld( Input::Key::R ))
    {
        static float rad = 0.f;
        if (rad < 3.14 / 3.f)
        {
            Scene::active->GetChild<HalfEdgeMesh>( "cy1" )->mTransform.RotateAround( glm::vec3( 0.f ), glm::vec3( 0, 1, 0 ), -0.005 );
            Scene::active->GetChild<HalfEdgeMesh>( "cy2" )->mTransform.RotateAround( glm::vec3( 0.f ), glm::vec3( 0, 1, 0 ), -0.005 );
        }
        rad += 0.01f;
    }
#endif
#ifdef EXAMPLE_SPHERETREE
    if (Input::IsKeyHeld( Input::Key::R ))
    {
        static float rad = 0.f;
        if (rad < 3.14 / 6.f)
        {
            Scene::active->GetChild<HalfEdgeMesh>( "cy1" )->mTransform.Translate( glm::vec3( 0, -0.01, 0 ) );
            Scene::active->GetChild<HalfEdgeMesh>( "cy2" )->mTransform.Translate( glm::vec3( 0, -0.01, 0 ) );
        }
        rad += 0.01f;
    }
#endif
#ifdef EXAMPLE_PERFORMANCE

#endif
}

void ElasticityApp::PostDraw()
{
    Input::Update();

    //static int count = 0;
    //float diff = MeshDistance( Scene::active->GetChild<FEMSolver>()->_surface.get(),
    //    &Scene::active->GetChild<PD::PDMetaballModel>()->Surface() );
    //if (Scene::active->GetChild<FEMSolver>()->_config.simulate)
    //{
    //    std::cout << diff << std::endl;
    //    test_ss << diff << '\n';
    //    count++;
    //    if (count == 1000)
    //    {
    //        std::ofstream ofs( "./testfem.txt" );
    //        ofs << test_ss.str();
    //        ofs.flush();
    //        ofs.close();
    //    }
    //}
    //float diff = MeshDistance( Scene::active->GetChild<FEMSolver>()->_surface.get(),
    //    &Scene::active->GetChild<PBD::MetaballModel>()->Surface() );
    //std::cout << "diff=" << diff << std::endl;
    //if (Scene::active->GetChild<FEMSolver>()->_config.simulate)
    //{
    //    std::cout << diff << std::endl;
    //    test_ss << diff << '\n';
    //    count++;
    //    if (count == 1000)
    //    {
    //        std::ofstream ofs( "./testfem2.txt" );
    //        ofs << test_ss.str();
    //        ofs.flush();
    //        ofs.close();
    //    }
    //}
}

void ElasticityApp::DrawGUI()
{
    using namespace ImGui;

    Begin( "FPS" );
    Text( "Application average %.3f ms/frame (%.1f FPS)", 1000.0f / GetIO().Framerate, GetIO().Framerate );
    End();

    BeginMainMenuBar();
    if (BeginMenu( "File" ))
    {
        if (ImGui::MenuItem( "Open" ))
        {
            ImGuiFileDialog::Instance()->OpenDialog( "ChooseFileKey", "File", ".xml", "D:/models", "" );
        }
        ImGui::EndMenu();
    }
    EndMainMenuBar();
    if (ImGuiFileDialog::Instance()->Display( "ChooseFileKey" ))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            LoadSceneFile( path.c_str() );
        }
        ImGuiFileDialog::Instance()->Close();
    }

    auto fem = Scene::active->GetChild<FEMSolver>();
    if (fem)
    {
        fem->DrawGUI();
    }

    auto metaballpd = Scene::active->GetChild<PD::PDMetaballModel>();
    if (metaballpd)
    {
        //metaballpd->DrawGUI();

        if (metaballpd->_simulate)
        {
            static int counter = 0;
            static float fps = 0.f;
            if (counter < 1000)
            {
                fps += GetIO().DeltaTime;
                counter++;
            }
            else if (counter == 1000)
            {
                fps /= 1000;
                fps = 1.f / fps;
                std::cout << "Average FPS: " << fps << std::endl;
                counter++;
            }
        }
    }

    auto metaballpd2 = Scene::active->GetChild<PD::PDGPUMetaballModel>();
    if (metaballpd2)
    {
        metaballpd2->DrawGUI();

        if (metaballpd2->_simulate)
        {
            static int counter = 0;
            static float fps = 0.f;
            if (counter < 1000)
            {
                fps += GetIO().DeltaTime;
                counter++;
            }
            else if (counter == 1000)
            {
                fps /= 1000;
                fps = 1.f / fps;
                std::cout << "Average FPS: " << fps << std::endl;
                counter++;
            }
        }
    }

    auto metaballpbd = Scene::active->GetChild<PBD::MetaballModel>();
    if (metaballpbd)
    {
        metaballpbd->DrawGUI();
    }

    auto tetpd = Scene::active->GetChild<PD::PDTetraModel>();
    if (tetpd)
    {
        tetpd->DrawGUI();
    }

    for (auto cam : Scene::active->GetAllChildOfType<Camera>())
    {
        //cam->DrawGUI();
    }
}

void ElasticityApp::DrawGraphics()
{
    glViewport( 0, 0, static_cast<GLsizei>(_width), static_cast<GLsizei>(_height) );
    glClearColor( 1, 1, 1, 1 );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    Scene::active->Draw();
}

PD::PDGPUMetaballModel* ElasticityApp::LoadPDGPUMetaballModel( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    PD::PDMetaballModelConfig config;
    config._method = std::stoi( get_elem_text( root, "MetaballGenMethod" ) );
    config._coarse_surface = boost::algorithm::trim_copy( std::string( get_elem_text( root, "Surface" ) ) );
    config._metaball_path = boost::algorithm::trim_copy( std::string( get_elem_text( root, "MetaballFile" ) ) );
    config._density = std::stof( get_elem_text( root, "Density" ) );
    config._sample_dx = std::stof( get_elem_text( root, "Sampledx" ) );
    config._nb_lloyd = std::stoi( get_elem_text( root, "NumberLloyd" ) );
    config._k_attach = std::stof( get_elem_text( root, "K_Attach" ) );
    config._k_stiff = std::stof( get_elem_text( root, "K_Stiff" ) );
    config._k_edge_stiff = config._k_stiff;
    config._nb_points = std::stoi( get_elem_text( root, "NumberPoints" ) );
    config._dt = std::stof( get_elem_text( root, "dt" ) );
    config._nb_solve = std::stoi( get_elem_text( root, "NumberSolve" ) );
    config._const_type = std::stoi( get_elem_text( root, "ConstraintType" ) );
    config._attach_filter = []( glm::vec3 v )->bool { return  v[1] > 0.3f && v[0] > 0.3f; };

    std::stringstream ss( get_elem_text( root, "Displacement" ) );
    ss >> config._displacement.x >> config._displacement.y >> config._displacement.z;

    auto surface = LoadPDMetaballHalfEdgeMesh( root->FirstChildElement( "FineSurface" ) );

    auto model = Scene::active->AddChild( std::make_unique<PD::PDGPUMetaballModel>( config, surface ) );
    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name = get_elem_text( root, "Name" );
        model->mName = name;
    }

    return model;
}

PD::PDMetaballModel* ElasticityApp::LoadPDMetaballModel( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    PD::PDMetaballModelConfig config;
    config._method = std::stoi( get_elem_text( root, "MetaballGenMethod" ) );
    config._coarse_surface = boost::algorithm::trim_copy( std::string( get_elem_text( root, "Surface" ) ) );

    config._metaball_path = boost::algorithm::trim_copy( std::string( get_elem_text( root, "MetaballFile" ) ) );
    config._density = std::stof( get_elem_text( root, "Density" ) );
    config._sample_dx = std::stof( get_elem_text( root, "Sampledx" ) );
    config._nb_lloyd = std::stoi( get_elem_text( root, "NumberLloyd" ) );
    config._k_attach = std::stof( get_elem_text( root, "K_Attach" ) );
    config._k_stiff = std::stof( get_elem_text( root, "K_Stiff" ) );
    config._k_edge_stiff = config._k_stiff;
    config._nb_points = std::stoi( get_elem_text( root, "NumberPoints" ) );
    config._dt = std::stof( get_elem_text( root, "dt" ) );
    config._nb_solve = std::stoi( get_elem_text( root, "NumberSolve" ) );
    config._const_type = std::stoi( get_elem_text( root, "ConstraintType" ) );
    config._attach_filter = []( glm::vec3 v )->bool { return  v[1] > 0.3f && v[0] > 0.3f; };

    std::stringstream ss( get_elem_text( root, "Displacement" ) );
    ss >> config._displacement.x >> config._displacement.y >> config._displacement.z;

    auto surface = LoadPDMetaballHalfEdgeMesh( root->FirstChildElement( "FineSurface" ) );

    auto model = Scene::active->AddChild( std::make_unique<PD::PDMetaballModel>( config, surface ) );
    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name = get_elem_text( root, "Name" );
        model->mName = name;
    }

    return model;
}

PD::PDMetaballModelFC* ElasticityApp::LoadPDMetaballModelFC( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    PD::PDMetaballModelConfig config;
    config._method = std::stoi( get_elem_text( root, "MetaballGenMethod" ) );
    config._coarse_surface = boost::algorithm::trim_copy( std::string( get_elem_text( root, "Surface" ) ) );
    if (config._method == 2)
        config._metaball_path = boost::algorithm::trim_copy( std::string( get_elem_text( root, "MetaballFile" ) ) );
    config._density = std::stof( get_elem_text( root, "Density" ) );
    config._sample_dx = std::stof( get_elem_text( root, "Sampledx" ) );
    config._nb_lloyd = std::stoi( get_elem_text( root, "NumberLloyd" ) );
    config._k_attach = std::stof( get_elem_text( root, "K_Attach" ) );
    config._k_stiff = std::stof( get_elem_text( root, "K_Stiff" ) );
    config._k_edge_stiff = config._k_stiff;
    config._nb_points = std::stoi( get_elem_text( root, "NumberPoints" ) );
    config._dt = std::stof( get_elem_text( root, "dt" ) );
    config._nb_solve = std::stoi( get_elem_text( root, "NumberSolve" ) );
    config._const_type = std::stoi( get_elem_text( root, "ConstraintType" ) );
    //config._attach_filter = []( glm::vec3 v )->bool { return  v[0] < 0.3f && v[0] > -0.3f; };

    std::stringstream ss( get_elem_text( root, "Displacement" ) );
    ss >> config._displacement.x >> config._displacement.y >> config._displacement.z;

    auto surface = LoadPDMetaballHalfEdgeMesh( root->FirstChildElement( "FineSurface" ) );

    auto model = Scene::active->AddChild( std::make_unique<PD::PDMetaballModelFC>( config, surface ) );
    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name = get_elem_text( root, "Name" );
        model->mName = name;
    }
    return  model;
}

HalfEdgeMesh* ElasticityApp::LoadMesh( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    std::string path( get_elem_text( root, "Path" ) );
    boost::algorithm::trim( path );
    auto mesh = Scene::active->AddChild( std::make_unique<HalfEdgeMesh>( path ) );

    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name( get_elem_text( root, "Name" ) );
        mesh->mName = name;
    }

    if (root->FirstChildElement( "Layer" ) != nullptr)
    {
        mesh->mLayer = std::stoi( root->FirstChildElement( "Layer" )->GetText() );
    }

    if (root->FirstChildElement( "Transform" ) != nullptr)
    {
        mesh->mTransform = LoadTransform( root->FirstChildElement( "Transform" ) );
    }
    return mesh;
}

PDMetaballHalfEdgeMesh* ElasticityApp::LoadPDMetaballHalfEdgeMesh( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    std::string path( get_elem_text( root, "Path" ) );
    auto mesh = Scene::active->AddChild( std::make_unique<PDMetaballHalfEdgeMesh>( path ) );

    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name( get_elem_text( root, "Name" ) );
        mesh->mName = name;
    }
    if (root->FirstChildElement( "Layer" ) != nullptr)
    {
        mesh->mLayer = std::stoi( root->FirstChildElement( "Layer" )->GetText() );
    }
    return mesh;
}

Transform ElasticityApp::LoadTransform( tinyxml2::XMLElement* root )
{
    Transform t;
    if (root->FirstChildElement( "Position" ) != nullptr)
    {
        std::stringstream ss( root->FirstChildElement( "Position" )->GetText() );
        glm::vec3 p;
        ss >> p.x >> p.y >> p.z;
        t.SetPos( p );
    }
    if (root->FirstChildElement( "Scale" ) != nullptr)
    {
        std::stringstream ss( root->FirstChildElement( "Scale" )->GetText() );
        glm::vec3 s;
        ss >> s.x >> s.y >> s.z;
        t.SetScale( s );
    }
    if (root->FirstChildElement( "Rotation" ) != nullptr)
    {
        std::string s( root->FirstChildElement( "Rotation" )->GetText() );
        std::vector<std::string> result;
        boost::algorithm::split( result, s, []( char c ) {  return c == ' '; } );
        if (result.size() == 3)
        {
            glm::vec3 euler;
            euler.x = std::stof( result[0] );
            euler.y = std::stof( result[1] );
            euler.z = std::stof( result[2] );
            t.SetRotation( euler );
        }
        else if (result.size() == 4)
        {
            glm::quat q;
            q.w = std::stof( result[0] );
            q.x = std::stof( result[1] );
            q.y = std::stof( result[2] );
            q.z = std::stof( result[3] );
            t.SetRotation( q );
        }
        else if (result.size() == 9)
        {
            glm::mat3 rot;
            rot[0][0] = std::stof( result[0] );
            rot[1][0] = std::stof( result[1] );
            rot[2][0] = std::stof( result[2] );
            rot[0][1] = std::stof( result[3] );
            rot[1][1] = std::stof( result[4] );
            rot[2][1] = std::stof( result[5] );
            rot[0][2] = std::stof( result[6] );
            rot[1][2] = std::stof( result[7] );
            rot[2][2] = std::stof( result[8] );
            t.SetRotation( glm::toQuat( rot ) );
        }
    }
    return t;
}

RigidBall* ElasticityApp::LoadRigidBall( tinyxml2::XMLElement* root )
{
    std::stringstream ss( root->FirstChildElement( "Position" )->GetText() );
    glm::vec3 pos;
    ss >> pos.x >> pos.y >> pos.z;

    float r = std::stof( root->FirstChildElement( "Radius" )->GetText() );


    auto rigidball = Scene::active->AddChild( std::make_unique<RigidBall>( pos, r ) );
    if (root->FirstChildElement( "Name" ))
    {
        rigidball->mName = std::string( root->FirstChildElement( "Name" )->GetText() );
    }

    return rigidball;
}

RigidStatic* ElasticityApp::LoadRigidStatic( tinyxml2::XMLElement* root )
{
    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    std::string path( get_elem_text( root, "Path" ) );
    boost::algorithm::trim( path );
    auto rigid_static = Scene::active->AddChild( std::make_unique<RigidStatic>( path ) );

    if (root->FirstChildElement( "Name" ) != nullptr)
    {
        std::string name( get_elem_text( root, "Name" ) );
        rigid_static->mName = name;
    }

    if (root->FirstChildElement( "Transform" ) != nullptr)
    {
        rigid_static->mTransform = LoadTransform( root->FirstChildElement( "Transform" ) );
    }

    return rigid_static;
}

void ElasticityApp::LoadSceneFile( const char* filename )
{
    tinyxml2::XMLDocument doc{};
    doc.LoadFile( filename );

    auto get_elem_text = []( const tinyxml2::XMLElement* parent, const char* name ) {
        return parent->FirstChildElement( name )->GetText();
    };

    auto root = doc.FirstChildElement();

    if (std::strcmp( root->Name(), "Scene" ) == 0)
    {
        auto child = root->FirstChildElement();
        while (child)
        {
            if (std::strcmp( child->Name(), "PDGPUMetaballModel" ) == 0)
            {
                LoadPDGPUMetaballModel( child );
            }
            else if (std::strcmp( child->Name(), "PDMetaballModel" ) == 0)
            {
                LoadPDMetaballModel( child );
            }
            else if (std::strcmp( child->Name(), "PDMetaballModelFC" ) == 0)
            {
                LoadPDMetaballModelFC( child );
            }
            else if (std::strcmp( child->Name(), "Mesh" ) == 0)
            {
                LoadMesh( child );
            }
            else if (std::strcmp( child->Name(), "RigidBall" ) == 0)
            {
                LoadRigidBall( child );
            }
            else if (std::strcmp( child->Name(), "RigidStatic" ) == 0)
            {
                LoadRigidStatic( child );
            }
            child = child->NextSiblingElement();
        }
    }
}