#include "ApplicationBase.h"

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include "input/input.h"
#include "util/Instrumentor.h"

static void glfw_error_callback( int error, const char* description )
{
    fprintf( stderr, "Glfw Error %d: %s\n", error, description );
}

void APIENTRY debugproc1( GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar* message,
    const void* userParam )
{
    if (type == GL_DEBUG_TYPE_ERROR)
    {
        std::cout << "GL_ERROR : " << static_cast<const char*>(message) << '\n';
        throw std::exception();
    }
}

void FrameBufferSizeCallback( GLFWwindow* window, int width, int height )
{
    ApplicationBase* app = (ApplicationBase*)glfwGetWindowUserPointer( window );
    app->Resize( width, height );
    glViewport( 0, 0, width, height );
}

ApplicationBase::ApplicationBase( std::string title, size_t width, size_t height )
    :_title( title ), _width( width ), _height( height )
{
}

ApplicationBase::~ApplicationBase()
{
    DeleteWindow();
}

void ApplicationBase::Start()
{
    InitWindow();
    OpenGLInit();
    ImguiInit();
    Init();
    MainLoop();
}

void ApplicationBase::Init()
{
}

void ApplicationBase::DrawGUI()
{
}

void ApplicationBase::DrawGraphics()
{
}

void ApplicationBase::PreDraw()
{
}

void ApplicationBase::PostDraw()
{
}

void ApplicationBase::InitWindow()
{
    glfwSetErrorCallback( glfw_error_callback );
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW." << std::endl;
        exit( -1 );
    }

    glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
    glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 5 );
    glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE );
#ifdef _DEBUG
    glfwWindowHint( GLFW_OPENGL_DEBUG_CONTEXT, true );
#endif
    //glfwWindowHint( GLFW_SAMPLES, 4 );

    _window = glfwCreateWindow( static_cast<int>(_width), static_cast<int>(_height), _title.c_str(), nullptr, nullptr );
    if (_window == nullptr)
    {
        std::cout << "Failed to create window." << std::endl;
        exit( -1 );
    }

    glfwSetWindowUserPointer( _window, this );
    glfwMakeContextCurrent( _window );
    glfwSetFramebufferSizeCallback( _window, FrameBufferSizeCallback );
    glfwSwapInterval( 0 );
    Input::CreateGLFWInput( _window );
}

void ApplicationBase::DeleteWindow()
{
    glfwDestroyWindow( _window );
    glfwTerminate();
}

void ApplicationBase::Resize( size_t width, size_t height )
{
    _width = width;
    _height = height;
}

void ApplicationBase::OneFrame()
{
    if (glfwWindowShouldClose( _window ))
    {
        return;
    }

    glfwPollEvents();
    PreDraw();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    DrawGraphics();

    DrawGUI();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
    glfwSwapBuffers( _window );

    PostDraw();

}

void ApplicationBase::MainLoop()
{
    while (!glfwWindowShouldClose( _window ))
    {
        OneFrame();
    }
    ImguiTerminate();
    OpenGLTerminate();
}

void ApplicationBase::OpenGLInit()
{
    if (!gladLoadGLLoader( (GLADloadproc)glfwGetProcAddress ))
    {
        std::cout << "Failed to load gl loader." << std::endl;
        exit( -1 );
    }
#ifdef _DEBUG
    glEnable( GL_DEBUG_OUTPUT );
    glDebugMessageCallback( debugproc1, nullptr );
#endif
}

void ApplicationBase::OpenGLTerminate()
{
}

void ApplicationBase::ImguiInit()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL( _window, true );
    ImGui_ImplOpenGL3_Init( "#version 450" );

}

void ApplicationBase::ImguiTerminate()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}
