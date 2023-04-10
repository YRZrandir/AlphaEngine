#pragma once
#include <iostream>
#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

class ApplicationBase
{
public:
    ApplicationBase( std::string title, size_t width, size_t height );
    virtual ~ApplicationBase();
    virtual void Start();
    virtual void Resize( size_t width, size_t height );

protected:
    virtual void Init();
    virtual void DrawGUI();
    virtual void DrawGraphics();
    virtual void PreDraw();
    virtual void PostDraw();

    virtual void InitWindow();
    virtual void DeleteWindow();
    virtual void OneFrame();
    virtual void MainLoop();
    virtual void OpenGLInit();
    virtual void OpenGLTerminate();
    virtual void ImguiInit();
    virtual void ImguiTerminate();

protected:
    std::string _title;
    size_t _width;
    size_t _height;
    GLFWwindow* _window{ nullptr };

};