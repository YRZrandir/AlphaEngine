#pragma once
#include "Input.h"
#include <array>
#include <unordered_map>
#include <GLFW/glfw3.h>

class InputGLFW :
    public InputBase
{
/*
**STATIC MEMBERS
*/
public:
    using GlfwKey = int;
    using GlfwMouseButton = int;
    using GlfwAction = int;

private:
    static const std::unordered_map<InputGLFW::GlfwKey, Input::Key> kKeyMap;
    static Input::Key GetKey(InputGLFW::GlfwKey key);
    static Input::MouseButton GetMouseButton(InputGLFW::GlfwMouseButton button);

    static void KeyCallback(GLFWwindow* window, GlfwKey glfwKey, int scancode, GlfwAction action, int modbits);
    static void MousePosCallback(GLFWwindow* window, double x, double y);
    static void MouseButtonCallback(GLFWwindow* window, GlfwMouseButton button, GlfwAction action, int modbits);
    static void MouseScrollCallback(GLFWwindow* window, double offsetX, double offsetY);
/*
**NON-STATIC MEMBERS
*/
public:
    InputGLFW();
    InputGLFW(GLFWwindow* pGlfwWindow);
private:
    GLFWwindow* mWindow;

    // Í¨¹ý InputBase ¼Ì³Ð
    virtual void Update() override;
    virtual void UpdateKeys() override;
    virtual void UpdateMouseButton() override;
    virtual void UpdateMousePos() override;
    virtual void UpdateMouseScroll() override;
};

