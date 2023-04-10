#pragma once
#include <array>
#include <memory>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

class InputBase;
class InputGLFW;
class InputQt;
class OpenglGladWindow;

class Input
{
public:
    enum class Key
    {
        UNKNOWN,
        SPACE,
        APOSTROPHE,  /* ' */
        COMMA,  /* , */
        MINUS,  /* - */
        PERIOD,  /* . */
        SLASH,  /* / */
        NUM_0, NUM_1, NUM_2, NUM_3, NUM_4, NUM_5, NUM_6, NUM_7, NUM_8, NUM_9,
        SEMICOLON,  /* ; */
        EQUAL,  /* = */
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        LEFT_BRACKET,  /* [ */
        BACKSLASH,  /* \ */
        RIGHT_BRACKET,  /* ] */
        GRAVE_ACCENT,  /* ` */
        ESCAPE,
        ENTER,
        TAB,
        BACKSPACE,
        INSERT,
        DEL,
        RIGHT,
        LEFT,
        DOWN,
        UP,
        PAGE_UP,
        PAGE_DOWN,
        HOME,
        END,
        CAPS_LOCK,
        SCROLL_LOCK,
        NUM_LOCK,
        PRINT_SCREEN,
        PAUSE,
        F1,
        F2,
        F3,
        F4,
        F5,
        F6,
        F7,
        F8,
        F9,
        F10,
        F11,
        F12,
        F13,
        F14,
        F15,
        F16,
        F17,
        F18,
        F19,
        F20,
        F21,
        F22,
        F23,
        F24,
        F25,
        KP_0,  /*KP = Keypad*/
        KP_1,
        KP_2,
        KP_3,
        KP_4,
        KP_5,
        KP_6,
        KP_7,
        KP_8,
        KP_9,
        KP_DECIMAL,
        KP_DIVIDE,
        KP_MULTIPLY,
        KP_SUBTRACT,
        KP_ADD,
        KP_ENTER,
        KP_EQUAL,
        LEFT_SHIFT,
        LEFT_CONTROL,
        LEFT_ALT,
        LEFT_SUPER,
        RIGHT_SHIFT,
        RIGHT_CONTROL,
        RIGHT_ALT,
        RIGHT_SUPER,
        MENU,
        KEY_COUNT    /*indicates num of keys*/
    };

    enum class MouseButton
    {
        Left,
        Right,
        Middle,
        COUNT
    };

    enum class Status
    {
        Down,   //button is pressed down in this frame
        Held,   //button was pressed down and being held in this frame
        Up,     //button is released in this frame
        Release //button is not pressed
    };

    static void CreateGLFWInput();
    static void CreateGLFWInput( GLFWwindow* glfwWindow );
#ifdef QT_VERSION
    static void CreateQtInput( OpenglGladWindow* qtwindow );
#endif
    static void Update();

    static bool IsKeyHeld( Input::Key key );
    static bool IsKeyDown( Input::Key key );
    static bool IsKeyUp( Input::Key key );
    static bool IsKeyReleased( Input::Key key );
    static bool IsMouseButtonHeld( Input::MouseButton button );
    static bool IsMouseButtonDown( Input::MouseButton button );
    static bool IsMouseButtonUp( Input::MouseButton button );
    static bool IsMouseButtonReleased( Input::MouseButton button );
    static glm::vec2 GetMousePosition();
    static float GetMouseScrollDelta();
    static glm::vec2 GetMousePosDelta();
private:
    friend InputGLFW;
    friend InputQt;
    static std::unique_ptr<InputBase> mInput;
};

class InputBase
{
public:
    bool IsKeyHeld( Input::Key key ) const;
    bool IsKeyDown( Input::Key key ) const;
    bool IsKeyUp( Input::Key key ) const;
    bool IsKeyReleased( Input::Key key ) const;
    bool IsMouseButtonHeld( Input::MouseButton button ) const;
    bool IsMouseButtonDown( Input::MouseButton button ) const;
    bool IsMouseButtonUp( Input::MouseButton button ) const;
    bool IsMouseButtonReleased( Input::MouseButton button ) const;
    glm::vec2 GetMousePosition() const;
    glm::vec2 GetMousePosDelta() const;
    float GetMouseScrollDelta() const;
    virtual void Update();

protected:
    virtual void UpdateKeys() = 0;
    virtual void UpdateMouseButton() = 0;
    virtual void UpdateMousePos() = 0;
    virtual void UpdateMouseScroll() = 0;

private:
    void ClearStatus();

protected:
    friend class InputGLFW;
    friend class InputQt;
    std::array<Input::Status, static_cast<unsigned>(Input::Key::KEY_COUNT)>     mKeyStatus;
    std::array<Input::Status, static_cast<unsigned>(Input::MouseButton::COUNT)> mMouseButtonStatus;
    glm::vec2 mMousePos;
    glm::vec2 mMousePosDelta;
    glm::vec2 mMouseScrollDelta;
};

