#include "Input.h"
#include "InputGLFW.h"
#ifdef QT_VERSION
#include "InputQt.h"
#endif

std::unique_ptr<InputBase> Input::mInput = nullptr;

void Input::CreateGLFWInput()
{
    mInput = std::make_unique<InputGLFW>();
}

void Input::CreateGLFWInput( GLFWwindow* glfwWindow )
{
    mInput = std::make_unique<InputGLFW>( glfwWindow );
}

#ifdef QT_VERSION
void Input::CreateQtInput( OpenglGladWindow* qtwindow )
{
    mInput = std::make_unique<InputQt>( qtwindow );
}
#endif

void Input::Update()
{
    mInput->Update();
}

bool Input::IsKeyHeld( Input::Key key )
{
    return mInput->IsKeyHeld( key );
}

bool Input::IsKeyDown( Input::Key key )
{
    return mInput->IsKeyDown( key );
}

bool Input::IsKeyUp( Input::Key key )
{
    return mInput->IsKeyUp( key );
}

bool Input::IsKeyReleased( Input::Key key )
{
    return mInput->IsKeyReleased( key );
}

bool Input::IsMouseButtonHeld( Input::MouseButton button )
{
    return mInput->IsMouseButtonHeld( button );
}

bool Input::IsMouseButtonDown( Input::MouseButton button )
{
    return mInput->IsMouseButtonDown( button );
}

bool Input::IsMouseButtonUp( Input::MouseButton button )
{
    return mInput->IsMouseButtonUp( button );
}

bool Input::IsMouseButtonReleased( Input::MouseButton button )
{
    return mInput->IsMouseButtonReleased( button );
}

glm::vec2 Input::GetMousePosition()
{
    return mInput->GetMousePosition();
}

float Input::GetMouseScrollDelta()
{
    return mInput->GetMouseScrollDelta();
}

glm::vec2 Input::GetMousePosDelta()
{
    return mInput->GetMousePosDelta();
}


bool InputBase::IsKeyHeld( Input::Key key ) const
{
    return mKeyStatus[static_cast<unsigned>(key)] == Input::Status::Held;
}

bool InputBase::IsKeyDown( Input::Key key ) const
{
    return mKeyStatus[static_cast<unsigned>(key)] == Input::Status::Down;
}

bool InputBase::IsKeyUp( Input::Key key ) const
{
    return mKeyStatus[static_cast<unsigned>(key)] == Input::Status::Up;
}

bool InputBase::IsKeyReleased( Input::Key key ) const
{
    return mKeyStatus[static_cast<unsigned>(key)] == Input::Status::Release;
}

bool InputBase::IsMouseButtonHeld( Input::MouseButton button ) const
{
    return mMouseButtonStatus[static_cast<unsigned>(button)] == Input::Status::Held;
}

bool InputBase::IsMouseButtonDown( Input::MouseButton button ) const
{
    return mMouseButtonStatus[static_cast<unsigned>(button)] == Input::Status::Down;
}

bool InputBase::IsMouseButtonUp( Input::MouseButton button ) const
{
    return mMouseButtonStatus[static_cast<unsigned>(button)] == Input::Status::Up;
}

bool InputBase::IsMouseButtonReleased( Input::MouseButton button ) const
{
    return mMouseButtonStatus[static_cast<unsigned>(button)] == Input::Status::Release;
}

glm::vec2 InputBase::GetMousePosition() const
{
    return mMousePos;
}

glm::vec2 InputBase::GetMousePosDelta() const
{
    return mMousePosDelta;
}

float InputBase::GetMouseScrollDelta() const
{
    return mMouseScrollDelta.y;
}

void InputBase::Update()
{
    UpdateKeys();
    UpdateMouseButton();
    UpdateMousePos();
    UpdateMouseScroll();
}

void InputBase::ClearStatus()
{
}
