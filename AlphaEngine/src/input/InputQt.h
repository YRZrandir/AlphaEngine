#pragma once
#include "Input.h"
#include <unordered_map>
#include <QObject>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

class OpenglGladWindow;

class InputQt :
    public InputBase
{
public:
    using QtKey = int;
    using QtMouseButton = int;
    using QtAction = int;

private:
    static const std::unordered_map<QtKey, Input::Key> kKeyMap;

    static Input::Key GetKey( const QKeyEvent* key );
    static Input::MouseButton GetMouseButton( QtMouseButton button );
    static void KeyCallback( const QKeyEvent* e );
    static void MouseEventCallback( const QMouseEvent* e );
    static void MouseScrollCallback( const QWheelEvent* e );

public:
    InputQt( OpenglGladWindow* window );
private:
    QObject* _window;
    // 通过 InputBase 继承
    virtual void Update() override;
    virtual void UpdateKeys() override;
    virtual void UpdateMouseButton() override;
    virtual void UpdateMousePos() override;
    virtual void UpdateMouseScroll() override;
};

