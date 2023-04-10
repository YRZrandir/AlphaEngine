#include "InputQt.h"
#include <iostream>
#include <Qt>
#include <QDebug>
#include "../OpenglGladWindow.h"

const std::unordered_map<InputQt::QtKey, Input::Key> InputQt::kKeyMap =
{
        { Qt::Key_unknown,      Input::Key::UNKNOWN       },
        { Qt::Key_Space,        Input::Key::SPACE         },
        { Qt::Key_Apostrophe,   Input::Key::APOSTROPHE    },
        { Qt::Key_Comma,        Input::Key::COMMA         },
        { Qt::Key_Minus,        Input::Key::MINUS         },
        { Qt::Key_Period,       Input::Key::PERIOD        },
        { Qt::Key_Slash,        Input::Key::SLASH         },
        { Qt::Key_0,            Input::Key::NUM_0         },
        { Qt::Key_1,            Input::Key::NUM_1         },
        { Qt::Key_2,            Input::Key::NUM_2         },
        { Qt::Key_3,            Input::Key::NUM_3         },
        { Qt::Key_4,            Input::Key::NUM_4         },
        { Qt::Key_5,            Input::Key::NUM_5         },
        { Qt::Key_6,            Input::Key::NUM_6         },
        { Qt::Key_7,            Input::Key::NUM_7         },
        { Qt::Key_8,            Input::Key::NUM_8         },
        { Qt::Key_9,            Input::Key::NUM_9         },
        { Qt::Key_Semicolon,    Input::Key::SEMICOLON     },
        { Qt::Key_Equal,        Input::Key::EQUAL         },
        { Qt::Key_A,            Input::Key::A             },
        { Qt::Key_B,            Input::Key::B             },
        { Qt::Key_C,            Input::Key::C             },
        { Qt::Key_D,            Input::Key::D             },
        { Qt::Key_E,            Input::Key::E             },
        { Qt::Key_F,            Input::Key::F             },
        { Qt::Key_G,            Input::Key::G             },
        { Qt::Key_H,            Input::Key::H             },
        { Qt::Key_I,            Input::Key::I             },
        { Qt::Key_J,            Input::Key::J             },
        { Qt::Key_K,            Input::Key::K             },
        { Qt::Key_L,            Input::Key::L             },
        { Qt::Key_M,            Input::Key::M             },
        { Qt::Key_N,            Input::Key::N             },
        { Qt::Key_O,            Input::Key::O             },
        { Qt::Key_P,            Input::Key::P             },
        { Qt::Key_Q,            Input::Key::Q             },
        { Qt::Key_R,            Input::Key::R             },
        { Qt::Key_S,            Input::Key::S             },
        { Qt::Key_T,            Input::Key::T             },
        { Qt::Key_U,            Input::Key::U             },
        { Qt::Key_V,            Input::Key::V             },
        { Qt::Key_W,            Input::Key::W             },
        { Qt::Key_X,            Input::Key::X             },
        { Qt::Key_Y,            Input::Key::Y             },
        { Qt::Key_Z,            Input::Key::Z             },
        { Qt::Key_BracketLeft,  Input::Key::LEFT_BRACKET  },
        { Qt::Key_Backslash,    Input::Key::BACKSLASH     },
        { Qt::Key_BracketRight, Input::Key::RIGHT_BRACKET },
        { Qt::Key_QuoteLeft,    Input::Key::GRAVE_ACCENT  },
        { Qt::Key_Escape,       Input::Key::ESCAPE        },
        { Qt::Key_Enter,        Input::Key::ENTER         },
        { Qt::Key_Tab,          Input::Key::TAB           },
        { Qt::Key_Backspace,    Input::Key::BACKSPACE     },
        { Qt::Key_Insert,       Input::Key::INSERT        },
        { Qt::Key_Delete,       Input::Key::DEL           },
        { Qt::Key_Right,        Input::Key::RIGHT         },
        { Qt::Key_Left,         Input::Key::LEFT          },
        { Qt::Key_Down,         Input::Key::DOWN          },
        { Qt::Key_Up,           Input::Key::UP            },
        { Qt::Key_PageUp,       Input::Key::PAGE_UP       },
        { Qt::Key_PageDown,     Input::Key::PAGE_DOWN     },
        { Qt::Key_Home,         Input::Key::HOME          },
        { Qt::Key_End,          Input::Key::END           },
        { Qt::Key_CapsLock,     Input::Key::CAPS_LOCK     },
        { Qt::Key_ScrollLock,   Input::Key::SCROLL_LOCK   },
        { Qt::Key_NumLock,      Input::Key::NUM_LOCK      },
        { Qt::Key_Print,        Input::Key::PRINT_SCREEN  },
        { Qt::Key_Pause ,       Input::Key::PAUSE         },
        { Qt::Key_F1,           Input::Key::F1            },
        { Qt::Key_F2,           Input::Key::F2            },
        { Qt::Key_F3,           Input::Key::F3            },
        { Qt::Key_F4,           Input::Key::F4            },
        { Qt::Key_F5,           Input::Key::F5            },
        { Qt::Key_F6,           Input::Key::F6            },
        { Qt::Key_F7,           Input::Key::F7            },
        { Qt::Key_F8,           Input::Key::F8            },
        { Qt::Key_F9,           Input::Key::F9            },
        { Qt::Key_F10,          Input::Key::F10           },
        { Qt::Key_F11,          Input::Key::F11           },
        { Qt::Key_F12,          Input::Key::F12           },
        { Qt::Key_F13,          Input::Key::F13           },
        { Qt::Key_F14,          Input::Key::F14           },
        { Qt::Key_F15,          Input::Key::F15           },
        { Qt::Key_F16,          Input::Key::F16           },
        { Qt::Key_F17,          Input::Key::F17           },
        { Qt::Key_F18,          Input::Key::F18           },
        { Qt::Key_F19,          Input::Key::F19           },
        { Qt::Key_F20,          Input::Key::F20           },
        { Qt::Key_F21,          Input::Key::F21           },
        { Qt::Key_F22,          Input::Key::F22           },
        { Qt::Key_F23,          Input::Key::F23           },
        { Qt::Key_F24,          Input::Key::F24           },
        { Qt::Key_F25,          Input::Key::F25           },
        { Qt::Key_Shift,        Input::Key::LEFT_SHIFT    },
        { Qt::Key_Alt,          Input::Key::LEFT_ALT      },
        { Qt::Key_Control,      Input::Key::LEFT_CONTROL  }
};

Input::Key InputQt::GetKey( const QKeyEvent* e )
{
    if (kKeyMap.find( e->key() ) != kKeyMap.end())
    {
        return kKeyMap.at( e->key() );
    }
    return Input::Key::UNKNOWN;
}

Input::MouseButton InputQt::GetMouseButton( QtMouseButton button )
{
    switch (button)
    {
    case Qt::LeftButton:
        return Input::MouseButton::Left;
        break;
    case Qt::RightButton:
        return Input::MouseButton::Right;
        break;
    case Qt::MiddleButton:
        return Input::MouseButton::Middle;
        break;
    default:
        //TODO: exception here
        break;
    }
}

void InputQt::KeyCallback( const QKeyEvent* e )
{
    Input::Key key = GetKey( e );
    unsigned int index = static_cast<unsigned>(key);
    switch (e->type())
    {
    case QEvent::Type::KeyPress:
        if (Input::IsKeyHeld( key ) || Input::IsKeyDown( key ))
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Held;
        }
        else
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Down;
        }
        break;
    case QEvent::Type::KeyRelease:
        if (e->isAutoRepeat())
        {
            if (Input::IsKeyHeld( key ) || Input::IsKeyDown( key ))
            {
                Input::mInput->mKeyStatus[index] = Input::Status::Held;
            }
            else
            {
                Input::mInput->mKeyStatus[index] = Input::Status::Down;
            }
        }
        else
        {
            if (Input::IsKeyHeld( key ) || Input::IsKeyDown( key ))
            {
                Input::mInput->mKeyStatus[index] = Input::Status::Up;
            }
            else
            {
                Input::mInput->mKeyStatus[index] = Input::Status::Release;
            }
        }
        break;
    default:
        //TODO: exception here
        break;
    }
}

void InputQt::MouseEventCallback( const QMouseEvent* e )
{
    switch (e->type())
    {
    case QEvent::Type::MouseMove:
    {
        glm::vec2 newpos( e->pos().x(), e->pos().y() );
        Input::mInput->mMousePosDelta = newpos - Input::mInput->mMousePos;
        Input::mInput->mMousePos = newpos;
        break;
    }
    case QEvent::Type::MouseButtonPress:
    {
        Input::MouseButton button = GetMouseButton( e->button() );
        if (Input::IsMouseButtonDown( button ) || Input::IsMouseButtonHeld( button ))
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Held;
        }
        else
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Down;
        }
        break;
    }
    case QEvent::Type::MouseButtonRelease:
    {
        Input::MouseButton button = GetMouseButton( e->button() );
        if (Input::IsMouseButtonDown( button ) || Input::IsMouseButtonHeld( button ))
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Up;
        }
        else
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Release;
        }
        break;
    }
    }
}

void InputQt::MouseScrollCallback( const QWheelEvent* e )
{
    Input::mInput->mMouseScrollDelta = glm::vec2( e->angleDelta().x(), e->angleDelta().y() );
}

InputQt::InputQt( OpenglGladWindow* window )
{
    window->SetCallbacks( InputQt::KeyCallback, InputQt::MouseEventCallback, InputQt::MouseScrollCallback );
    mKeyStatus.fill( Input::Status::Release );
    mMouseButtonStatus.fill( Input::Status::Release );
}

void InputQt::Update()
{
    UpdateKeys();
    UpdateMouseButton();
    UpdateMousePos();
    UpdateMouseScroll();
}

void InputQt::UpdateKeys()
{
    for (auto& status : mKeyStatus)
    {
        if (status == Input::Status::Down)
        {
            status = Input::Status::Held;
        }
        else if (status == Input::Status::Up)
        {
            status = Input::Status::Release;
        }
    }
}

void InputQt::UpdateMouseButton()
{
    for (auto& status : mMouseButtonStatus)
    {
        if (status == Input::Status::Down)
        {
            status = Input::Status::Held;
        }
        else if (status == Input::Status::Up)
        {
            status = Input::Status::Release;
        }
    }
}

void InputQt::UpdateMousePos()
{
    mMousePosDelta = glm::vec2( 0.f );
}

void InputQt::UpdateMouseScroll()
{
    mMouseScrollDelta = glm::vec2( 0.f, 0.f );
}
