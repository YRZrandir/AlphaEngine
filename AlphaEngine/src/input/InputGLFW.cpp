#include "InputGLFW.h"
#include <iostream>

const std::unordered_map<InputGLFW::GlfwKey, Input::Key> InputGLFW::kKeyMap = 
{
    { GLFW_KEY_UNKNOWN        , Input::Key::UNKNOWN      },
    { GLFW_KEY_SPACE          , Input::Key::SPACE        },
    { GLFW_KEY_APOSTROPHE     , Input::Key::APOSTROPHE   },
    { GLFW_KEY_COMMA          , Input::Key::COMMA        },
    { GLFW_KEY_MINUS          , Input::Key::MINUS        },
    { GLFW_KEY_PERIOD         , Input::Key::PERIOD       },
    { GLFW_KEY_SLASH          , Input::Key::SLASH        },
    { GLFW_KEY_0              , Input::Key::NUM_0        },
    { GLFW_KEY_1              , Input::Key::NUM_1        },
    { GLFW_KEY_2              , Input::Key::NUM_2        },
    { GLFW_KEY_3              , Input::Key::NUM_3        },
    { GLFW_KEY_4              , Input::Key::NUM_4        },
    { GLFW_KEY_5              , Input::Key::NUM_5        },
    { GLFW_KEY_6              , Input::Key::NUM_6        },
    { GLFW_KEY_7              , Input::Key::NUM_7        },
    { GLFW_KEY_8              , Input::Key::NUM_8        },
    { GLFW_KEY_9              , Input::Key::NUM_9        },
    { GLFW_KEY_SEMICOLON      , Input::Key::SEMICOLON    },
    { GLFW_KEY_EQUAL          , Input::Key::EQUAL        },
    { GLFW_KEY_A              , Input::Key::A            },
    { GLFW_KEY_B              , Input::Key::B            },
    { GLFW_KEY_C              , Input::Key::C            },
    { GLFW_KEY_D              , Input::Key::D            },
    { GLFW_KEY_E              , Input::Key::E            },
    { GLFW_KEY_F              , Input::Key::F            },
    { GLFW_KEY_G              , Input::Key::G            },
    { GLFW_KEY_H              , Input::Key::H            },
    { GLFW_KEY_I              , Input::Key::I            },
    { GLFW_KEY_J              , Input::Key::J            },
    { GLFW_KEY_K              , Input::Key::K            },
    { GLFW_KEY_L              , Input::Key::L            },
    { GLFW_KEY_M              , Input::Key::M            },
    { GLFW_KEY_N              , Input::Key::N            },
    { GLFW_KEY_O              , Input::Key::O            },
    { GLFW_KEY_P              , Input::Key::P            },
    { GLFW_KEY_Q              , Input::Key::Q            },
    { GLFW_KEY_R              , Input::Key::R            },
    { GLFW_KEY_S              , Input::Key::S            },
    { GLFW_KEY_T              , Input::Key::T            },
    { GLFW_KEY_U              , Input::Key::U            },
    { GLFW_KEY_V              , Input::Key::V            },
    { GLFW_KEY_W              , Input::Key::W            },
    { GLFW_KEY_X              , Input::Key::X            },
    { GLFW_KEY_Y              , Input::Key::Y            },
    { GLFW_KEY_Z              , Input::Key::Z            },
    { GLFW_KEY_LEFT_BRACKET   , Input::Key::LEFT_BRACKET },
    { GLFW_KEY_BACKSLASH      , Input::Key::BACKSLASH    },
    { GLFW_KEY_RIGHT_BRACKET  , Input::Key::RIGHT_BRACKET},
    { GLFW_KEY_GRAVE_ACCENT   , Input::Key::GRAVE_ACCENT },
    { GLFW_KEY_ESCAPE         , Input::Key::ESCAPE       },
    { GLFW_KEY_ENTER          , Input::Key::ENTER        },
    { GLFW_KEY_TAB            , Input::Key::TAB          },
    { GLFW_KEY_BACKSPACE      , Input::Key::BACKSPACE    },
    { GLFW_KEY_INSERT         , Input::Key::INSERT       },
    { GLFW_KEY_DELETE         , Input::Key::DEL          },
    { GLFW_KEY_RIGHT          , Input::Key::RIGHT        },
    { GLFW_KEY_LEFT           , Input::Key::LEFT         },
    { GLFW_KEY_DOWN           , Input::Key::DOWN         },
    { GLFW_KEY_UP             , Input::Key::UP           },
    { GLFW_KEY_PAGE_UP        , Input::Key::PAGE_UP      },
    { GLFW_KEY_PAGE_DOWN      , Input::Key::PAGE_DOWN    },
    { GLFW_KEY_HOME           , Input::Key::HOME         },
    { GLFW_KEY_END            , Input::Key::END          },
    { GLFW_KEY_CAPS_LOCK      , Input::Key::CAPS_LOCK    },
    { GLFW_KEY_SCROLL_LOCK    , Input::Key::SCROLL_LOCK  },
    { GLFW_KEY_NUM_LOCK       , Input::Key::NUM_LOCK     },
    { GLFW_KEY_PRINT_SCREEN   , Input::Key::PRINT_SCREEN },
    { GLFW_KEY_PAUSE          , Input::Key::PAUSE        },
    { GLFW_KEY_F1             , Input::Key::F1           },
    { GLFW_KEY_F2             , Input::Key::F2           },
    { GLFW_KEY_F3             , Input::Key::F3           },
    { GLFW_KEY_F4             , Input::Key::F4           },
    { GLFW_KEY_F5             , Input::Key::F5           },
    { GLFW_KEY_F6             , Input::Key::F6           },
    { GLFW_KEY_F7             , Input::Key::F7           },
    { GLFW_KEY_F8             , Input::Key::F8           },
    { GLFW_KEY_F9             , Input::Key::F9           },
    { GLFW_KEY_F10            , Input::Key::F10          },
    { GLFW_KEY_F11            , Input::Key::F11          },
    { GLFW_KEY_F12            , Input::Key::F12          },
    { GLFW_KEY_F13            , Input::Key::F13          },
    { GLFW_KEY_F14            , Input::Key::F14          },
    { GLFW_KEY_F15            , Input::Key::F15          },
    { GLFW_KEY_F16            , Input::Key::F16          },
    { GLFW_KEY_F17            , Input::Key::F17          },
    { GLFW_KEY_F18            , Input::Key::F18          },
    { GLFW_KEY_F19            , Input::Key::F19          },
    { GLFW_KEY_F20            , Input::Key::F20          },
    { GLFW_KEY_F21            , Input::Key::F21          },
    { GLFW_KEY_F22            , Input::Key::F22          },
    { GLFW_KEY_F23            , Input::Key::F23          },
    { GLFW_KEY_F24            , Input::Key::F24          },
    { GLFW_KEY_F25            , Input::Key::F25          },
    { GLFW_KEY_KP_0           , Input::Key::KP_0         },
    { GLFW_KEY_KP_1           , Input::Key::KP_1         },
    { GLFW_KEY_KP_2           , Input::Key::KP_2         },
    { GLFW_KEY_KP_3           , Input::Key::KP_3         },
    { GLFW_KEY_KP_4           , Input::Key::KP_4         },
    { GLFW_KEY_KP_5           , Input::Key::KP_5         },
    { GLFW_KEY_KP_6           , Input::Key::KP_6         },
    { GLFW_KEY_KP_7           , Input::Key::KP_7         },
    { GLFW_KEY_KP_8           , Input::Key::KP_8         },
    { GLFW_KEY_KP_9           , Input::Key::KP_9         },
    { GLFW_KEY_KP_DECIMAL     , Input::Key::KP_DECIMAL   },
    { GLFW_KEY_KP_DIVIDE      , Input::Key::KP_DIVIDE    },
    { GLFW_KEY_KP_MULTIPLY    , Input::Key::KP_MULTIPLY  },
    { GLFW_KEY_KP_SUBTRACT    , Input::Key::KP_SUBTRACT  },
    { GLFW_KEY_KP_ADD         , Input::Key::KP_ADD       },
    { GLFW_KEY_KP_ENTER       , Input::Key::KP_ENTER     },
    { GLFW_KEY_KP_EQUAL       , Input::Key::KP_EQUAL     },
    { GLFW_KEY_LEFT_SHIFT     , Input::Key::LEFT_SHIFT   },
    { GLFW_KEY_LEFT_CONTROL   , Input::Key::LEFT_CONTROL },
    { GLFW_KEY_LEFT_ALT       , Input::Key::LEFT_ALT     },
    { GLFW_KEY_LEFT_SUPER     , Input::Key::LEFT_SUPER   },
    { GLFW_KEY_RIGHT_SHIFT    , Input::Key::RIGHT_SHIFT  },
    { GLFW_KEY_RIGHT_CONTROL  , Input::Key::RIGHT_CONTROL},
    { GLFW_KEY_RIGHT_ALT      , Input::Key::RIGHT_ALT    },
    { GLFW_KEY_RIGHT_SUPER    , Input::Key::RIGHT_SUPER  },
    { GLFW_KEY_MENU           , Input::Key::MENU         }
};

Input::Key InputGLFW::GetKey(InputGLFW::GlfwKey key)
{
    if (kKeyMap.find(key) != kKeyMap.end())
    {
        return kKeyMap.at(key);
    }
    return Input::Key::UNKNOWN;
}

Input::MouseButton InputGLFW::GetMouseButton(InputGLFW::GlfwMouseButton button)
{
    switch (button)
    {
    case GLFW_MOUSE_BUTTON_LEFT:
        return Input::MouseButton::Left;
        break;
    case GLFW_MOUSE_BUTTON_RIGHT:
        return Input::MouseButton::Right;
        break;
    case GLFW_MOUSE_BUTTON_MIDDLE:
        return Input::MouseButton::Middle;
        break;
    default:
        //TODO: exception here
        break;
    }
}

void InputGLFW::KeyCallback(GLFWwindow* window, GlfwKey glfwKey, int scancode, GlfwAction action, int modbits)
{
    Input::Key key = GetKey(glfwKey);
    unsigned int index = static_cast<unsigned>(key);
    switch (action)
    {
    case GLFW_REPEAT:
        break;
    case GLFW_PRESS:
        if (Input::IsKeyHeld(key) || Input::IsKeyDown(key))
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Held;
        }
        else
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Down;
        }
        break;
    case GLFW_RELEASE:
        if (Input::IsKeyHeld(key) || Input::IsKeyDown(key))
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Up;
        }
        else
        {
            Input::mInput->mKeyStatus[index] = Input::Status::Release;
        }
        break;
    default:
        //TODO: exception here
        break;
    }
}

void InputGLFW::MousePosCallback(GLFWwindow* window, double x, double y)
{
    glm::vec2 newpos(x, y);
    Input::mInput->mMousePosDelta = newpos - Input::mInput->mMousePos;
    Input::mInput->mMousePos = newpos;
}

void InputGLFW::MouseButtonCallback(GLFWwindow* window, GlfwMouseButton glfwButton, GlfwAction action, int modbits)
{
    Input::MouseButton button = GetMouseButton(glfwButton);
    switch (action)
    {
    case GLFW_PRESS:
        if (Input::IsMouseButtonDown(button) || Input::IsMouseButtonHeld(button))
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Held;
        }
        else
        {
            Input::mInput->mMouseButtonStatus[static_cast<unsigned>(button)] = Input::Status::Down;
        }       
        break;
    case GLFW_RELEASE:
        if (Input::IsMouseButtonDown(button) || Input::IsMouseButtonHeld(button))
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

void InputGLFW::MouseScrollCallback(GLFWwindow* window, double offsetX, double offsetY)
{
    Input::mInput->mMouseScrollDelta = glm::vec2(offsetX, offsetY);
}

InputGLFW::InputGLFW()
    :InputGLFW(glfwGetCurrentContext())
{
}

InputGLFW::InputGLFW(GLFWwindow* pGlfwWindow)
    :mWindow(pGlfwWindow)
{
    glfwSetKeyCallback(pGlfwWindow, InputGLFW::KeyCallback);
    glfwSetCursorPosCallback(pGlfwWindow, InputGLFW::MousePosCallback);
    glfwSetMouseButtonCallback(pGlfwWindow, InputGLFW::MouseButtonCallback);
    glfwSetScrollCallback(pGlfwWindow, InputGLFW::MouseScrollCallback);
    mKeyStatus.fill(Input::Status::Release);
    mMouseButtonStatus.fill(Input::Status::Release);
}

void InputGLFW::Update()
{
    UpdateKeys();
    UpdateMouseButton();
    UpdateMousePos();
    UpdateMouseScroll();
    glfwPollEvents();
}

void InputGLFW::UpdateKeys()
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

void InputGLFW::UpdateMouseButton()
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

void InputGLFW::UpdateMousePos()
{
    mMousePosDelta = glm::vec2(0.f);
}

void InputGLFW::UpdateMouseScroll()
{
    mMouseScrollDelta = glm::vec2(0.f, 0.f);
}
