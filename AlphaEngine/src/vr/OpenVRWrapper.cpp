#include "OpenVRWrapper.h"

#include <iostream>
#include <exception>
#include <stdexcept>

#include "../gl/FrameBuffer.h"

using Status = OpenVRWrapper::IO::Status;

bool             OpenVRWrapper::mInit;
vr::IVRSystem* OpenVRWrapper::mVRSystem;
unsigned int     OpenVRWrapper::mRenderWidth;
unsigned int     OpenVRWrapper::mRenderHeight;
std::unique_ptr<FrameBuffer> OpenVRWrapper::mLeftFrame;
std::unique_ptr<FrameBuffer> OpenVRWrapper::mRightFrame;

vr::TrackedDevicePose_t  OpenVRWrapper::mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
OpenVRWrapper::ControllerState  OpenVRWrapper::mControllerState[2];
OpenVRWrapper::HMDState         OpenVRWrapper::mHmdState;

glm::mat4 OpenVRWrapper::mEyePosLeft;
glm::mat4 OpenVRWrapper::mEyePosRight;

glm::mat4 OpenVRWrapper::mProjectionCenter;
glm::mat4 OpenVRWrapper::mProjectionLeft;
glm::mat4 OpenVRWrapper::mProjectionRight;

glm::vec2   OpenVRWrapper::IO::mJoystickValue[2] = { glm::vec2(0.f), glm::vec2(0.f) };
Status OpenVRWrapper::IO::mButtonStatus[12] = { Status::Release };


void OpenVRWrapper::Init()
{
    //Init
    vr::EVRInitError initError = vr::VRInitError_None;
    mVRSystem = vr::VR_Init(&initError, vr::VRApplication_Scene);
    if (initError != vr::VRInitError_None)
    {
        std::cout << "VR_Init Failed" << std::endl;
        throw std::runtime_error("OpenVR Init Failed : " +
            std::string(vr::VR_GetVRInitErrorAsEnglishDescription(initError)));
    }

    //Render Targets
    mVRSystem->GetRecommendedRenderTargetSize(&mRenderWidth, &mRenderHeight);
    mRenderWidth *= 2;
    mRenderHeight *= 2;
    mLeftFrame = std::make_unique<FrameBuffer>(mRenderWidth, mRenderHeight);
    mRightFrame = std::make_unique<FrameBuffer>(mRenderWidth, mRenderHeight);

    //Init Compositor
    if (!vr::VRCompositor())
    {
        std::cout << "VRCompositor Failed" << std::endl;
        throw std::runtime_error("OpenVR Init Compositor Failed");
    }

    //Init Pose
    mEyePosLeft = ConvertSteamVRMatrixToMatrix4(mVRSystem->GetEyeToHeadTransform(vr::Eye_Left));
    mEyePosLeft = glm::inverse(mEyePosLeft);
    mEyePosRight = ConvertSteamVRMatrixToMatrix4(mVRSystem->GetEyeToHeadTransform(vr::Eye_Right));
    mEyePosRight = glm::inverse(mEyePosRight);
    mProjectionLeft = ConvertSteamVRMatrixToMatrix4(mVRSystem->GetProjectionMatrix(vr::Eye_Left, 0.001, 100));
    mProjectionRight = ConvertSteamVRMatrixToMatrix4(mVRSystem->GetProjectionMatrix(vr::Eye_Right, 0.001, 100));

    int controllerNum = 0;
    for (unsigned int i = 0; i < vr::k_unMaxTrackedDeviceCount;i++)
    {
        if (mVRSystem->GetTrackedDeviceClass(i) == vr::ETrackedDeviceClass::TrackedDeviceClass_Controller)
        {
            ControllerState* state = nullptr;
            controllerNum++;
            vr::ETrackedControllerRole role =
                mVRSystem->GetControllerRoleForTrackedDeviceIndex(i);
            if (role == vr::TrackedControllerRole_LeftHand)
            {
                state = &mControllerState[0];
            }
            else if (role == vr::TrackedControllerRole_RightHand)
            {
                state = &mControllerState[1];
            }

            if (state)
            {
                state->deviceid = i;
                for (int x = 0; x < vr::k_unControllerStateAxisCount; x++)
                {
                    int prop = mVRSystem->GetInt32TrackedDeviceProperty(state->deviceid,
                        (vr::ETrackedDeviceProperty)(vr::Prop_Axis0Type_Int32 + x));
                    if (prop == vr::k_eControllerAxis_Trigger)
                        state->idtrigger = x;
                    else if (prop == vr::k_eControllerAxis_TrackPad)
                        state->idpad = x;
                    else if (prop == vr::k_eControllerAxis_Joystick)
                        state->idjoystick = x;
                }
            }
        }
        if (mVRSystem->GetTrackedDeviceClass(i) == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD)
        {
            mHmdState.deviceid = i;
        }
    }

    mInit = true;
}

bool OpenVRWrapper::UseVr()
{
    return mInit;
}

void OpenVRWrapper::BindLeftEye()
{
    mLeftFrame->Bind();
    mLeftFrame->Clear();
}

void OpenVRWrapper::BindRightEye()
{
    mRightFrame->Bind();
    mRightFrame->Clear();
}

unsigned int OpenVRWrapper::GetRenderWidth()
{
    return mRenderWidth;
}

unsigned int OpenVRWrapper::GetRenderHeight()
{
    return mRenderHeight;
}

glm::mat4 OpenVRWrapper::GetLeftEyePose()
{
    return mEyePosLeft;
}

glm::mat4 OpenVRWrapper::GetRightEyePose()
{
    return mEyePosRight;
}

glm::vec3 OpenVRWrapper::GetHMDPose()
{
    return mHmdState.pos;
}

glm::quat OpenVRWrapper::GetHMDRotate()
{
    return mHmdState.rotate;
}

glm::mat4 OpenVRWrapper::GetHMDMat()
{
    return mHmdState.mat;
}

glm::mat4 OpenVRWrapper::GetLeftProjection()
{
    return mProjectionLeft;
}

glm::mat4 OpenVRWrapper::GetRightProjection()
{
    return mProjectionRight;
}

glm::vec3 OpenVRWrapper::GetRightHandPos()
{
    return mControllerState[1].pos;
}

glm::quat OpenVRWrapper::GetRightHandRotate()
{
    return mControllerState[1].rotate;
}

glm::mat4 OpenVRWrapper::GetRightHandMat()
{
    return mControllerState[1].mat;
}

glm::vec3 OpenVRWrapper::GetLeftHandPos()
{
    return mControllerState[0].pos;
}

glm::quat OpenVRWrapper::GetLeftHandRotate()
{
    return mControllerState[0].rotate;
}

glm::mat4 OpenVRWrapper::GetLeftHandMat()
{
    return mControllerState[0].mat;
}

void OpenVRWrapper::Submit()
{
    vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)mLeftFrame->GetRenderTargetID(),
        vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
    vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)mRightFrame->GetRenderTargetID(),
        vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
}

void OpenVRWrapper::HandleInput()
{
    IO::ClearStatus();
    vr::VREvent_t e;
    while (mVRSystem->PollNextEvent(&e, sizeof(e)))
    {
        ProcessEvent(e);
    }
}

void OpenVRWrapper::UpdateMatrix()
{
    vr::VRCompositor()->WaitGetPoses(mTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
    {
        if (mTrackedDevicePose[nDevice].bPoseIsValid)
        {
            if (mVRSystem->GetTrackedDeviceClass(nDevice) == vr::TrackedDeviceClass_Controller)
            {
                vr::ETrackedControllerRole role =
                    mVRSystem->GetControllerRoleForTrackedDeviceIndex(nDevice);
                if (role == vr::TrackedControllerRole_LeftHand)
                {
                     mControllerState[0].mat = 
                        ConvertSteamVRMatrixToMatrix4(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                     mControllerState[0].pos = CalcPosition(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                     mControllerState[0].rotate = CalcRotation(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                }
                else if (role == vr::TrackedControllerRole_RightHand)
                {
                    mControllerState[1].mat = 
                        ConvertSteamVRMatrixToMatrix4(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                    mControllerState[1].pos = CalcPosition(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                    mControllerState[1].rotate = CalcRotation(mTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
                }
            }
        }
    }

    if (mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
    {
        mHmdState.pos = CalcPosition(mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
        mHmdState.rotate = CalcRotation(mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
        mHmdState.mat = glm::inverse(ConvertSteamVRMatrixToMatrix4(mTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking));
    }

    vr::VRControllerState_t controllerState;
    for (int i = 0; i < 2; i++)
    {
        ControllerState* state = &(mControllerState[i]);
        if (state->deviceid < 0 || !mVRSystem->IsTrackedDeviceConnected(state->deviceid))
            continue;
        mVRSystem->GetControllerState(state->deviceid, &controllerState, sizeof(controllerState));
        int j = state->idjoystick;
        state->joystickX = controllerState.rAxis[j].x;
        state->joystickY = controllerState.rAxis[j].y;
        IO::mJoystickValue[i].x = state->joystickX;
        IO::mJoystickValue[i].y = state->joystickY;
    }
}

void OpenVRWrapper::ProcessEvent(const vr::VREvent_t& e)
{
    auto trackedDeviceClass = mVRSystem->GetTrackedDeviceClass(e.trackedDeviceIndex);
    if (trackedDeviceClass != vr::ETrackedDeviceClass::TrackedDeviceClass_Controller)
    {
        return;
    }
    if (e.eventType == vr::VREvent_ButtonPress || e.eventType == vr::VREvent_ButtonUnpress)
    {
        ProcessButtonEvent(e);
    }
}

void OpenVRWrapper::ProcessButtonEvent(const vr::VREvent_t& e)
{
    vr::ETrackedControllerRole role =
        mVRSystem->GetControllerRoleForTrackedDeviceIndex(e.trackedDeviceIndex);
    IO::Button button;
    if (role == vr::TrackedControllerRole_LeftHand)
    {
        switch (e.data.controller.button)
        {
        case vr::EVRButtonId::k_EButton_A:
            button = IO::Button::X;
            break;
        case vr::EVRButtonId::k_EButton_ApplicationMenu:
            button = IO::Button::Y;
            break;
        case vr::EVRButtonId::k_EButton_Axis3:
            button = IO::Button::L1;
            break;
        case vr::EVRButtonId::k_EButton_Axis1:
            button = IO::Button::L2;
            break;
        case vr::EVRButtonId::k_EButton_Axis0:
            button = IO::Button::LJ;
            break;
        case vr::EVRButtonId::k_EButton_Grip:
            button = IO::Button::LG;
            break;
        default:
            std::cout << "unknown\n";
            break;
        }
        std::cout << std::endl;
    }
    else if (role == vr::TrackedControllerRole_RightHand)
    {
        switch (e.data.controller.button)
        {
        case vr::EVRButtonId::k_EButton_A:
            button = IO::Button::A;
            break;
        case vr::EVRButtonId::k_EButton_ApplicationMenu:
            button = IO::Button::B;
            break;
        case vr::EVRButtonId::k_EButton_Axis3:
            button = IO::Button::R1;
            break;
        case vr::EVRButtonId::k_EButton_Axis1:
            button = IO::Button::R2;
            break;
        case vr::EVRButtonId::k_EButton_Axis0:
            button = IO::Button::RJ;
            break;
        case vr::EVRButtonId::k_EButton_Grip:
            button = IO::Button::RG;
            break;
        default:
            std::cout << "unknown\n";
            break;
        }
    }
    switch (e.eventType)
    {
    case vr::VREvent_ButtonPress:
    {
        if (IO::mButtonStatus[(unsigned)button] == Status::Down || 
            IO::mButtonStatus[(unsigned)button] == Status::Held)
        {
            IO::mButtonStatus[(unsigned)button] = Status::Held;
        }
        else
        {
            IO::mButtonStatus[(unsigned)button] = Status::Down;
        }
        break;
    }
    case vr::VREvent_ButtonUnpress:
    {
        if (IO::mButtonStatus[(unsigned)button] == Status::Down ||
            IO::mButtonStatus[(unsigned)button] == Status::Held)
        {
            IO::mButtonStatus[(unsigned)button] = Status::Up;
        }
        else
        {
            IO::mButtonStatus[(unsigned)button] = Status::Release;
        }
        break;
    }
    }
}

glm::mat4 OpenVRWrapper::ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t& mat)
{
    glm::mat4 result(
        glm::vec4(mat.m[0][0], mat.m[1][0], mat.m[2][0], 0.0),
        glm::vec4(mat.m[0][1], mat.m[1][1], mat.m[2][1], 0.0),
        glm::vec4(mat.m[0][2], mat.m[1][2], mat.m[2][2], 0.0),
        glm::vec4(mat.m[0][3], mat.m[1][3], mat.m[2][3], 1.0)
    );
    return result;
}

glm::mat4 OpenVRWrapper::ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix44_t& mat)
{
    glm::mat4 result(
        glm::vec4(mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0]),
        glm::vec4(mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1]),
        glm::vec4(mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2]),
        glm::vec4(mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3])
    );
    return result;
}

glm::vec3 OpenVRWrapper::CalcPosition(const vr::HmdMatrix34_t& matPose)
{
    return glm::vec3(matPose.m[0][3], matPose.m[1][3], matPose.m[2][3]);
}

glm::quat OpenVRWrapper::CalcRotation(const vr::HmdMatrix34_t& matrix)
{
    glm::quat q;
    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}

bool OpenVRWrapper::IO::IsButtonPressed(Button key)
{
    return mButtonStatus[(unsigned)key] == Status::Down;
}

bool OpenVRWrapper::IO::IsButtonHeld(Button key)
{
    return mButtonStatus[(unsigned)key] == Status::Held;
}

bool OpenVRWrapper::IO::IsButtonUp(Button key)
{
    return mButtonStatus[(unsigned)key] == Status::Up;
}

bool OpenVRWrapper::IO::IsButtonReleased(Button key)
{
    return mButtonStatus[(unsigned)key] == Status::Release;
}

glm::vec2 OpenVRWrapper::IO::GetJoystickValue(Hand hand)
{
    return mJoystickValue[(unsigned)hand];
}

void OpenVRWrapper::IO::ClearStatus()
{
    for (size_t i = 0; i < 12; i++)
    {
        if (mButtonStatus[i] == Status::Down)
        {
            mButtonStatus[i] = Status::Held;
        }
        if (mButtonStatus[i] == Status::Up)
        {
            mButtonStatus[i] = Status::Release;
        }
    }
}
