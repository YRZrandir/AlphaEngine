#pragma once
#include <memory>
#include <openvr.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class FrameBuffer;

class OpenVRWrapper
{
public:
    class IO;

    OpenVRWrapper() = delete;  
    ~OpenVRWrapper() = delete;
    OpenVRWrapper(const OpenVRWrapper&) = delete;
    OpenVRWrapper(OpenVRWrapper&&) = delete;
    OpenVRWrapper& operator=(const OpenVRWrapper&) = delete;
    OpenVRWrapper& operator=(OpenVRWrapper&&) = delete;

    static void Init();
    static bool UseVr();

    static void HandleInput();
    static void UpdateMatrix();

    static void         BindLeftEye();
    static void         BindRightEye();
    static unsigned int GetRenderWidth();
    static unsigned int GetRenderHeight();
    static glm::mat4    GetLeftEyePose();
    static glm::mat4    GetRightEyePose();
    static glm::vec3    GetHMDPose();
    static glm::quat    GetHMDRotate();
    static glm::mat4    GetHMDMat();
    static glm::mat4    GetLeftProjection();
    static glm::mat4    GetRightProjection();
    static glm::vec3    GetRightHandPos();
    static glm::quat    GetRightHandRotate();
    static glm::mat4    GetRightHandMat();
    static glm::vec3    GetLeftHandPos();
    static glm::quat    GetLeftHandRotate();
    static glm::mat4    GetLeftHandMat();
    static void         Submit();

private:
    struct ControllerState
    {
        vr::VRControllerState_t state;
        int hand;   //0=left, 1=right, 2=invalid
        bool show = true;
        glm::mat4 mat;
        glm::vec3 pos;
        glm::quat rotate;
        int deviceid;
        int idtrigger;
        int idpad;
        int idjoystick;
        float triggerVal;
        float padX;
        float padY;
        float joystickX;
        float joystickY;
    };

    struct HMDState
    {
        int deviceid;
        glm::vec3 pos;
        glm::quat rotate;
        glm::mat4 mat;
    };

    static bool             mInit;
    static vr::IVRSystem*   mVRSystem;
    static unsigned int     mRenderWidth;
    static unsigned int     mRenderHeight;
    static std::unique_ptr<FrameBuffer> mLeftFrame;
    static std::unique_ptr<FrameBuffer> mRightFrame;

    static vr::TrackedDevicePose_t  mTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    static ControllerState          mControllerState[2];
    static HMDState                 mHmdState;

    static glm::mat4 mEyePosLeft;
    static glm::mat4 mEyePosRight;

    static glm::mat4 mProjectionCenter;
    static glm::mat4 mProjectionLeft;
    static glm::mat4 mProjectionRight;

    static void ProcessEvent(const vr::VREvent_t& e);
    static void ProcessButtonEvent(const vr::VREvent_t& e);

    static glm::mat4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t& matPose);
    static glm::mat4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix44_t& matPose);
    static glm::vec3 CalcPosition(const vr::HmdMatrix34_t& mat);
    static glm::quat CalcRotation(const vr::HmdMatrix34_t& mat);
};


class OpenVRWrapper::IO
{
public:
    enum class Button { A, B, X, Y, R1, R2, L1, L2, LG, RG, LJ, RJ };

    enum class Hand { Left, Right };

    enum class Status
    {
        Down,   //button is pressed down in this frame
        Held,   //button was pressed down and being held in this frame
        Up,     //button is released in this frame
        Release //button is not pressed
    };

    static bool IsButtonPressed(Button key);
    static bool IsButtonHeld(Button key);
    static bool IsButtonUp(Button key);
    static bool IsButtonReleased(Button key);

    static glm::vec2 GetJoystickValue(Hand hand);

private:
    friend class OpenVRWrapper;
    static void ClearStatus();
    static glm::vec2 mJoystickValue[2];
    static Status mButtonStatus[12];
};