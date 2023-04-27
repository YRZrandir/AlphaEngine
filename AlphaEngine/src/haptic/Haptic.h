#pragma once
#include <memory>
#include "chai3d.h"
#include <Eigen/Eigen>

class Haptic
{
public:
    static bool Init( std::function<void()> );
    static void Disconnect();
    static Haptic& Get();
    bool IsOpen();
    Eigen::Vector3f GetPos();
    void AddForce( Eigen::Vector3f f );
    const chai3d::cHapticDeviceInfo& GetInfo() const;

private:
    Haptic( std::function<void()> );
    ~Haptic();
private:
    static Haptic* _instance;
    static void RenderHaptic();

    chai3d::cHapticDeviceHandler* _handler = nullptr;
    std::shared_ptr<chai3d::cGenericHapticDevice> _device = nullptr;
    chai3d::cHapticDeviceInfo _infos;
    chai3d::cThread* _thread;
    std::function<void()> _render_haptics;
    bool _run = true;
};