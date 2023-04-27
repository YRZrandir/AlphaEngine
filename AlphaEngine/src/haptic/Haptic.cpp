#include "Haptic.h"

Haptic* Haptic::_instance = nullptr;

bool Haptic::Init( std::function<void()> func )
{
    try
    {
        _instance = new Haptic( func );
        //_instance->_thread->start( Haptic::RenderHaptic, chai3d::CThreadPriority::CTHREAD_PRIORITY_HAPTICS );
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    return true;
}

void Haptic::Disconnect()
{
    _instance->_run = false;
    _instance->_thread->stop();
    _instance->_device->close();
    delete _instance->_handler;
}

Haptic& Haptic::Get()
{
    return *_instance;
}

void Haptic::RenderHaptic()
{
    while (_instance->_run)
    {
        _instance->_render_haptics();
    }
}

Haptic::Haptic( std::function<void()> func )
{
    _render_haptics = func;

    _handler = new chai3d::cHapticDeviceHandler();

    if (!_handler->getDevice( _device, 0 ))
        throw std::runtime_error( "Failed to get device" );

    _infos = _device->getSpecifications();

    if (_device->open())
        std::cout << "CHAI3D: Device Name=" << _infos.m_modelName << std::endl;
    else
        throw std::runtime_error( "Cannot Open Haptic Device. Check device connection and drivers." );

    _device->calibrate();

    _thread = new chai3d::cThread();
}

Haptic::~Haptic()
{
    //_device->close();
    //_thread->stop();
    //delete _handler;
}

bool Haptic::IsOpen()
{
    return _device != nullptr;
}

Eigen::Vector3f Haptic::GetPos()
{
    chai3d::cVector3d vec;
    _device->getPosition( vec );
    return Eigen::Vector3f( vec.x(), vec.y(), vec.z() );
}

void Haptic::AddForce( Eigen::Vector3f f )
{
    _device->setForce( chai3d::cVector3d( f.x(), f.y(), f.z() ) );
}

const chai3d::cHapticDeviceInfo& Haptic::GetInfo() const
{
    return _infos;
}
