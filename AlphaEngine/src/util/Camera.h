#pragma once
#include <memory>
#include <unordered_map>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "../model/Transform.h"
#include "../util/Scene.h"
#include "../util/SceneObject.h"
#include "../model/Ray.h"


class Camera
    : public SceneObject
{
public:
    Camera( const std::string& name );
    virtual glm::mat4 GetViewMatrix() const;
    virtual glm::mat4 GetProjectionMatrix() const;
    glm::vec3 WorldToScreenPoint( glm::vec3 p ) const;
    glm::vec3 WorldToViewportPoint( glm::vec3 p ) const;
    Ray		  ScreenPointToRay( glm::vec2 p ) const;
    void Update() override = 0;
    void Draw() override = 0;
    virtual void DrawGUI() = 0;
    static Camera* main;
    static Camera* current;
    glm::vec2 _viewport_corner = glm::vec2( 0, 0 );
    glm::vec2 _viewport_size;
    float _pitch;
    float _yaw;

protected:
    float _fov;
    float _far_plane;
    float _near_plane;
};


class SurroundCamera : public Camera
{
public:
    SurroundCamera( const std::string& name, glm::vec3 pos = glm::vec3( 0.0f, 0.0f, 0.0f ), float yaw = 0.0f, float pitch = 0.0f );
    // 通过 Camera 继承
    virtual void Update() override;
    virtual void Draw() override;
    virtual void DrawGUI() {}
public:
    float _mouse_speed = 0.3f;
    float _zoom = 1.5f;

private:
    void ProcessMouseMovement( float xoffset, float yoffset );
    void ProcessMouseScroll( float yoffset );
};

class FreeCamera : public Camera
{
public:
    float mSpeed = 0.5f;

    FreeCamera( const std::string& name, glm::vec3 pos, float speed );
    // Inherited via Camera
    virtual void Update() override;
    virtual void Draw() override;
    virtual void DrawGUI() override;
};


class VRCamera : public Camera
{
public:
    glm::mat4 mViewMat;
    glm::mat4 mProjMat;
    int mEye = 0; //0==left, 1==right

    VRCamera( const std::string& name, int eye );
    virtual glm::mat4 GetViewMatrix() const override;
    virtual glm::mat4 GetProjectionMatrix() const override;
    virtual void Update() override;
    virtual void Draw() override;
    virtual void DrawGUI() override {}
};
