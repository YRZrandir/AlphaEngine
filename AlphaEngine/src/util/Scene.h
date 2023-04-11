#pragma once
#include "../util/SceneObject.h"
#include <array>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include "gl/UniformBuffer.h"

class Scene
{
public:
    Scene();
    virtual void Update();
    virtual void Draw();
    void DrawShadowDepthBuffer();

    template <typename T>
    T* AddChild( std::unique_ptr<T>&& child );
    template <typename T>
    const T* GetChild() const;
    template <typename T>
    T* GetChild();
    template <typename T>
    std::vector<T*> GetAllChildOfType();
    template <typename T>
    const T* GetChild( const std::string& name ) const;
    template <typename T>
    T* GetChild( const std::string& name );
    template <typename T>
    bool RemoveChild();
    template <typename T>
    bool RemoveChild( const std::string& name );
    bool RemoveChild( const std::string& name );

    static std::unique_ptr<Scene> active;

protected:
    struct CameraUniformBlock
    {
        glm::vec3 pos;
        float padding;
        glm::mat4 view_mat;
        glm::mat4 proj_mat;
        glm::mat4 viewproj_mat;
    };

    struct TransformUniformBlock
    {
        glm::mat4 world_mat;
        glm::mat4 normal_mat;
    };

    struct DirLightUniformBlock
    {
        glm::vec3 dir;
        float padding0;
        glm::vec3 ambient;
        float padding1;
        glm::vec3 diffuse;
        float padding2;
        glm::vec3 specular;
        float intensity;
        glm::mat4 light_space_mat;
    };

    struct PointLightUniformBlock
    {
        glm::vec3 pos;
        float padding0;
        glm::vec3 color;
        float intensity;
        float att_const;
        float att_linear;
        float att_exp;
        float padding1;
    };

    struct LightUniformBlock
    {
        unsigned int nb_dirlights;
        unsigned int nb_pointlights;
        float padding0[2];
        std::array<DirLightUniformBlock, 5> dirlights;
        std::array<PointLightUniformBlock, 5> pointlights;
    };

    void SetUniformBuffers();
    void SetUniformBuffersForObject( const SceneObject& obj );
    void DrawChild( SceneObject& obj );
public:
    //TODO This should not be public. Maybe move to a Renderer class.
    CameraUniformBlock _camera_ubo_info;
    LightUniformBlock _lights_ubo_info;
    TransformUniformBlock _transform_ubo_info;
    std::unique_ptr<UniformBuffer> _camera_ubo{ nullptr };
    std::unique_ptr<UniformBuffer> _lights_ubo{ nullptr };
    std::unique_ptr<UniformBuffer> _transform_ubo{ nullptr };
    std::vector<std::unique_ptr<SceneObject>> _children;
};

template<typename T>
T* Scene::AddChild( std::unique_ptr<T>&& child )
{
    SceneObject* ptr = dynamic_cast<SceneObject*>(child.get());
    if (ptr)
    {
        _children.push_back( std::move( child ) );
        return dynamic_cast<T*>(_children.back().get());
    }
    return nullptr;
}

template<typename T>
const T* Scene::GetChild() const
{
    for (const auto& child : _children)
    {
        const T* ptr = const_cast<const T*>(dynamic_cast<T*>(child.get()));
        if (ptr)
        {
            return ptr;
        }
    }
    return nullptr;
}

template <typename T>
T* Scene::GetChild()
{
    for (const auto& child : _children)
    {
        T* ptr = dynamic_cast<T*>(child.get());
        if (ptr)
        {
            return ptr;
        }
    }
    return nullptr;
}

template <typename T>
std::vector<T*> Scene::GetAllChildOfType()
{
    std::vector<T*> result;
    for (const auto& child : _children)
    {
        T* ptr = dynamic_cast<T*>(child.get());
        if (ptr)
        {
            result.push_back( ptr );
        }
    }
    return result;
}

template<typename T>
const T* Scene::GetChild( const std::string& name ) const
{
    for (const auto& child : _children)
    {
        const T* ptr = const_cast<const T*>(dynamic_cast<T*>(child.get()));
        if (ptr && child->mName == name)
        {
            return ptr;
        }
    }
    return nullptr;
}

template <typename T>
T* Scene::GetChild( const std::string& name )
{
    for (const auto& child : _children)
    {
        T* ptr = dynamic_cast<T*>(child.get());
        if (ptr && child->mName == name)
        {
            return ptr;
        }
    }
    return nullptr;
}

template <typename T>
bool Scene::RemoveChild()
{
    for (auto it = std::begin( _children ); it != std::end( _children ); it++)
    {
        if (dynamic_cast<T*>((*it).get()))
        {
            it->release();
            _children.erase( it );
            return true;
        }
    }
    return false;
}

template <typename T>
bool Scene::RemoveChild( const std::string& name )
{
    for (auto it = std::begin( _children ); it != std::end( _children ); it++)
    {
        if (dynamic_cast<T*>((*it).get()) && (*it)->mName == name)
        {
            it->release();
            _children.erase( it );
            return true;
        }
    }
    return false;
}
