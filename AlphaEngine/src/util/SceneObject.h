#pragma once
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include "../model/Transform.h"
#include "RenderConfig.h"

class SceneObject
{
public:
    std::string mName;
    Transform   mTransform;
    int mLayer = 0; //small number will be draw first
    RenderConfig mRenderConfig;

    SceneObject() = default;
    SceneObject( const std::string& name );
    SceneObject( const std::string& name, Transform transform );
    virtual ~SceneObject() = default;

    virtual void Update() = 0;
    virtual void Draw() = 0;
    virtual void DrawShadowDepth() {}
    //std::unique_ptr<SceneObject> clone() const;

protected:
    SceneObject( const SceneObject& ) = default;
    SceneObject( SceneObject&& ) = default;
    SceneObject& operator=( const SceneObject& ) = default;
    SceneObject& operator=( SceneObject&& ) = default;

private:
    //virtual SceneObject* virtual_clone() const = 0;
};
