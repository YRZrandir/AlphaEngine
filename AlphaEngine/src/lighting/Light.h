﻿#pragma once
#include <string>
#include <unordered_map>
#include <memory>
#include <glm/glm.hpp>
#include "model/Transform.h"
#include "util/SceneObject.h"
#include "gl/FrameBuffer.h"
#include "ECS/ECS.h"

class Shader;

class Light : public Component
{
public:
    virtual void SetShaderUniforms( Shader& shader ) = 0;

    virtual glm::mat4 GetLightSpaceMat() const = 0;
    virtual glm::mat4 GetLightSpaceViewMat() const = 0;
    virtual glm::mat4 GetLightSpaceProjMat() const = 0;
    static void SetAllLightUniforms( Shader& shader );
};


class DirLight : public Light
{
public:
    DirLight( glm::vec3 ambient, glm::vec3 diffuse, float intensity );
    // 通过 Light 继承
    virtual void SetShaderUniforms( Shader& shader ) override;
    virtual void DrawGUI() override;
    virtual glm::mat4 GetLightSpaceMat() const override;
    virtual glm::mat4 GetLightSpaceViewMat() const override;
    virtual glm::mat4 GetLightSpaceProjMat() const override;
    FrameBuffer* GetShadowDepthBuffer();
    bool CastShadow() const;
    void CastShadow( bool value );

    glm::vec3 ambient = glm::vec3( 0.04f );
    glm::vec3 diffuse = glm::vec3( 0.8f );
    float _intensity = 1.0f;
    int id = 0;
protected:
    friend class Light;
    bool _cast_shadow = true;
    std::unique_ptr<FrameBuffer> _shadow_depth_buffer = nullptr;

    static std::unique_ptr<Texture2DArray> _shadow_depth_texarray;
    static int _instance_count;
};

class PointLight : public Light
{
public:
    PointLight( glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp );
    virtual void SetShaderUniforms( Shader& shader ) override;
    virtual glm::mat4 GetLightSpaceMat() const override;
    virtual glm::mat4 GetLightSpaceViewMat() const override;
    virtual glm::mat4 GetLightSpaceProjMat() const override;
    glm::vec3 _color;
    float _intensity;
    float _att_const;
    float _att_linear;
    float _att_exp;
};