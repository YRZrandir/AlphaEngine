#pragma once
#include "VertexArray.h"
#include "IndexBuffer.h"
#include "UniformBuffer.h"
#include "util/Camera.h"
#include "lighting/Light.h"
#include "material/Material.h"

class Renderer
{
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

public:
    static Renderer& Get();
    void Draw( const VertexArray& vao, const Material& material );
    void DrawShadowDepth( const VertexArray& vao, std::string shader = "depth" );
    void UpdateEnvUniforms();
    void UpdateTranformUniform();
    void UpdateCameraUniform();

    void AddDirLight( glm::vec3 dir, glm::vec3 ambient, glm::vec3 diffuse, glm::vec3 specular, float intensity, glm::mat4 light_space_mat );
    void SetDirLight( unsigned id, glm::vec3 dir, glm::vec3 ambient, glm::vec3 diffuse, glm::vec3 specular, float intensity, glm::mat4 light_space_mat );
    void ClearDirLights();
    void AddPointLight( glm::vec3 pos, glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp );
    void SetPointLight( unsigned id, glm::vec3 pos, glm::vec3 color, float intensity, float att_const, float att_linear, float att_exp );
    void ClearPointLights();
    void SetCamera( glm::vec3 pos, glm::mat4 view_mat, glm::mat4 proj_mat );
    void SetTransform( glm::mat4 world_mat );

protected:
    CameraUniformBlock _camera_ubo_info;
    LightUniformBlock _lights_ubo_info;
    TransformUniformBlock _transform_ubo_info;
    std::unique_ptr<UniformBuffer> _camera_ubo{ nullptr };
    std::unique_ptr<UniformBuffer> _lights_ubo{ nullptr };
    std::unique_ptr<UniformBuffer> _transform_ubo{ nullptr };
    static std::unique_ptr<Renderer> _instance;
};

