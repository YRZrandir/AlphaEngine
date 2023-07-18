#pragma once
#include <vector>
#include <string>
#include <memory>
#include <glm/glm.hpp>
#include "Texture.h"
#include "ECS/ECS.h"

class Shader;

const glm::vec3		DEFAULT_AMBIENT_COLOR( 0, 1, 0 );
const glm::vec3		DEFAULT_DIFFUSE_COLOR( 0, 0.6, 1 );
const glm::vec3		DEFAULT_SPECULAR_COLOR( 1, 1, 1 );
const float			DEFAULT_SHININESS = 32.0f;

class MaterialInfos
{
public:
    std::string		diffuseTexPath;
    std::string		specularTexPath;
    std::string		normalTexPath;
    std::string     roughnessTexPath;
    std::string     metallicTexPath;
    glm::vec3		diffuseColor = DEFAULT_DIFFUSE_COLOR;
    glm::vec3		specularColor = DEFAULT_SPECULAR_COLOR;
    float			shininess = DEFAULT_SHININESS;
    float			roughness = 0.25f;
    float			metallic = 0.01f;
    float			alpha = 1.0f;
    bool			useDiffuseColor = false;
    bool			useSpecularColor = false;
};

class Material : public Component
{
public:
    Material();
    Material( const std::string& directory, const std::string& name, const MaterialInfos& materialInfos );

    void SetShaderUniforms( Shader& shader ) const;
    void UpdateTextureUnits( GLuint minUnit );
    GLuint GetTextureUnitUsage() const;
    void SetDiffuseColor( float r, float g, float b );
    virtual void DrawGUI();

    std::unique_ptr<Texture>	mDiffuseTexture = nullptr;
    std::unique_ptr<Texture>	mSpecularTexture = nullptr;
    std::unique_ptr<Texture>	mNormalTexture = nullptr;
    std::unique_ptr<Texture>    mRoughnessTexture = nullptr;
    std::unique_ptr<Texture>    mMetallicTexture = nullptr;

    glm::vec3 mDiffuseColor = DEFAULT_DIFFUSE_COLOR;
    glm::vec3 mSpecularColor = DEFAULT_SPECULAR_COLOR;
    float	mShininess = 32.0f;
    float	mAlpha = 1.0f;
    float	mMetallic = 0.01f;
    float	mRoughness = 0.25f;
    std::string mShader = "model";
private:
    std::string	mName = "material";
    std::string	mDirectory;
    int _unit_start_pos = 0;
};

//class BRDFMaterial
//{
//public:
//    BRDFMaterial( const MaterialInfos& materialInfos );
//
//};