#include "Material.h"
#include "util/Shader.h"
#include "util/util.h"

Material::Material()
{
    unsigned char trival[3] = { 255, 255, 255 };
    mDiffuseTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::DIFFUSE, trival );
    mSpecularTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::SPECULAR, trival );
    unsigned char trival_normal[3] = { 127, 127, 255 };
    mNormalTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::NORMAL, trival_normal );
    float trival_roughness = 1.0;
    mRoughnessTexture = std::make_unique<Texture>( 1, 1, 1, Texture::Types::ROUGHNESS, &trival_roughness );
    float trival_metal = 1.0;
    mMetallicTexture = std::make_unique<Texture>( 1, 1, 1, Texture::Types::METALLIC, &trival_metal );
}

Material::Material( const std::string& directory, const std::string& name, const MaterialInfos& materialInfos )
    : mDirectory( directory ),
    mDiffuseColor( materialInfos.diffuseColor ),
    mSpecularColor( materialInfos.specularColor ),
    mShininess( materialInfos.shininess ),
    mRoughness( materialInfos.roughness ),
    mMetallic( materialInfos.metallic )
{
    if (!materialInfos.diffuseTexPath.empty())
    {
        mDiffuseTexture = std::make_unique<Texture>( directory + materialInfos.diffuseTexPath, Texture::Types::DIFFUSE );
    }
    else
    {
        unsigned char trival[3] = { 255, 255, 255 };
        mDiffuseTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::DIFFUSE, trival );
    }
    if (!materialInfos.specularTexPath.empty())
    {
        mSpecularTexture = std::make_unique<Texture>( directory + materialInfos.specularTexPath, Texture::Types::SPECULAR );
    }
    else
    {
        unsigned char trival[3] = { 255, 255, 255 };
        mSpecularTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::SPECULAR, trival );
    }
    if (!materialInfos.normalTexPath.empty())
    {
        mNormalTexture = std::make_unique<Texture>( directory + materialInfos.normalTexPath, Texture::Types::NORMAL );
    }
    else
    {
        unsigned char trival_normal[3] = { 127, 127, 255 };
        mNormalTexture = std::make_unique<Texture>( 1, 1, 3, Texture::Types::NORMAL, trival_normal );
    }
    if (!materialInfos.roughnessTexPath.empty())
    {
        mRoughnessTexture = std::make_unique<Texture>( directory + materialInfos.roughnessTexPath, Texture::Types::ROUGHNESS );
    }
    else
    {
        float trival = 1.0;
        mRoughnessTexture = std::make_unique<Texture>( 1, 1, 1, Texture::Types::ROUGHNESS, &trival );
    }

    if (!materialInfos.metallicTexPath.empty())
    {
        mMetallicTexture = std::make_unique<Texture>( directory + materialInfos.metallicTexPath, Texture::Types::METALLIC );
    }
    else
    {
        float trival = 1.0;
        mMetallicTexture = std::make_unique<Texture>( 1, 1, 1, Texture::Types::METALLIC, &trival );
    }

}

void Material::SetShaderUniforms( Shader& shader ) const
{
    shader.use();
    int unit = _unit_start_pos;

    mDiffuseTexture->SetShaderUniform( shader, mName, unit++ );
    mSpecularTexture->SetShaderUniform( shader, mName, unit++ );
    mNormalTexture->SetShaderUniform( shader, mName, unit++ );
    mRoughnessTexture->SetShaderUniform( shader, mName, unit++ );
    mMetallicTexture->SetShaderUniform( shader, mName, unit++ );

    shader.setVec( mName + ".diffuse", mDiffuseColor );
    shader.setVec( mName + ".specular", mSpecularColor );
    shader.setFloat( mName + ".roughness", mRoughness );
    shader.setFloat( mName + "metallic", mMetallic );
    shader.setFloat( mName + ".alpha", mAlpha );
}

void Material::UpdateTextureUnits( GLuint minUnit )
{
    _unit_start_pos = minUnit;
}

GLuint Material::GetTextureUnitUsage() const
{
    return 5;
}

void Material::SetDiffuseColor( float r, float g, float b )
{
    auto p = mDiffuseTexture->GetImage().Data();
    p[0] = static_cast<unsigned char>(r * 255);
    p[1] = static_cast<unsigned char>(g * 255);
    p[2] = static_cast<unsigned char>(b * 255);
    mDiffuseTexture->UpdateData();
}