#include "Material.h"
#include "util/Shader.h"
#include "util/util.h"
#include "ui/ImGuiFileDialog.h"

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
    shader.setFloat( mName + ".metallic", mMetallic );
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

void Material::DrawGUI()
{
    ImGui::Text( "Albedo" );
    if (ImGui::ImageButton( reinterpret_cast<void*>(mDiffuseTexture->GetID()), { 20, 20 } ))
    {
        ImGuiFileDialog::Instance()->OpenDialog( "Choose Image1", "Choose Image", "{.jpg,.png,.bmp,.jpeg}", ".", "" );
    }
    if (ImGuiFileDialog::Instance()->Display( "Choose Image1" ))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            mDiffuseTexture->LoadFromFile( path );
            mDiffuseColor = glm::vec3( 1.f );
        }
        ImGuiFileDialog::Instance()->Close();
    }
    ImGui::ColorEdit3( "Base Color", &mDiffuseColor[0] );

    ImGui::Text( "Roughness" );
    if (ImGui::ImageButton( reinterpret_cast<void*>(mRoughnessTexture->GetID()), { 20, 20 } ))
    {
        ImGuiFileDialog::Instance()->OpenDialog( "Choose Image2", "Choose Image", "{.jpg,.png,.bmp,.jpeg}", ".", "" );
    }
    if (ImGuiFileDialog::Instance()->Display( "Choose Image2" ))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            mRoughnessTexture->LoadFromFile( path );
            mRoughness = 1.f;
        }
        ImGuiFileDialog::Instance()->Close();
    }
    ImGui::DragFloat( "Roughness", &mRoughness, 0.01f, 0.f, 1.f );

    ImGui::Text( "Metallic" );
    if (ImGui::ImageButton( reinterpret_cast<void*>(mMetallicTexture->GetID()), { 20, 20 } ))
    {
        ImGuiFileDialog::Instance()->OpenDialog( "Choose Image3", "Choose Image", "{.jpg,.png,.bmp,.jpeg}", ".", "" );
    }
    if (ImGuiFileDialog::Instance()->Display( "Choose Image3" ))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            mMetallicTexture->LoadFromFile( path );
            mMetallic = 1.f;
        }
        ImGuiFileDialog::Instance()->Close();
    }
    ImGui::DragFloat( "Metallic", &mMetallic, 0.01f, 0.f, 1.f );

    ImGui::Text( "Normal" );
    if (ImGui::ImageButton( reinterpret_cast<void*>(mNormalTexture->GetID()), { 20, 20 } ))
    {
        ImGuiFileDialog::Instance()->OpenDialog( "Choose Image4", "Choose Image", "{.jpg,.png,.bmp,.jpeg}", ".", "" );
    }
    if (ImGuiFileDialog::Instance()->Display( "Choose Image4" ))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string path = ImGuiFileDialog::Instance()->GetFilePathName();
            mNormalTexture->LoadFromFile( path );
        }
        ImGuiFileDialog::Instance()->Close();
    }
}