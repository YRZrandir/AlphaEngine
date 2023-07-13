#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <shaderc/shaderc.hpp>
#include <libshaderc_util/file_finder.h>

enum class ShaderDataType
{
    Int32,
    Uint32,
    Float32,
    Vec2f,
    Vec3f,
    Vec4f,
    Mat3f,
    Mat4f,
    Sampler2D,
    SamplerCube,
    Unknown
};

ShaderDataType GLTypeToShaderDataType( GLenum type );

GLenum ShaderDataTypeToGLType( ShaderDataType type );

constexpr const char* ShaderDataTypeToString( ShaderDataType type );

class ShaderUniformInfo
{
public:
    ShaderUniformInfo( std::string name, ShaderDataType type )
        :_name( name ), _type( type )
    {
        if (sPredefinedEnvVarNames.contains( name.substr( 0, name.find( '.' ) ) ))
            _env = true;
        else
            _env = false;
    }

    std::string _name{ "" };
    ShaderDataType _type{ ShaderDataType::Unknown };
    bool _env{ true };

    static const std::unordered_set<std::string> sPredefinedEnvVarNames;
};

struct ShaderVertexAttribInfo
{
    std::string name;
    ShaderDataType type;
};

class Shader
{
public:
    using shader_type = GLuint;
    using shader_paths = std::vector<std::pair<std::string, shader_type>>;

    Shader( const shader_paths& infos );
    Shader( const std::string& path );
    ~Shader();
    Shader( const Shader& ) = delete;
    Shader& operator=( const Shader& ) = delete;
    Shader( Shader&& ) noexcept;
    Shader& operator=( Shader&& ) noexcept;

    void use();
    void setBool( const std::string& name, GLboolean value );
    void setInt( const std::string& name, GLint value );
    void setFloat( const std::string& name, GLfloat value );
    void setUnsignedInt( const std::string& name, GLuint value );
    void setMat( const std::string& name, glm::mat3x4 value );
    void setMat( const std::string& name, glm::mat4x3 value );
    void setMat( const std::string& name, glm::mat4 value );
    void setMat( const std::string& name, glm::mat3 value );
    void setVec( const std::string& name, glm::vec2 value );
    void setVec( const std::string& name, glm::vec3 value );
    void setVec( const std::string& name, glm::vec4 value );
    void BuildShaderInfo();
    void PrintShaderInfo() const;

protected:
    GLuint mID;
    std::string _path;
    shader_paths _paths;
    std::vector<ShaderVertexAttribInfo> _vertex_attribs;
    std::vector<ShaderUniformInfo> _uniform_infos;

    std::string readFile( const char* path );
    std::vector<std::pair<std::string, GLuint>> splitShaderCode( const std::string& code );
    void PreprocessShader( std::vector<std::pair<std::string, GLuint>>& shader_sources );
    static shaderc_shader_kind ShaderStageToShaderc( GLuint stage );
    GLuint openglCreateShader( const GLchar* code, shader_type type, const std::string& name );
    GLuint BuildProgram( const std::vector<std::pair<std::string, GLuint>>& shaderCodeTypeArray );

    /*SHADER MANAGEMENT*/
public:
    static void Add( std::unique_ptr<Shader>&& shader, const std::string& name );
    static void Remove( const std::string& name );
    static Shader* Find( const std::string& name );
protected:
    static std::unordered_map<std::string, std::unique_ptr<Shader>> shaders;
};


class ShaderIncluder : public shaderc::CompileOptions::IncluderInterface
{
public:
    ShaderIncluder( const shaderc_util::FileFinder* _file_finder );
    virtual shaderc_include_result* GetInclude( const char* requested_source, shaderc_include_type type, const char* requesting_source, size_t include_depth ) override;
    virtual void ReleaseInclude( shaderc_include_result* data ) override;

protected:
    const shaderc_util::FileFinder* _file_finder;
};

