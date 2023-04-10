#include "Shader.h"
#include <array>
#include <filesystem>
#include <format>

std::unordered_map<std::string, std::unique_ptr<Shader>> Shader::shaders = {};

const std::unordered_set<std::string> ShaderUniformInfo::sPredefinedEnvVarNames
{
    "Camera",
    "Lights",
    "Transform"
};

ShaderDataType GLTypeToShaderDataType( GLenum type )
{
    static std::unordered_map<GLenum, ShaderDataType> sTable{
        { GL_FLOAT,      ShaderDataType::Float32 },
        { GL_FLOAT_VEC2, ShaderDataType::Vec2f },
        { GL_FLOAT_VEC3, ShaderDataType::Vec3f },
        { GL_FLOAT_VEC4, ShaderDataType::Vec4f },
        { GL_INT,        ShaderDataType::Int32 },
        { GL_UNSIGNED_INT, ShaderDataType::Uint32 },
        { GL_FLOAT_MAT3, ShaderDataType::Mat3f },
        { GL_FLOAT_MAT4, ShaderDataType::Mat4f },
        { GL_SAMPLER_2D, ShaderDataType::Sampler2D },
        { GL_SAMPLER_CUBE, ShaderDataType::SamplerCube }
    };

    if (sTable.count( type ) > 0)
    {
        return sTable[type];
    }
    return ShaderDataType::Unknown;
}

GLenum ShaderDataTypeToGLType( ShaderDataType type )
{
    static std::unordered_map<ShaderDataType, GLenum> sTable{
        { ShaderDataType::Float32, GL_FLOAT },
        { ShaderDataType::Vec2f, GL_FLOAT_VEC2 },
        { ShaderDataType::Vec3f, GL_FLOAT_VEC3 },
        { ShaderDataType::Vec4f, GL_FLOAT_VEC4 },
        { ShaderDataType::Int32, GL_INT },
        { ShaderDataType::Uint32, GL_UNSIGNED_INT },
        { ShaderDataType::Mat3f, GL_FLOAT_MAT3 },
        { ShaderDataType::Mat4f, GL_FLOAT_MAT4 },
        { ShaderDataType::Sampler2D, GL_SAMPLER_2D },
        { ShaderDataType::SamplerCube, GL_SAMPLER_CUBE }
    };

    if (sTable.count( type ) > 0)
    {
        return sTable[type];
    }
    return 0;
}

Shader::Shader( const shader_paths& infos )
    :_paths( infos )
{
    std::vector<std::pair<std::string, GLuint>> shader_sources;
    for (auto& pair : infos)
    {
        std::string shaderCode = readFile( pair.first.c_str() );
        shader_sources.emplace_back( shaderCode, pair.second );
    }
    PreprocessShader( shader_sources );
    mID = BuildProgram( shader_sources );
    BuildShaderInfo();
}

Shader::Shader( const std::string& path )
    :_path( path )
{
    auto shader_sources = splitShaderCode( readFile( path.c_str() ) );
    PreprocessShader( shader_sources );
    mID = BuildProgram( shader_sources );
    BuildShaderInfo();
}

Shader::~Shader()
{
    glDeleteProgram( mID );
}

Shader::Shader( Shader&& rh ) noexcept
{
    this->mID = rh.mID;
    rh.mID = 0;
}

Shader& Shader::operator=( Shader&& rh ) noexcept
{
    this->mID = rh.mID;
    rh.mID = 0;
    return *this;
}

void Shader::use()
{
    glUseProgram( mID );
}

std::string Shader::readFile( const char* path )
{
    std::ifstream ifs;
    std::stringstream ss;
    std::string str;

    try
    {
        ifs.open( path );
        ss << ifs.rdbuf();
        str = ss.str();
        ifs.close();
    }
    catch (std::ifstream::failure e)
    {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ\n";
    }
    return str;
}

void Shader::PreprocessShader( std::vector<std::pair<std::string, GLuint>>& shader_sources )
{
    static shaderc::Compiler compiler;
    shaderc_util::FileFinder file_finder;
    file_finder.search_path().emplace_back( "res/shaders/common/" );
    for (auto& [source, stage] : shader_sources)
    {
        shaderc::CompileOptions options;
        options.SetIncluder( std::make_unique<ShaderIncluder>( &file_finder ) );
        const auto preprocessing_result = compiler.PreprocessGlsl( source, ShaderStageToShaderc( stage ), "", options );
        if (preprocessing_result.GetCompilationStatus() != shaderc_compilation_status_success)
            std::cerr << std::format( "Failed to preprocess : {}. \nError: {}", source, preprocessing_result.GetErrorMessage() ) << std::endl;
        source = std::string( preprocessing_result.begin(), preprocessing_result.end() );
    }
}

shaderc_shader_kind Shader::ShaderStageToShaderc( GLuint stage )
{
    switch (stage)
    {
    case GL_VERTEX_SHADER:
        return shaderc_shader_kind::shaderc_vertex_shader;
    case GL_FRAGMENT_SHADER:
        return shaderc_shader_kind::shaderc_fragment_shader;
    case GL_GEOMETRY_SHADER:
        return shaderc_shader_kind::shaderc_geometry_shader;
    case GL_TESS_CONTROL_SHADER:
        return shaderc_shader_kind::shaderc_tess_control_shader;
    case GL_TESS_EVALUATION_SHADER:
        return shaderc_shader_kind::shaderc_tess_evaluation_shader;
    case GL_COMPUTE_SHADER:
        return shaderc_shader_kind::shaderc_compute_shader;
    default:
        return shaderc_shader_kind::shaderc_glsl_infer_from_source;
    }
}

std::vector<std::pair<std::string, GLuint>> Shader::splitShaderCode( const std::string& code )
{
    static const std::string MARK( "###SHADER " );
    static const std::unordered_map<std::string, GLuint> TYPE_MAPPING{
        {"VERT", GL_VERTEX_SHADER},
        {"FRAG", GL_FRAGMENT_SHADER},
        {"GEOM", GL_GEOMETRY_SHADER},
        {"TESC", GL_TESS_CONTROL_SHADER},
        {"TESE", GL_TESS_EVALUATION_SHADER},
        {"COMP", GL_COMPUTE_SHADER }
    };

    std::vector<std::pair<std::string, GLuint>> shaderCodeTypeArray;
    size_t pleft = 0;
    size_t pright = pleft;
    while ((pleft = code.find( MARK, pleft )) != std::string::npos)
    {
        pleft += MARK.length();
        GLuint typeId = TYPE_MAPPING.at( code.substr( pleft, 4 ) );
        pleft += 4;

        if ((pright = code.find( MARK, pleft )) == std::string::npos)
            pright = code.length();

        std::string shaderCode = code.substr( pleft, pright - pleft );
        shaderCodeTypeArray.emplace_back( shaderCode, typeId );
        pleft = pright;
    }

    return shaderCodeTypeArray;
}

GLuint Shader::openglCreateShader( const GLchar* fileContent, shader_type type, const std::string& name )
{
    GLuint shader = glCreateShader( type );
    glShaderSource( shader, 1, &fileContent, NULL );
    glCompileShader( shader );
    GLint success;
    GLchar info[512];
    glGetShaderiv( shader, GL_COMPILE_STATUS, &success );
    if (!success)
    {
        glGetShaderInfoLog( shader, sizeof( info ), nullptr, info );
        std::cout << "ERROR::SHADER COMPILATION FAILED " << "name: " << std::hex << name << '\n'
            << info;
    }
    return shader;
}

GLuint Shader::BuildProgram( const std::vector<std::pair<std::string, GLuint>>& shaderCodeTypeArray )
{
    GLuint id = glCreateProgram();
    std::vector<GLuint> shaderHandles;
    for (const auto& pair : shaderCodeTypeArray)
    {
        GLuint shaderId = openglCreateShader( pair.first.c_str(), pair.second, _path );
        glAttachShader( id, shaderId );
        shaderHandles.push_back( shaderId );
    }
    glLinkProgram( id );
    glLinkProgram( id );
    GLchar info[512];
    GLint success = 1;
    glGetProgramiv( id, GL_LINK_STATUS, &success );
    if (!success)
    {
        glGetProgramInfoLog( id, sizeof( info ), nullptr, info );
        std::cout << "ERROR::PROGRAM_LINK_FAILED\n" << info;
        return 0;
    }
    for (GLuint s : shaderHandles)
    {
        glDeleteShader( s );
    }
    return id;
}

void Shader::BuildShaderInfo()
{
    int attrNum = 0;
    int attrNameMaxLength = 0;
    int uniformNum = 0;
    int uniformNameMaxLength = 0;
    glGetProgramiv( mID, GL_ACTIVE_ATTRIBUTES, &attrNum );
    glGetProgramiv( mID, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &attrNameMaxLength );
    glGetProgramiv( mID, GL_ACTIVE_UNIFORMS, &uniformNum );
    glGetProgramiv( mID, GL_ACTIVE_UNIFORM_MAX_LENGTH, &uniformNameMaxLength );

    std::vector<char> attr_name_buf( attrNameMaxLength );
    for (size_t i = 0; i < attrNum; i++)
    {
        int length = 0;
        int size = 0;
        GLenum type = 0;
        glGetActiveAttrib( mID, i, attrNameMaxLength, &length, &size, &type, attr_name_buf.data() );
        _vertex_attribs.push_back( { std::string( attr_name_buf.begin(), attr_name_buf.begin() + length ), GLTypeToShaderDataType( type ) } );
    }

    std::vector<char> uniform_name_buf( uniformNameMaxLength );
    for (size_t i = 0; i < uniformNum; i++)
    {
        int length = 0;
        int size = 0;
        GLenum type = 0;
        glGetActiveUniform( mID, i, uniformNameMaxLength, &length, &size, &type, uniform_name_buf.data() );
        _uniform_infos.push_back( { std::string( uniform_name_buf.begin(), uniform_name_buf.begin() + length ), GLTypeToShaderDataType( type ) } );
    }
}

void Shader::setBool( const std::string& name, GLboolean value )
{
    glUniform1i( glGetUniformLocation( mID, name.c_str() ), value );
}

void Shader::setInt( const std::string& name, GLint value )
{
    glUniform1i( glGetUniformLocation( mID, name.c_str() ), value );
}

void Shader::setFloat( const std::string& name, GLfloat value )
{
    glUniform1f( glGetUniformLocation( mID, name.c_str() ), value );
}

void Shader::setUnsignedInt( const std::string& name, GLuint value )
{
    glUniform1ui( glGetUniformLocation( mID, name.c_str() ), value );
}

void Shader::setMat( const std::string& name, glm::mat3x4 value )
{
    glUniformMatrix3x4fv( glGetUniformLocation( mID, name.c_str() ), 1, GL_FALSE, glm::value_ptr( value ) );
}

void Shader::setMat( const std::string& name, glm::mat4x3 value )
{
    glUniformMatrix4x3fv( glGetUniformLocation( mID, name.c_str() ), 1, GL_FALSE, glm::value_ptr( value ) );
}

void Shader::setMat( const std::string& name, glm::mat4 value )
{
    glUniformMatrix4fv( glGetUniformLocation( mID, name.c_str() ), 1, GL_FALSE, glm::value_ptr( value ) );
}

void Shader::setMat( const std::string& name, glm::mat3 value )
{
    glUniformMatrix3fv( glGetUniformLocation( mID, name.c_str() ), 1, GL_FALSE, glm::value_ptr( value ) );
}

void Shader::setVec( const std::string& name, glm::vec2 value )
{
    glUniform2fv( glGetUniformLocation( mID, name.c_str() ), 1, glm::value_ptr( value ) );
}

void Shader::setVec( const std::string& name, glm::vec3 value )
{
    glUniform3fv( glGetUniformLocation( mID, name.c_str() ), 1, glm::value_ptr( value ) );
}

void Shader::setVec( const std::string& name, glm::vec4 value )

{
    glUniform4fv( glGetUniformLocation( mID, name.c_str() ), 1, glm::value_ptr( value ) );
}

void Shader::Add( std::unique_ptr<Shader>&& shader, const std::string& name )
{
    shaders[name] = std::move( shader );
}

Shader* Shader::Find( const std::string& name )
{
    auto it = shaders.find( name );
    if (it != std::end( shaders ))
    {
        return it->second.get();
    }
    return nullptr;
}

void Shader::Remove( const std::string& name )
{
    shaders.erase( name );
}

ShaderIncluder::ShaderIncluder( const shaderc_util::FileFinder* file_finder )
    :_file_finder( file_finder )
{}

shaderc_include_result* ShaderIncluder::GetInclude( const char* requested_source, shaderc_include_type type, const char* requesting_source, size_t include_depth )
{
    const std::filesystem::path requested_full_path = (type == shaderc_include_type_relative)
        ? _file_finder->FindRelativeReadableFilepath( requesting_source, requested_source )
        : _file_finder->FindReadableFilepath( requested_source );
    std::ifstream ifs( requested_full_path );
    std::string source( std::istreambuf_iterator<char>{ ifs }, std::istreambuf_iterator<char>{} );
    auto* container = new std::array<std::string, 2>;
    container->at( 0 ) = requested_source;
    container->at( 1 ) = source;
    auto* data = new shaderc_include_result;
    data->user_data = container;
    data->source_name = container->at( 0 ).data();
    data->source_name_length = container->at( 0 ).size();
    data->content = container->at( 1 ).data();
    data->content_length = container->at( 1 ).size();
    return data;
}

void ShaderIncluder::ReleaseInclude( shaderc_include_result* data )
{
    delete static_cast<std::array<std::string, 2>*>(data->user_data);
    delete data;
}
