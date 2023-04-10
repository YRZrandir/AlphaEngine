#pragma once
#include <memory>
#include <string>
#include <vector>
#include <glm/glm.hpp>

class ModelLoader
{
public:
    static std::vector<std::pair<glm::vec3, float>> LoadSphereList( const std::string& path );
};
