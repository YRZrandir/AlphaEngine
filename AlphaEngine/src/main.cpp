#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "application/ElasticityApp.h"

int main( int argc, char* argv[] )
{
    std::unique_ptr<ElasticityApp> app = std::make_unique<ElasticityApp>( "Elasticity", 1000, 600 );
    app->Start();
    return 0;
}
