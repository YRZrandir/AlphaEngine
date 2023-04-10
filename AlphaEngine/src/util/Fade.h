#pragma once
#include <fade/Fade_2D.h>
#include <glm/glm.hpp>
#include "../model/Triangle.h"

//void Triangulation25d( const std::vector<glm::vec3>& points, std::vector<Triangle>* out_triangles, std::vector<glm::vec3>* out_vertices );

void Triangulation2d( const std::vector<glm::vec2>& points, std::vector<Triangle>* out_triangles, std::vector<glm::vec2>* out_vertices );