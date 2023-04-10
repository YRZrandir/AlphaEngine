#pragma once
#include <glm/glm.hpp>

class Ray
{
public:
	glm::vec3 o;
	glm::vec3 d;

	Ray() = default;

	Ray(glm::vec3 o, glm::vec3 d)
		:o(o), d(glm::normalize(d))
	{}

	glm::vec3 at(float t) const
	{
		return o + d * t;
	}
};

class IntersectionRec
{
public:
	glm::vec3 p;
	glm::vec3 normal;
	float t;

	IntersectionRec() = default;

	IntersectionRec(glm::vec3 p, glm::vec3 normal, float t)
		:p(p), normal(glm::normalize(normal)), t(t)
	{}
};