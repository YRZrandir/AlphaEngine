#pragma once
#include <glm/glm.hpp>

class AABB
{
public:
	glm::vec3 max_corner;
	glm::vec3 min_corner;

	AABB();

	void Expand(glm::vec3 p);

	unsigned int LongestAxis() const;

	glm::vec3 GetCenter() const;

	bool Intersect(glm::vec3 point) const;
};