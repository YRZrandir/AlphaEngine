#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

class Transform
{
private:
	glm::vec3 mPos;
	glm::vec3 mScale;
	glm::quat mRotation;
	glm::mat4 mModelMat;

public:
	Transform();
	Transform(glm::vec3 pos, glm::vec3 scale, glm::quat rotation);

	void SetPos(glm::vec3 pos);
	void SetScale(glm::vec3 scale = glm::vec3(1.f, 1.f, 1.f));
	void SetRotation(glm::quat rotation = glm::quat());
	void SetRotation(glm::vec3 eulerRadians);
	void SetYawPitchRow(float yaw_rd, float pitch_rd, float row_rd);

	void Translate(glm::vec3 move);
	void Scale(glm::vec3 scale);
	void Scale(float scale);
	void Rotate(glm::quat rotation);
	void Rotate(glm::vec3 axis, float degree);
	void Rotate(float x_rad, float y_rad, float z_rad);
	void Rotate(glm::mat4 rotation);
	void Rotate(glm::vec3 eulerRadians);
	void RotateAround(glm::vec3 point, glm::vec3 axis, float radian);
	void LookAt(glm::vec3 target);
	void LookAt(glm::vec3 target, glm::vec3 worldup);
	void UpdateModelMat();

	glm::mat4 GetModelMat() const;
	glm::vec3 Forward() const;
	glm::vec3 Up() const;
	glm::vec3 Left() const;
	glm::mat4 GetRotationMat() const;
	glm::quat GetRotation() const;
	glm::vec3 GetEulerAngleRadian() const;
	glm::mat4 GetPositionMat() const;
	glm::vec3 GetPosition() const;
	glm::mat4 GetScaleMat() const;
	glm::vec3 GetScale() const;
};