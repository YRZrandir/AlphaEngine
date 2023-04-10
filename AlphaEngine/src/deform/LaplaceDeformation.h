#pragma once
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../model/HalfEdgeMesh.h"

class LaplaceDeformer
{
public:
    LaplaceDeformer();
    void SetMesh(std::shared_ptr<HalfEdgeMesh> mesh);
    void SetNewPoints(std::vector<glm::vec3>& new_points);
    std::vector<glm::vec3> Transform(const std::vector<int>& anchors);
private:
    std::shared_ptr<HalfEdgeMesh> mMesh;
    std::vector<glm::vec3> mNewPoints;
    Eigen::MatrixXd mPositions;

};