#include "LaplaceDeformation.h"
#include <chrono>
#include <iostream>
#include <Eigen/IterativeLinearSolvers>

LaplaceDeformer::LaplaceDeformer()
{
    
}

void LaplaceDeformer::SetMesh(std::shared_ptr<HalfEdgeMesh> mesh)
{
    mMesh = mesh;
    mPositions.resize(mMesh->_vertices.size(), 3);
}

void LaplaceDeformer::SetNewPoints(std::vector<glm::vec3>& new_points)
{
    mNewPoints = new_points;
}

std::vector<glm::vec3> LaplaceDeformer::Transform(const std::vector<int>& anchors)
{
    const size_t point_num = mMesh->_vertices.size();
    Eigen::SparseMatrix<double> Ls(point_num + anchors.size(), point_num);
    std::vector<Eigen::Triplet<double>> triplets_Ls;

    std::cout << "Building Ls" << std::endl;
    for (size_t i = 0; i < point_num; i++)
    {
        int degree = mMesh->GetNeighborVertices(i).size();
        triplets_Ls.push_back(Eigen::Triplet<double>(i, i, (double)degree));
    }
    for (const auto& pair : mMesh->_edge_map)
    {
        auto idx_pair = pair.first;
        triplets_Ls.push_back(Eigen::Triplet<double>(idx_pair.first, idx_pair.second, -1.0f));
    }
    for (int i = point_num; i < point_num + anchors.size(); i++)
    {
        int anchor = anchors[i - point_num];
        triplets_Ls.push_back(Eigen::Triplet<double>(i, anchor, 1.0));
    }
    Ls.setFromTriplets(triplets_Ls.cbegin(), triplets_Ls.cend());

    std::cout << "Building b" << std::endl;
    Eigen::MatrixXd b(point_num + anchors.size(), 3);
    for (size_t i = 0; i < point_num; i++)
    {
        glm::vec3 pos = mMesh->DiffCoord(i);
        b(i, 0) = pos.x;
        b(i, 1) = pos.y;
        b(i, 2) = pos.z;
    }
    for (size_t i = point_num; i < point_num + anchors.size(); i++)
    {
        int anchor = anchors[i - point_num];
        b(i, 0) = mNewPoints[anchor].x;
        b(i, 1) = mNewPoints[anchor].y;
        b(i, 2) = mNewPoints[anchor].z;
    }
 
    std::cout << "Building Complete" << std::endl;
    std::cout << "Size: " << point_num + anchors.size() << std::endl;
    Eigen::SparseMatrix<double> LsT = Ls.transpose();
    Eigen::SparseMatrix<double> mat = LsT * Ls;

    std::cout << "Solving..." << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(mat);
    Eigen::MatrixXd X = chol.solve(LsT * b);

    auto cost = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t1)).count();
    std::cout << "Solving Complete, time=" << cost << "ms" << std::endl;

    std::vector<glm::vec3> result;
    for (size_t i = 0; i < point_num; i++)
    {
        glm::vec3 p(X(i, 0), X(i, 1), X(i, 2));
        result.push_back(p);
    }

    return result;
}
