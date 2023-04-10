#pragma once
#include <memory>
#include <vector>
#include <glm/glm.hpp>

class Shader;
class SingleMesh;
class PressingTool;
class ShaderStorageBuffer;
class Ray;

class RayIntersectSolver
{
private:
    Shader* mComputeShader = nullptr;
    std::shared_ptr<ShaderStorageBuffer> mVertexSSBO = nullptr;
    std::shared_ptr<ShaderStorageBuffer> mTriangleSSBO = nullptr;
    std::unique_ptr<ShaderStorageBuffer> mResultSSBO = nullptr;

    unsigned int mAtomicCounter = 0;
    unsigned int mAtomicCounterBuffer = 0;
    class PressSolverResult
    {
    public:
        unsigned int id;
        float t;
    };
    std::vector<PressSolverResult> mResults;

public:
    RayIntersectSolver( bool half_edge_mesh = false );
    ~RayIntersectSolver() = default;

    void DispatchCompute( const Ray& ray, unsigned int& id, bool& intersect, int* intersect_count = nullptr );

    void UpdateData();

    void				SetVertexDataBuffer( std::shared_ptr<ShaderStorageBuffer> vertexDataBuffer );
    std::shared_ptr<ShaderStorageBuffer>	GetVertexDataBuffer() const;

    void				SetTriangleDataBuffer( std::shared_ptr<ShaderStorageBuffer> triangleDataBuffer );
    std::shared_ptr<ShaderStorageBuffer>	GetTriangleDataBuffer() const;

    unsigned int GetResultCount() const;
};

