#include "RayIntersectSolver.h"
#include <algorithm>
#include "../gl/ShaderStorageBuffer.h"
#include "../util/Shader.h"
#include "../model/Triangle.h"
#include "../model/Ray.h"

RayIntersectSolver::RayIntersectSolver( bool half_edge_mesh )
{
    if (half_edge_mesh)
    {
        mComputeShader = Shader::Find( "half_edge_ray_solver" );
    }
    else
    {
        mComputeShader = Shader::Find( "ray_solver" );
    }
    mResultSSBO = std::make_unique<ShaderStorageBuffer>( nullptr, 0 );

    glCreateBuffers( 1, &mAtomicCounterBuffer );
    glBindBuffer( GL_ATOMIC_COUNTER_BUFFER, mAtomicCounter );
    glNamedBufferStorage( mAtomicCounterBuffer, 1 * sizeof( unsigned int ), &mAtomicCounter, GL_DYNAMIC_STORAGE_BIT );
}

void RayIntersectSolver::DispatchCompute( const Ray& ray, unsigned int& id, bool& intersect, int* intersect_count )
{
    mComputeShader->use();
    mVertexSSBO->BindBufferBase( 0 );
    mTriangleSSBO->BindBufferBase( 1 );
    mResultSSBO->BindBufferBase( 2 );
    mAtomicCounter = 0;
    glNamedBufferSubData( mAtomicCounterBuffer, 0, sizeof( unsigned int ), &mAtomicCounter );
    glBindBufferBase( GL_ATOMIC_COUNTER_BUFFER, 0, mAtomicCounterBuffer );

    unsigned int triangleNum = mTriangleSSBO->GetSize() / sizeof( Triangle );
    mComputeShader->setUnsignedInt( "uTriangleNum", triangleNum );
    mComputeShader->setVec( "uRayOri", ray.o );
    mComputeShader->setVec( "uRayDir", ray.d );
    size_t num_group_x = static_cast<size_t>(std::ceilf( static_cast<float>(triangleNum) / 128 ));
    glDispatchCompute( num_group_x, 1, 1 );
    glGetNamedBufferSubData( mAtomicCounterBuffer, 0, sizeof( unsigned int ), &mAtomicCounter );
    mResults.resize( mAtomicCounter );
    glGetNamedBufferSubData( mResultSSBO->GetID(), 0, mAtomicCounter * sizeof( RayIntersectSolver::PressSolverResult ), mResults.data() );

    if (mResults.empty())
    {
        intersect = false;
    }
    else
    {
        intersect = true;
        id = std::min_element( std::cbegin( mResults ), std::cend( mResults ), []( const auto& left, const auto& right )->bool
            {
                return left.t < right.t;
            } )->id;
        if (intersect_count)
        {
            *intersect_count = mResults.size();
        }
    }
}

void RayIntersectSolver::UpdateData()
{
    mResultSSBO->UpdateData( nullptr, mTriangleSSBO->GetSize() / sizeof( Triangle ) * sizeof( PressSolverResult ) );
}

void RayIntersectSolver::SetVertexDataBuffer( std::shared_ptr<ShaderStorageBuffer> vertexDataBuffer )
{
    mVertexSSBO = vertexDataBuffer;
}

std::shared_ptr<ShaderStorageBuffer> RayIntersectSolver::GetVertexDataBuffer() const
{
    return mVertexSSBO;
}

void RayIntersectSolver::SetTriangleDataBuffer( std::shared_ptr<ShaderStorageBuffer> triangleDataBuffer )
{
    mTriangleSSBO = triangleDataBuffer;
}

std::shared_ptr<ShaderStorageBuffer> RayIntersectSolver::GetTriangleDataBuffer() const
{
    return mTriangleSSBO;
}

unsigned int RayIntersectSolver::GetResultCount() const
{
    return mAtomicCounter;
}
