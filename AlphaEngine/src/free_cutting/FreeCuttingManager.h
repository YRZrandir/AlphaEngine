#pragma once
#include <memory>
#include <map>
#include <chrono>
#include <list>
#include <vector>
#include <algorithm>
#include <glm/glm.hpp>
#include "../util/SceneObject.h"
#include "../model/HalfEdgeMesh.h"
#include "../PBD/Scalpel.h"
#include "../free_cutting/CuttingFace.h"
#include "./CuttingTool.h"
#include "../deform/LaplaceDeformation.h"

class VertexArray;
class VertexBuffer;
class FreeCuttingTool;
namespace PBD
{
class MetaballModel;
class PBDTetraModel;
}

class SurfaceCurve :
    public SceneObject
{
public:
    SurfaceCurve();
    void AddPoint( glm::vec3 point );
    void Clear();
    void UpdateGPUMem();
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;
    std::vector<glm::vec3> mPoints;
private:
    std::shared_ptr<VertexArray> mVAO;
    std::shared_ptr<VertexBuffer> mVBO;
};

class FreeCuttingTool :
    public SceneObject
{
public:
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;
private:
    std::shared_ptr<HalfEdgeMesh> mMesh;
};

class RayIntersectRec
{
public:
    glm::vec3 pos;
    int tri_handle;
};

class EdgePlaneIntersectRec
{
public:
    class Hash
    {
    public:
        size_t operator()( const EdgePlaneIntersectRec& pair ) const
        {
            return std::hash<int>()(pair.istart) ^ std::hash<int>()(pair.iend);
        }
    };
    class Pred
    {
    public:
        bool operator()( const EdgePlaneIntersectRec& left, const EdgePlaneIntersectRec& right ) const
        {
            if ((left.istart == right.istart && left.iend == right.iend) ||
                (left.istart == right.iend && left.iend == right.istart))
            {
                return true;
            }
            return false;
        }
    };

public:
    int istart;
    int iend;
    glm::vec3 pos;
    int tri;

    EdgePlaneIntersectRec( int start, int end, glm::vec3 p, int t )
        :istart( start ), iend( end ), pos( p ), tri( t )
    {}
};


class FreeCuttingManager :
    public SceneObject
{
public:
    FreeCuttingManager() = default;
    void Enable();
    void Disable();
    void SetMesh( std::shared_ptr<HalfEdgeMesh> mesh );
    void SetMetaballModel( PBD::MetaballModel* model );
    void SetTetraModel( PBD::PBDTetraModel* model );
    // 通过 SceneObject 继承
    virtual void Update() override;
    virtual void Draw() override;

private:
    void ProcessRay();
    void ProcessIntersection( IntersectionRec rec, int face );
    int SplitTriangleByPoint( glm::vec3 point, int tri );
    void CalcDistField();
    void SplitCutLine();
    void MoveCutTracePairs();
    void BuildCutFace();

private:
    std::shared_ptr<HalfEdgeMesh> mMesh = nullptr;
    PBD::MetaballModel* mMetaballModel = nullptr;
    PBD::PBDTetraModel* mTetModel = nullptr;

    std::list<int> mCutIndices;
    std::list<float> mDistanceField;
    std::list<std::pair<int, int>> mCutTracePairs;
    std::list<glm::vec3> mSplitDir;
    std::list<glm::vec3> mCutDir;
    std::vector<std::pair<int, glm::vec3>> mPostAdjust;
    std::unordered_set<int> mAnchors;

    SurfaceCurve mCurve;
    bool mActive = false;
    bool mCutting = false;
    int mLastFace = -1;
    IntersectionRec mLastRec;
    std::vector<int> mActiveTriangles;
    LaplaceDeformer mLaplaceDeformer;
};
