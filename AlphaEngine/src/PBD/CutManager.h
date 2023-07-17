#pragma once
#include <optional>
#include "MetaballModel.h"
#include "../model/HalfEdgeMesh.h"
#include "ScalpelRect.h"

class CutManager
{
public:
    class IntersectInfo
    {
    public:
        enum class Type { TriSplit, EdgeSplit };

        class Hash
        {
        public:
            size_t operator()( const IntersectInfo& info ) const
            {
                return std::hash<int>()(info.iv0) ^ std::hash<int>()(info.iv1);
            }
        };

        class Pred
        {
        public:
            bool operator()( const IntersectInfo& info1, const IntersectInfo& info2 ) const
            {
                return ((info1.iv0 == info2.iv0 && info1.iv1 == info2.iv1) ||
                    (info1.iv0 == info2.iv1 && info1.iv1 == info2.iv0));
            }
        };

        IntersectInfo() = default;

        IntersectInfo( int iv0, int iv1, glm::vec3 p, Type type )
            :iv0( iv0 ), iv1( iv1 ), p( p ), type( type )
        {
        }

        Type type;
        int iv0 = -1; // edge idx when type == EdgeSplit, face idx when type == TriSplit
        int iv1 = -1;
        glm::vec3 p;
    };

    CutManager( HalfEdgeMesh* surface, PBD::ScalpelRect* scalpel );
    HalfEdgeMesh* GetSurface();
    PBD::ScalpelRect* GetScalpel();
    void CutMesh();

private:
    HalfEdgeMesh* _surface;
    HalfEdgeSurfaceTester _surface_tester;
    PBD::ScalpelRect* _scalpel;

    std::unordered_set<IntersectInfo, CutManager::IntersectInfo::Hash, CutManager::IntersectInfo::Pred> _split_infos;
};

