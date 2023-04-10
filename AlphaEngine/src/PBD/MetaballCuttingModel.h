#pragma once
#include "../util/SceneObject.h"
#include "MetaballModel.h"
#include "MetaballCutter.h"
#include "../model/HalfEdgeMesh.h"
#include "CutManager.h"

namespace PBD
{
    class MetaballCuttingModel :
        public SceneObject
    {
    public:
        MetaballCuttingModel( MetaballModel* metaball_model, HalfEdgeMesh* surface, ScalpelRect* scalpel );
        virtual void Update() override;
        virtual void Draw() override;
        void SetEnable( bool value );
        bool IsEnable() const;
    private:
        void RecalcSurfaceX0();

        MetaballModel* _metaball_model = nullptr;
        HalfEdgeMesh* _surface = nullptr;
        ScalpelRect* _scalpel = nullptr;
        std::unique_ptr<MetaballCutter> _metaball_cutter = nullptr;
        std::unique_ptr<CutManager> _surface_cutter = nullptr;
        bool _enabled = false;
    };

}

