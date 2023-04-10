#pragma once
#include "MetaballModel.h"
#include "ScalpelRect.h"
#include "CutManager.h"

namespace PBD
{
    class MetaballCutter
    {
    public:
        MetaballCutter( PBD::MetaballModel* model, PBD::ScalpelRect* scalpel );
        void ProcessOneStepCutRect( Rect& rect );
        void ProcessTotalCutRect( const Rect& rect );

    private:
        std::vector<PBD::MeshlessPoint> SamplePointsInBall( glm::vec3 c, float r, float step = 0.01f );
        std::vector<glm::vec3> SamplePointsOnRemovedBallSurface();
        std::vector<glm::vec3> SamplePointsOnRect( const Rect& rect, float step = 0.01f );

        PBD::MetaballModel* _model;
        PBD::ScalpelRect* _scalpel;
        std::vector<int>       _removed_balls;
        std::vector<glm::vec3> _scalpel_point_history;
    };

}

