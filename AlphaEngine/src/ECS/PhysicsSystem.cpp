#include "PhysicsSystem.h"
#include <omp.h>
#include "PD/PDMetaballModelFC.h"

PhysicsSystem::PhysicsSystem()
{
    _spatial_hash = std::make_unique<PD::SpatialHash>( 0.1 );
}

void PhysicsSystem::Update()
{
    auto pd_metaball_fc_models = EntityManager::Get().GetAllComponentOfType<PD::PDMetaballModelFC>();
    for (int i = 0; i < _substeps; i++)
    {
#ifdef MT
#pragma omp parallel for
#endif
        for (int j = 0; j < pd_metaball_fc_models.size(); j++)
        {
            auto model = pd_metaball_fc_models[j];
            if (model->_simulate)
            {
                model->UpdateSn();
            }
        }

        _spatial_hash->Clear();
        for (int j = 0; j < pd_metaball_fc_models.size(); j++)
        {
            auto model = pd_metaball_fc_models[j];
            if (model->_simulate)
            {
                for (int k = 0; k < model->GetMetaballModel().BallsNum(); k++)
                {
                    _spatial_hash->Insert( &model->GetMetaballModel().Ball( k ) );
                }
            }
        }

#ifdef MT
#pragma omp parallel for
#endif
        for (int j = 0; j < pd_metaball_fc_models.size(); j++)
        {
            auto model = pd_metaball_fc_models[j];
            if (model->_simulate)
            {
                model->CollisionDetection( _spatial_hash.get() );
            }
        }

#ifdef MT
#pragma omp parallel for
#endif
        for (int j = 0; j < pd_metaball_fc_models.size(); j++)
        {
            auto model = pd_metaball_fc_models[j];
            if (model->_simulate)
            {
                model->PDSolve();
            }
        }

#ifdef MT
#pragma omp parallel for
#endif
        for (int j = 0; j < pd_metaball_fc_models.size(); j++)
        {
            auto model = pd_metaball_fc_models[j];
            if (model->_simulate)
            {
                model->PostPhysicalUpdate();
            }
        }

    }
}