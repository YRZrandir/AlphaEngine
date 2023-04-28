#include "PBDScene.h"
#include <thread>
#include <omp.h>
#include "util/GlobalTimer.h"
#include "util/Instrumentor.h"
#include "PD/PDGPUMetaballModel.h"
#include "PD/PDMetaballModelFC.h"
#include "PBD/MetaballModel.h"
#include "Camera.h"


PBDScene::PBDScene( bool mt )
    :Scene(), _mt( mt )
{
    _spatial_hash = std::make_unique<PD::SpatialHash>( 0.1f );
}

void PBDScene::Update()
{
    using hrc = std::chrono::high_resolution_clock;

    if (_mt)
    {
        std::vector<PBD::MetaballModel*> models = GetAllChildOfType<PBD::MetaballModel>();
        if (!models.empty())
        {
            int iteration_num = 2;
            float interval = GlobalTimer::RecentAvgFrameTimeMs() / 1000.f / iteration_num;
            int block_num = models.size();
            for (int i = 0; i < iteration_num; i++)
            {
                auto start = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
                for (int j = 0; j < models.size(); j++)
                {
                    models[j]->PBDPrediction( interval );
                }
#pragma omp parallel for
                for (int j = 0; j < models.size(); j++)
                {
                    models[j]->PBDCheckCollision( interval );
                }
#pragma omp parallel for
                for (int j = 0; j < models.size(); j++)
                {
                    models[j]->PBDSolveConstraints( interval );
                }
            }
        }

        std::vector<PD::PDMetaballModel*> pdmodels = GetAllChildOfType<PD::PDMetaballModel>();
        if (!pdmodels.empty())
        {
            for (int i = 0; i < 10; i++)
            {
#pragma omp parallel for
                for (int j = 0; j < pdmodels.size(); j++)
                {
                    if (pdmodels[j]->_simulate)
                        pdmodels[j]->PhysicalUpdate();
                }
#pragma omp parallel for
                for (int j = 0; j < pdmodels.size(); j++)
                {
                    if (pdmodels[j]->_simulate)
                        pdmodels[j]->CollisionDetection();
                }
#pragma omp parallel for
                for (int j = 0; j < pdmodels.size(); j++)
                {
                    if (pdmodels[j]->_simulate)
                        pdmodels[j]->PostPhysicalUpdate();
                }
            }
        }

        std::vector<PD::PDGPUMetaballModel*> gpupdmodels = GetAllChildOfType<PD::PDGPUMetaballModel>();
        if (!gpupdmodels.empty())
        {
            for (int i = 0; i < 1; i++)
            {
#pragma omp parallel for
                for (int j = 0; j < gpupdmodels.size(); j++)
                {
                    if (gpupdmodels[j]->_simulate)
                    {
                        gpupdmodels[j]->CudaPhysicalUpdate();
                    }
                }
#pragma omp parallel for
                for (int j = 0; j < gpupdmodels.size(); j++)
                {
                    if (gpupdmodels[j]->_simulate)
                    {
                        gpupdmodels[j]->CollisionDetection();
                    }
                }
#pragma omp parallel for
                for (int j = 0; j < gpupdmodels.size(); j++)
                {
                    if (gpupdmodels[j]->_simulate)
                    {
                        gpupdmodels[j]->PostPhysicalUpdate();
                    }
                }
            }
        }

        std::vector<PD::PDMetaballModelFC*> pdfcmodels = GetAllChildOfType<PD::PDMetaballModelFC>();
        if (!pdfcmodels.empty())
        {
            for (int i = 0; i < 2; i++)
            {
                auto t0 = hrc::now();
#pragma omp parallel for
                for (int j = 0; j < pdfcmodels.size(); j++)
                {
                    if (pdfcmodels[j]->_simulate)
                        pdfcmodels[j]->UpdateSn();
                }
                auto d0 = hrc::now() - t0;

                auto t1 = hrc::now();
                _spatial_hash->Clear();
                for (auto model : pdfcmodels)
                {
                    if (model->_simulate)
                    {
                        for (int j = 0; j < model->_mesh->BallsNum(); j++)
                        {
                            _spatial_hash->Insert( &model->_mesh->Ball( j ) );
                        }
                    }
                }
#pragma omp parallel for
                for (int j = 0; j < pdfcmodels.size(); j++)
                {
                    if (pdfcmodels[j]->_simulate)
                        pdfcmodels[j]->CollisionDetection( _spatial_hash.get() );
                }
                auto d1 = hrc::now() - t1;

                auto t2 = hrc::now();
#pragma omp parallel for
                for (int j = 0; j < pdfcmodels.size(); j++)
                    if (pdfcmodels[j]->_simulate)
                        pdfcmodels[j]->PDSolve();
#pragma omp parallel for
                for (int j = 0; j < pdfcmodels.size(); j++)
                {
                    if (pdfcmodels[j]->_simulate)
                        pdfcmodels[j]->PostPhysicalUpdate();
                }
                auto d2 = hrc::now() - t2;

                std::cout << "Simulation: " << (float)(std::chrono::duration_cast<std::chrono::microseconds>(d0).count() + std::chrono::duration_cast<std::chrono::microseconds>(d2).count()) / 1000.f << std::endl;
                std::cout << "CD: " << (float)std::chrono::duration_cast<std::chrono::microseconds>(d1).count() / 1000.f << std::endl;
            }
        }
    }
    else
    {
        std::vector<PBD::MetaballModel*> models = GetAllChildOfType<PBD::MetaballModel>();
        if (!models.empty())
        {
            int iteration_num = 10;
            float interval = 30.f / 1000.f / iteration_num;
            for (int i = 0; i < iteration_num; i++)
            {
                for (auto model : models)
                {
                    model->PBDPrediction( interval );
                    model->PBDCheckCollision( interval );
                    model->PBDSolveConstraints( interval );
                }
            }
        }

        std::vector<PD::PDMetaballModel*> pdmodels = GetAllChildOfType<PD::PDMetaballModel>();
        for (int i = 0; i < 1; i++)
        {
            for (auto model : pdmodels)
            {
                if (model->_simulate)
                {
                    model->PhysicalUpdate();
                    //model->CollisionDetection();
                    model->PostPhysicalUpdate();
                }
            }
        }

        std::vector<PD::PDMetaballModelFC*> pdfcmodels = GetAllChildOfType<PD::PDMetaballModelFC>();
        for (int i = 0; i < 1; i++)
        {

            for (auto model : pdfcmodels)
                if (model->_simulate)
                    model->UpdateSn();

            _spatial_hash->Clear();
            for (auto model : pdfcmodels)
            {
                if (model->_simulate)
                {
                    for (int j = 0; j < model->_mesh->BallsNum(); j++)
                    {
                        _spatial_hash->Insert( &model->_mesh->Ball( j ) );
                    }
                }
            }

            for (auto model : pdfcmodels)
                if (model->_simulate)
                    model->CollisionDetection( _spatial_hash.get() );
            for (auto model : pdfcmodels)
                if (model->_simulate)
                    model->PDSolve();
            for (auto model : pdfcmodels)
                if (model->_simulate)
                    model->PostPhysicalUpdate();
        }

        std::vector<PD::PDGPUMetaballModel*> gpupdmodels = GetAllChildOfType<PD::PDGPUMetaballModel>();
        for (int i = 0; i < 1; i++)
        {
            for (auto model : gpupdmodels)
            {
                if (model->_simulate)
                {
                    model->CudaPhysicalUpdate();
                }
            }
            for (auto model : gpupdmodels)
            {
                if (model->_simulate)
                {
                    //model->CollisionDetection();
                }
            }
            for (auto model : gpupdmodels)
            {
                if (model->_simulate)
                {
                    model->PostPhysicalUpdate();
                }
            }
        }
    }
    Scene::Update();
}
