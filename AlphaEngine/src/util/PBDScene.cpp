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
}

void PBDScene::Update()
{
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
            for (int i = 0; i < 3; i++)
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
                    model->CollisionDetection();
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
            for (auto model : pdfcmodels)
                if (model->_simulate)
                    model->CollisionDetection();
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
                    model->CollisionDetection();
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
