#include "HalfEdgeMeshRendererSystem.h"
#include "gl/HalfEdgeMeshRenderer.h"

HalfEdgeMeshRendererSystem::HalfEdgeMeshRendererSystem()
{
}

void HalfEdgeMeshRendererSystem::OnPreRender()
{
    for (auto& mesh_renderer : EntityManager::Get().GetAllComponentOfType<HalfEdgeMeshRenderer>())
    {
        mesh_renderer->RenderShadowDepth();
    }
}

void HalfEdgeMeshRendererSystem::OnRender()
{
    for (auto& mesh_renderer : EntityManager::Get().GetAllComponentOfType<HalfEdgeMeshRenderer>())
    {
        mesh_renderer->Render();
    }
}
