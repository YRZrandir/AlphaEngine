#include "ECS.h"

std::unique_ptr<EntityManager> EntityManager::_instance = nullptr;

std::vector<Behavior*> EntityManager::GetBehaviors() const
{
    std::vector<Behavior*> results;
    for (const auto& [tid, components] : _components)
    {
        if (components.empty())
            continue;
        if (dynamic_cast<Behavior*>(components.begin()->second))
        {
            for (const auto& [ent, component] : components)
            {
                results.push_back( dynamic_cast<Behavior*>(component) );
            }
        }
    }
    return results;
}

void BehaviorSystem::Start()
{
}

void BehaviorSystem::Update()
{
    for (auto behavior : EntityManager::Get().GetBehaviors())
    {
        behavior->Update();
    }
}

void BehaviorSystem::OnPreRender()
{
    for (auto behavior : EntityManager::Get().GetBehaviors())
    {
        behavior->OnPreRender();
    }
}

void BehaviorSystem::OnRender()
{
    for (auto behavior : EntityManager::Get().GetBehaviors())
    {
        behavior->OnRender();
    }
}

void BehaviorSystem::OnPostRender()
{
    for (auto behavior : EntityManager::Get().GetBehaviors())
    {
        behavior->OnPostRender();
    }
}
