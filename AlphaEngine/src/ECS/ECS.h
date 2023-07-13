#pragma once
#include <iterator>
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <typeindex>
#include <ranges>
#include "ECS.h"

class Component;

class Entity
{
public:
    Entity() = delete;

protected:
    friend class EntityManager;
    Entity( unsigned id ) : _id( id )
    {
    }
    unsigned _id{ 0 };
    std::unordered_map<std::type_index, std::unique_ptr<Component>> _components;
};


template <typename ComponentType>
class ComponentIterator
{
public:
    using difference_type = std::ptrdiff_t;
    using value_type = ComponentType;
    using pointer = ComponentType*;
    using reference = ComponentType;
    using iterator_category = std::input_iterator_tag;
    using map_it = std::unordered_map<unsigned, ComponentType*>::iterator;
    map_it _it;

    ComponentIterator( const map_it& it ) : _it( it ) {}
    ComponentIterator( ComponentIterator&& ) = default;
    ComponentIterator( const ComponentIterator& ) = default;
    ComponentIterator& operator=( ComponentIterator&& ) = default;
    ComponentIterator& operator=( const ComponentIterator& ) = default;

    ComponentType& operator*() const
    {
        return *(_it->second);
    }

    ComponentType* operator->() const
    {
        return _it->second;
    }

    ComponentIterator operator++()
    {
        _it++;
        return *this;
    }

    ComponentIterator operator++( int )
    {
        auto temp = *this;
        ++(*this);
        return temp;
    }

    friend bool operator==( const ComponentIterator& lhs, const ComponentIterator& rhs )
    {
        return lhs._it == rhs._it;
    }

    friend bool operator!=( const ComponentIterator& lhs, const ComponentIterator& rhs )
    {
        return lhs._it != rhs._it;
    }
};

template <typename ComponentType>
class ComponentRange
{
public:
    ComponentRange( std::unordered_map<unsigned, ComponentType*>& container ) : _it0( container.begin() ), _it1( container.end() )
    {

    }

    ComponentIterator<ComponentType> begin()
    {
        return _it0;
    }

    ComponentIterator<ComponentType> end()
    {
        return _it1;
    }

protected:
    ComponentIterator<ComponentType> _it0;
    ComponentIterator<ComponentType> _it1;
};


class EntityManager
{
protected:
    EntityManager() = default;
    static std::unique_ptr<EntityManager> _instance;
public:
    static EntityManager& Get()
    {
        if (_instance == nullptr)
        {
            _instance = std::unique_ptr<EntityManager>( new EntityManager() );
        }
        return *_instance;
    }

    template <typename ComponentType, typename...P0toN>
    void AddComponent( unsigned entity, P0toN&&...args )
    {
        if (!_entities.contains( entity ))
            return;
        auto tid = std::type_index( typeid(ComponentType) );
        //ugly. but have to pass the entity id to the component. Is there another way?
        auto it = _entities.at( entity )._components.insert( { tid, std::make_unique<ComponentType>( std::forward<P0toN>( args )... ) } );
        it.first->second->SetEntity( entity );
        it.first->second->Start();
        _components[tid].insert( { entity, it.first->second.get() } );
    }

    template <typename ComponentType>
    ComponentType* GetComponent( unsigned entity ) const
    {
        auto tid = std::type_index( typeid(ComponentType) );
        if (!_components.contains( tid ))
            return nullptr;
        if (!_components.at( tid ).contains( entity ))
            return nullptr;
        return static_cast<ComponentType*>(_components.at( tid ).at( entity ));
    }

    unsigned AddEntity()
    {
        _entities.insert( { uuid, Entity( uuid ) } );
        return uuid++;
    }

    unsigned AddEntity( std::string name )
    {
        unsigned new_ent_id = uuid;
        uuid++;
        _entities.insert( { new_ent_id, Entity( new_ent_id ) } );
        _entity_readable_names.insert( { new_ent_id, std::move( name ) } );
        return new_ent_id;
    }

    void SetEntityName( unsigned ent, std::string name )
    {
        if (_entities.contains( ent ))
        {
            _entity_readable_names.insert( { ent, std::move( name ) } );
        }
    }

    std::string GetEntityName( unsigned ent ) const
    {
        if (_entity_readable_names.contains( ent ))
        {
            return _entity_readable_names.at( ent );
        }
        return std::string();
    }

    template <typename ComponentType>
    void RemoveComponent( unsigned entity )
    {
        auto tid = std::type_index( typeid(ComponentType) );
        if (_components.contains( tid ))
        {
            _components[tid].erase( entity );
            entity->_components.erase( tid );
        }
    }

    template <typename ComponentType>
    bool HasComponent( unsigned entity ) const
    {
        auto tid = std::type_index( typeid(ComponentType) );
        return entity->_components.contains( tid );
    }

    template <typename ComponentType>
    std::vector<ComponentType*> GetAllComponentOfType() const
    {
        auto tid = std::type_index( typeid(ComponentType) );
        if (!_components.contains( tid ))
            return {};
        std::vector<ComponentType*> results;
        for (auto& [ent, comp] : _components.at( tid ))
        {
            results.push_back( static_cast<ComponentType*>(comp) );
        }
        return results;
    }

    template <typename ComponentType>
    ComponentRange<ComponentType> GetComponentsRange()
    {
        auto tid = std::type_index( typeid(ComponentType) );
        return ComponentRange<ComponentType>( _components[tid] );
    }

    template <typename ComponentType>
    std::vector<unsigned> GetAllEntitiesWith() const
    {
        auto tid = std::type_index( typeid(ComponentType) );
        std::vector<unsigned> results;
        if (!_components.contains( tid ))
            return results;
        for (auto& [ent, comp] : _components[tid])
        {
            results.push_back( ent );
        }
        return results;
    }

    void EraseEntity( unsigned ent )
    {
        for (auto& [ctype, ctable] : _components)
        {
            ctable.erase( ent );
        }
        _entity_readable_names.erase( ent );
        _entities.erase( ent );
    }

protected:
    std::unordered_map<std::type_index, std::unordered_map<unsigned, Component*>> _components;
    std::unordered_map<unsigned, Entity> _entities;
    std::unordered_map<unsigned, std::string> _entity_readable_names;
    unsigned uuid{ 1 }; // zero is preserved.
};

class Component
{
public:
    virtual ~Component() = default;

    unsigned GetEntity() const noexcept
    {
        return _ent;
    }

    template<typename ComponentType>
    ComponentType* GetComponent()
    {
        if (_ent == 0)
        {
            std::cout << "ERR: Component doesn't belong to an entity!" << std::endl;
            return nullptr;
        }
        return EntityManager::Get().GetComponent<ComponentType>( _ent );
    }

protected:
    friend class EntityManager;
    Component() = default;
    virtual void Start() {};
    void SetEntity( unsigned entity ) { _ent = entity; }

private:
    unsigned _ent{ 0 };
};

class System
{
public:
    virtual ~System() = default;
    virtual void Start() {};
    virtual void Update() {};
    virtual void OnPreRender() {};
    virtual void OnRender() {};
    virtual void OnPostRender() {};
};