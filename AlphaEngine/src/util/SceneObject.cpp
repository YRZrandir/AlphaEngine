#include "SceneObject.h"

SceneObject::SceneObject( const std::string& name )
    :SceneObject( name, Transform() )
{
}

SceneObject::SceneObject( const std::string& name, Transform transform )
    : mName( name ), mTransform( transform )
{
}

//std::unique_ptr<SceneObject> SceneObject::clone() const
//{
//    return std::unique_ptr<SceneObject>( virtual_clone() );
//}


