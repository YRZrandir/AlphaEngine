#include "Hand.h"
#include "material/Material.h"
#include "model/HalfEdgeMesh.h"
#include "input/Input.h"

Hand::Hand()
{
    _hand1 = std::make_unique<HalfEdgeMesh>( "res/models/hand1.obj" );
    _hand2 = std::make_unique<HalfEdgeMesh>( "res/models/hand2.obj" );
    _hand1->Normalize();
    _hand2->Normalize();
    _hand1->UpdatePosBuffer();
    _hand2->UpdatePosBuffer();
    mTransform.SetScale( glm::vec3( 0.2f ) );
    _hand1->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand2->mRenderConfig._face_mattype = RenderConfig::MatType::PlainColor;
    _hand1->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand2->_material_main->mDiffuseColor = glm::vec3( 0.5, 0.5, 1.0 );
    _hand1->_material_main->mAlpha = 0.5f;
    _hand2->_material_main->mAlpha = 0.5f;
}

void Hand::Update()
{

}

void Hand::Draw()
{
    if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
    {
        _hand2->mTransform = mTransform;
        _hand2->Draw();
    }
    else
    {
        _hand1->mTransform = mTransform;
        _hand1->Draw();
    }
}
