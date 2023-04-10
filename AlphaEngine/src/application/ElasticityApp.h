#pragma once
#include "ApplicationBase.h"
#include <sstream>
#include <tinyxml2.h>

namespace PD
{
class PDGPUMetaballModel;
class PDMetaballModel;
}
class HalfEdgeMesh;
class PDMetaballHalfEdgeMesh;

class ElasticityApp : public ApplicationBase
{
public:
    ElasticityApp( std::string title, size_t width, size_t height );
    virtual ~ElasticityApp();
protected:
    virtual void Init() override;
    virtual void PreDraw() override;
    virtual void PostDraw() override;
    virtual void DrawGUI() override;
    virtual void DrawGraphics() override;
    PD::PDGPUMetaballModel* LoadPDGPUMetaballModel( tinyxml2::XMLElement* root );
    PD::PDMetaballModel* LoadPDMetaballModel( tinyxml2::XMLElement* root );
    HalfEdgeMesh* LoadMesh( tinyxml2::XMLElement* root );
    PDMetaballHalfEdgeMesh* LoadPDMetaballHalfEdgeMesh( tinyxml2::XMLElement* root );
    void LoadSceneFile( const char* filename );
};