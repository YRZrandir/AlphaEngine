#include "ModelLoader.h"

#include <fstream>
#include <iostream>
#include <string>
#include <iterator>
#include <filesystem>
#include "PBD/MetaballModel.h"

std::vector<std::pair<glm::vec3, float>> ModelLoader::LoadSphereList( const std::string& path )
{
    std::ifstream ifs( path, std::ios_base::in );
    std::string first_line;
    std::getline( ifs, first_line );
    std::stringstream ss( first_line );

    int depth = 0;
    int degree = 0;
    ss >> depth >> degree;

    if (depth <= 0 || degree <= 0)
    {
        throw std::runtime_error( "File format incorrect" );
    }

    int node_num = 0;
    for (size_t i = 0; i < depth; i++)
    {
        node_num += (int)glm::pow( degree, i );
    }

    std::vector<PBD::Metaball> metaballs;
    for (size_t i = 0; i < node_num; i++)
    {
        std::string line;
        std::getline( ifs, line );
        std::stringstream ss( line );
        float x, y, z, r;
        ss >> x >> y >> z >> r;
        metaballs.push_back( PBD::Metaball( glm::vec3( x, y, z ), r ) );
    }

    PBD::MetaballTreeNode root( metaballs, 0 );
    metaballs.clear();
    root.GetLeafNodes( metaballs );

    std::unordered_set<PBD::Metaball, PBD::Metaball::Hash, PBD::Metaball::Pred> ballset;
    ballset.insert( metaballs.begin(), metaballs.end() );
    metaballs = std::vector<PBD::Metaball>( ballset.begin(), ballset.end() );

    std::vector<std::pair<glm::vec3, float>> spheres;
    for (auto& ball : ballset)
    {
        spheres.push_back( { ball.x, ball.r } );
    }

    return spheres;
}