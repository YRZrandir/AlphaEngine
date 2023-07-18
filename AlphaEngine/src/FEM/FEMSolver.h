#pragma once
#include <mutex>
#include <glm/glm.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <omp.h>
#include <tinycolormap.hpp>
#include "model/TetraMesh.h"
#include "model/HalfEdgeMesh.h"
#include "util/util.h"
#include "util/SceneObject.h"
#include <imgui/imgui.h>
#include "input/Input.h"
#include "util/Camera.h"
#include "util/Intersection.h"
#include "util/MathTypeConverter.h"
struct FEMConfig
{
    std::string path;
    std::string surface_path;
    float density = 1.f;
    float dt = 0.03f;
    int substep = 10;
    float mu = 580.f;
    float lambda = 380.f;
    bool show_tet = false;
    bool show_surface = true;
    bool simulate = false;
};

template <std::floating_point T>
class FEMSolver : public SceneObject
{
public:
    using Matrix3X = Eigen::Matrix3X<T>;
    using Matrix3 = Eigen::Matrix3<T>;
    using Matrix4 = Eigen::Matrix4<T>;
    using Quat = Eigen::Quaternion<T>;
    using VectorX = Eigen::VectorX<T>;
    using Vector4 = Eigen::Vector4<T>;
    using Vector3 = Eigen::Vector3<T>;
    using MatrixX = Eigen::MatrixX<T>;

    FEMSolver( FEMConfig config );
    void PhysicalUpdate();
    void Update();
    void Draw();
    void DrawGUI();

    FEMConfig _config;
    std::unique_ptr<TetraMesh> _mesh;
    std::unique_ptr<HalfEdgeMesh> _surface;
    Matrix3X _x;
    Matrix3X _x0;
    Matrix3X _v;
    Matrix3X _f;
    VectorX  _m;
    VectorX  _w;
    std::vector<Matrix3> _invDm;
    std::vector<Matrix3> _Hs;

private:
    struct SkinInfo
    {
        int tid = -1;
        Vector4 bc;
    };
    void ExtrudeTetMesh( T d );
    void CreateSurfaceMapping();
    void MapToSurface();

    std::vector<SkinInfo> _skininfos;
    int _hold_vertex = -1;
    glm::vec2 _cursor_start_pos;
    glm::vec3 _oripos;
};

template <std::floating_point T>
FEMSolver<T>::FEMSolver( FEMConfig config )
    :_config( config )
{
    _mesh = std::make_unique<TetraMesh>();
    _mesh->CreateFromSurface( config.path );

    //ExtrudeTetMesh( T( 0.005 ) );
    _surface = std::make_unique<HalfEdgeMesh>( config.surface_path );
    _surface->_use_face_normal = true;
    CreateSurfaceMapping();

    int nb_points = _mesh->GetPointNum();
    int nb_tet = _mesh->GetTetraNum();
    _x.resize( 3, nb_points );
    _x0.resize( 3, nb_points );
    _v.resize( 3, nb_points );
    _v.fill( 0.0 );
    _f.resize( 3, nb_points );
    _f.fill( 0.0 );
    _m.resize( nb_points );
    _m.fill( 0.0 );
    _w.resize( nb_tet );
    _w.fill( 0.0 );

    for (int i = 0; i < nb_points; i++)
    {
        _x.col( i ) = ToEigen( glm::vec3( _mesh->mPoints[i] ) ).cast<T>();
        _x0.col( i ) = _x.col( i );
    }

    for (int i = 0; i < nb_tet; i++)
    {
        double vol = std::abs( _mesh->GetTetraVolume( i ) );
        auto& tet = _mesh->mTetras[i];
        _m[tet.a] += vol * _config.density * 0.25;
        _m[tet.b] += vol * _config.density * 0.25;
        _m[tet.c] += vol * _config.density * 0.25;
        _m[tet.d] += vol * _config.density * 0.25;
        _w[i] = vol;
    }

    _invDm.resize( nb_tet );
    for (int i = 0; i < nb_tet; i++)
    {
        Vector3 p0 = _x.col( _mesh->mTetras[i].a );
        Vector3 p1 = _x.col( _mesh->mTetras[i].b );
        Vector3 p2 = _x.col( _mesh->mTetras[i].c );
        Vector3 p3 = _x.col( _mesh->mTetras[i].d );
        Matrix3 Dm;
        Dm.col( 0 ) = p0 - p3;
        Dm.col( 1 ) = p1 - p3;
        Dm.col( 2 ) = p2 - p3;
        _invDm[i] = Dm.inverse();
    }

    _Hs.resize( nb_tet );
}

template <std::floating_point T>
void FEMSolver<T>::PhysicalUpdate()
{
    auto start = std::chrono::high_resolution_clock::now();
    for (auto& p : _mesh->mColors)
        p = glm::vec3( 0.f );

    int nb_tet = _mesh->GetTetraNum();
#pragma omp parallel for 
    for (int i = 0; i < nb_tet; i++)
    {
        const auto& tet = _mesh->mTetras[i];
        int a = tet.a;
        int b = tet.b;
        int c = tet.c;
        int d = tet.d;
        Matrix3 Ds;
        Ds.col( 0 ) = _x.col( a ) - _x.col( d );
        Ds.col( 1 ) = _x.col( b ) - _x.col( d );
        Ds.col( 2 ) = _x.col( c ) - _x.col( d );
        Matrix3 F = Ds * _invDm[i];

        auto det = F.determinant();

        Eigen::JacobiSVD<Matrix3> svd( F, Eigen::ComputeFullU | Eigen::ComputeFullV );
        Matrix3 R = svd.matrixU() * svd.matrixV().transpose();
        Matrix3 I = Matrix3::Identity();
        Matrix3 S = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
        Matrix3 P = R * (2.0 * _config.mu * (S - I) + _config.lambda * (S - I).trace() * I);

        Matrix3 H = -_w[i] * 1000.f * P * _invDm[i].transpose().eval();

        _Hs[i] = H;
#pragma omp critical
        {
            _f.col( a ) += H.col( 0 );
            _f.col( b ) += H.col( 1 );
            _f.col( c ) += H.col( 2 );
            _f.col( d ) -= (H.col( 0 ) + H.col( 1 ) + H.col( 2 ));
            //auto color = tinycolormap::GetGrayColor( std::clamp( std::abs( det ), 0.f, 1.f ) );
            //glm::vec3 glmc( color.r(), color.g(), color.b() );
            //_mesh->mColors[a] = glmc;
            //_mesh->mColors[b] = glmc;
            //_mesh->mColors[c] = glmc;
            //_mesh->mColors[d] = glmc;
        }
    }

    int nb_points = _mesh->GetPointNum();
    float h = _config.dt / _config.substep;
#pragma omp parallel for
    for (int i = 0; i < nb_points; i++)
    {
        //_f.col( i ) += Vector3( 0, -9.8f, 0 );
        if (!(_x0( 0, i ) > 0.3f))
        {
            _v.col( i ) += _f.col( i ) * h;
            _x.col( i ) += _v.col( i ) * h;
        }
        if (_x0( 0, i ) < -0.3f)
        {
            _x.col( i ) = _x0.col( i );
            _v.col( i ) = Vector3( 0, 0, 0 );
            _f.col( i ) = Vector3( 0, 0, 0 );
        }
        //_v.col( i ) *= 0.99f;
    }
    _f.setZero();

    static float rad = 0.0f;
    auto q = Quat( Eigen::AngleAxis<T>( std::min( 3.1415f * 1.5f, rad ), Vector3( 1, 0, 0 ) ) );
    Matrix3 m = q.toRotationMatrix();
    for (int i = 0; i < nb_points; i++)
    {
        auto p0 = _x0.col( i );
        if (p0.x() > 0.3)
        {
            _x.col( i ) = m * p0;
            _v.col( i ) = Vector3( 0, 0, 0 );
            _f.col( i ) = Vector3( 0, 0, 0 );
        }
    }
    rad += _config.dt * 0.1f;

    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start) << std::endl;

}

template <std::floating_point T>
void FEMSolver<T>::Update()
{
    _mesh->Update();
    if (Input::IsKeyDown( Input::Key::P ))
    {
        _config.simulate = !_config.simulate;
    }
    if (_config.simulate)
    {
        if (Input::IsMouseButtonDown( Input::MouseButton::Left ))
        {
            glm::vec2 cursor = Input::GetMousePosition();
            cursor.y = Camera::current->_viewport_size.y - cursor.y;
            Ray ray = Camera::current->ScreenPointToRay( cursor );
            _hold_vertex = -1;
            bool intersect = false;
            int tid = -1;

            IntersectionRec rec;
            rec.t = std::numeric_limits<float>::max();
            for (int i = 0, size = _mesh->mBorderFaces.size(); i < size; ++i)
            {
                const glm::vec3& p0 = _mesh->mPoints[_mesh->mBorderFaces[i].a];
                const glm::vec3& p1 = _mesh->mPoints[_mesh->mBorderFaces[i].b];
                const glm::vec3& p2 = _mesh->mPoints[_mesh->mBorderFaces[i].c];

                if (RayTriIntersect( ray, p0, p1, p2, &rec, 0.f, rec.t ))
                {
                    intersect = true;
                    tid = i;
                }
            }

            if (intersect)
            {
                const glm::vec3& p0 = _mesh->mPoints[_mesh->mBorderFaces[tid].a];
                const glm::vec3& p1 = _mesh->mPoints[_mesh->mBorderFaces[tid].b];
                const glm::vec3& p2 = _mesh->mPoints[_mesh->mBorderFaces[tid].c];
                float d0 = glm::distance2( p0, rec.p );
                float d1 = glm::distance2( p1, rec.p );
                float d2 = glm::distance2( p2, rec.p );
                if (d0 < d1 && d0 < d2)
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].a;
                }
                else if (d1 < d0 && d1 < d2)
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].b;
                }
                else
                {
                    _hold_vertex = _mesh->mBorderFaces[tid].c;
                }

                _cursor_start_pos = Input::GetMousePosition();
                _oripos = _mesh->mPoints[_hold_vertex];
            }
        }

        if (Input::IsMouseButtonHeld( Input::MouseButton::Left ))
        {
            auto& cam_trans = Camera::current->mTransform;
            glm::vec2 delta = Input::GetMousePosition() - _cursor_start_pos;
            glm::vec3 delta3d = -delta.x * Camera::current->mTransform.Left() - delta.y * Camera::current->mTransform.Up();
            delta3d *= 60;
            _f.col( _hold_vertex ) = ToEigen( delta3d ).cast<T>();
        }

        for (int i = 0; i < _config.substep; i++)
        {
            PhysicalUpdate();
        }
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(_mesh->GetPointNum()); i++)
        {
            _v.col( i ) *= 0.95;
        }
    }
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(_mesh->GetPointNum()); i++)
    {
        _mesh->mPoints[i] = ToGLM( Vector3( _x.col( i ) ) );
    }
    if (_config.show_tet)
    {
        _mesh->UpdateBuffers();
    }
    if (_config.show_surface)
    {
        MapToSurface();
    }
}

template <std::floating_point T>
void FEMSolver<T>::Draw()
{
    if (_config.show_tet)
    {
        _mesh->Draw();
    }
    if (_config.show_surface)
    {
        _surface->Draw();
    }
}

template <std::floating_point T>
void FEMSolver<T>::DrawGUI()
{
    ImGui::Begin( "fem" );
    ImGui::Checkbox( "show tet", &_config.show_tet );
    ImGui::Checkbox( "show surface", &_config.show_surface );
    ImGui::DragFloat( "mu", &_config.mu, 1.f, 0.f, 50000.f );
    ImGui::DragFloat( "lambda", &_config.lambda, 1.f, 0.f, 3000.f );
    ImGui::DragFloat( "dt", &_config.dt, 0.001f, 0.0f, 0.1f, "%.6f" );
    ImGui::InputInt( "substep", &_config.substep );
    ImGui::Checkbox( "simulate", &_config.simulate );
    ImGui::End();
}

template <std::floating_point T>
void FEMSolver<T>::ExtrudeTetMesh( T d )
{
    std::vector<glm::vec3> extrude_dir( _mesh->GetPointNum(), glm::vec3( 0.f ) );
    _mesh->CalcBorderFaces();
    for (int f = 0; f < _mesh->GetBorderFaceNum(); f++)
    {
        Triangle face = _mesh->mBorderFaces[f];
        int ia = face.a;
        int ib = face.b;
        int ic = face.c;
        glm::vec3 va = _mesh->mPoints[ia];
        glm::vec3 vb = _mesh->mPoints[ib];
        glm::vec3 vc = _mesh->mPoints[ic];
        glm::vec3 n = glm::cross( vb - va, vc - va );
        extrude_dir[ia] += n;
        extrude_dir[ib] += n;
        extrude_dir[ic] += n;
    }

    for (int i = 0; i < _mesh->GetPointNum(); i++)
    {
        if (glm::length2( extrude_dir[i] ) != 0.f)
            _mesh->mPoints[i] += glm::normalize( extrude_dir[i] ) * d;
    }
    _mesh->UpdateBuffers();
}

template <std::floating_point T>
void FEMSolver<T>::CreateSurfaceMapping()
{
    const int nb_vertex = _surface->GetVertexNumber();
    _skininfos.resize( nb_vertex );
#pragma omp parallel
    for (int i = 0; i < nb_vertex; i++)
    {
        glm::vec3 p = _surface->GetPosition( i );
        for (int t = 0; t < _mesh->GetTetraNum(); t++)
        {
            glm::vec4 bc = glm::TetraBarycentricPos( _mesh->mPoints[_mesh->mTetras[t].a],
                _mesh->mPoints[_mesh->mTetras[t].b],
                _mesh->mPoints[_mesh->mTetras[t].c],
                _mesh->mPoints[_mesh->mTetras[t].d],
                p );
            if (bc[0] >= 0.f && bc[1] >= 0.f && bc[2] >= 0.f && bc[3] >= 0.f)
            {
                _skininfos[i].tid = t;
                _skininfos[i].bc = ToEigen( bc ).cast<T>();
                break;
            }
        }
        if (_skininfos[i].tid == -1)
        {
            float min_dist = std::numeric_limits<float>::max();
            for (int t = 0; t < _mesh->GetTetraNum(); t++)
            {

                float d0 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].a], p );
                float d1 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].b], p );
                float d2 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].c], p );
                float d3 = glm::distance2( _mesh->mPoints[_mesh->mTetras[t].d], p );
                if (d0 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d0;
                }
                if (d1 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d1;
                }
                if (d2 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d2;
                }
                if (d3 < min_dist)
                {
                    _skininfos[i].tid = t;
                    min_dist = d3;
                }
            }
            glm::vec4 bc = glm::TetraBarycentricPos( _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].a],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].b],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].c],
                _mesh->mPoints[_mesh->mTetras[_skininfos[i].tid].d],
                p );
            _skininfos[i].bc = ToEigen( bc ).cast<T>();
        }
    }
}

template <std::floating_point T>
void FEMSolver<T>::MapToSurface()
{
#pragma omp parallel
    for (int i = 0; i < _surface->GetVertexNumber(); i++)
    {
        int tid = _skininfos[i].tid;
        glm::vec4 bc = ToGLM( _skininfos[i].bc );

        glm::vec3 pa = _mesh->mPoints[_mesh->mTetras[tid].a];
        glm::vec3 pb = _mesh->mPoints[_mesh->mTetras[tid].b];
        glm::vec3 pc = _mesh->mPoints[_mesh->mTetras[tid].c];
        glm::vec3 pd = _mesh->mPoints[_mesh->mTetras[tid].d];

        glm::vec3 result = pa * bc[0] + pb * bc[1] + pc * bc[2] + pd * bc[3];
        _surface->SetPosition( i, result );
    }
    _surface->UpdateNormal();
    _surface->UpdatePosBuffer();
    _surface->UpdateAttrBuffer();
}
