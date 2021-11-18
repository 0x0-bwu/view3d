#include "Model.h"
using namespace view;
Model::~Model()
{
}

Model2D::~Model2D()
{
}

void Model2D::Clear()
{
    trans = transform2d_t();
    vertices.clear();
}

void Model2D::Normalize()
{
    trans = makeNormalized(vertices);
}

size_t Model2D::AddVertex(point2d_t p)
{
    vertices.push_back(std::move(p));
    return vertices.size() - 1;
}

transform3d_t Model2D::Transform() const
{
    return transform3d_t(trans);
}

transform3d_t Model2D::invTansform(bool * res) const
{
    auto inv = Transform();
    auto r = inv.Inverse();
    if(res) *res = r;
    return inv;
}

void Model2D::SetTransform(const transform2d_t & t)
{
    trans = t;
    geometry::Transform<point2d_t>(vertices.begin(), vertices.end(), trans);
}

Model3D::~Model3D()
{
}

void Model3D::Clear()
{
    trans = transform3d_t();
    vertices.clear();
}

void Model3D::Normalize()
{
    trans = makeNormalized(vertices);
}

size_t Model3D::AddVertex(point3d_t p)
{
    vertices.push_back(std::move(p));
    return vertices.size() - 1;
}

transform3d_t Model3D::Transform() const
{
    return trans;
}

transform3d_t Model3D::invTansform(bool * res) const
{
    auto inv = Transform();
    auto r = inv.Inverse();
    if(res) *res = r;
    return inv;
}

void Model3D::SetTransform(const transform3d_t & t)
{
    trans = t;
    geometry::Transform<point3d_t>(vertices.begin(), vertices.end(), trans);
}

FrameModel::~FrameModel()
{
}

void FrameModel::Clear()
{
    models.clear();
}

FrameModel2D::~FrameModel2D()
{
}

void FrameModel2D::Clear()
{
    Model2D::Clear();
    FrameModel::Clear();
}

FrameModel3D::~FrameModel3D()
{

}

void FrameModel3D::Clear()
{
    Model3D::Clear();
    FrameModel::Clear();
}

SurfaceModel::~SurfaceModel()
{
}

void SurfaceModel::Clear()
{
    colors.clear();
    vertexGraph.reset();
}

void SurfaceModel::GetLineStrips(LineStrips & lines) const
{
    lines.clear();
    if(nullptr == vertexGraph) return;
    vertexGraph->GetEdges(lines);
}

SurfaceModel2D::~SurfaceModel2D()
{
}

void SurfaceModel2D::Clear()
{
    Model2D::Clear();
    SurfaceModel::Clear();
}

SurfaceModel3D::~SurfaceModel3D()
{
}

void SurfaceModel3D::Clear()
{
    Model3D::Clear();
    SurfaceModel::Clear();
}

TetrahedronModel::~TetrahedronModel()
{
}

void TetrahedronModel::Clear()
{
    colors.clear();
    elements.clear();
    Model3D::Clear();
}
