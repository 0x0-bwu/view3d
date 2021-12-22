#ifndef VIEW_MODEL_H
#define VIEW_MODEL_H
#include "Config.h"
#include "Color.h"
#include "generic/geometry/TriangulationRefinement.hpp"
#include "generic/geometry/Tetrahedralization.hpp"
#include "generic/geometry/GeometryTraits.hpp"
#include "generic/geometry/Topology.hpp"
#include <memory>
#include <vector>
#include <list>
namespace view {

class Model
{
public:
    Model(){}
    virtual ~Model();
    virtual transform3d_t Transform() const = 0;
    virtual transform3d_t invTansform(bool * res = nullptr) const = 0;
};

class Model2D : public Model
{
public:
    transform2d_t trans;
    std::vector<point2d_t> vertices;

    Model2D(){}
    virtual ~Model2D();
    virtual void Clear();
    virtual void Normalize();
    virtual size_t AddVertex(point2d_t p);
    virtual transform3d_t Transform() const;
    virtual transform3d_t invTansform(bool * res = nullptr) const;
    virtual void SetTransform(const transform2d_t & t);
};

class Model3D : public Model
{
public:
    transform3d_t trans;
    std::vector<point3d_t> vertices;

    Model3D(){}
    virtual ~Model3D();
    virtual void Clear();
    virtual void Normalize();
    virtual size_t AddVertex(point3d_t p);
    virtual transform3d_t Transform() const;
    virtual transform3d_t invTansform(bool * res = nullptr) const;
    virtual void SetTransform(const transform3d_t & t);
};

struct VertexList
{
    int32_t color = color::white;
    std::list<size_t> indices;
};

class FrameModel
{
public:
    std::vector<VertexList> models;
    
    template <typename iterator>
    void AddVertexList(iterator begin, iterator end, int32_t color)
    {
        VertexList vl;
        vl.color = color;
        vl.indices.insert(vl.indices.end(), begin, end);
        models.emplace_back(std::move(vl));
    }

    FrameModel(){}
    virtual ~FrameModel();
    virtual void Clear();
};

class FrameModel2D : public FrameModel, public Model2D
{
public:
    FrameModel2D(){}
    virtual ~FrameModel2D();

    virtual void Clear();

    template <typename num_type>
    point2d_t toPoint2D(const Point2D<num_type> & p)
    {
        return p.template Cast<coor_t>();
    }

    template <typename num_type>
    void AddPoint2D(const Point2D<num_type> & p, num_type radius, int32_t color)
    {
        AddCircle2D(Circle<num_type>(p, radius), color);
    }

    template <typename num_type>
    void AddCircle2D(const Circle<num_type> & c, int32_t color)
    {
        AddPolygon2D(geometry::InscribedPolygon(c, constant::circle_div), color);
    }

    template <typename num_type>
    void AddSegment2D(const Segment2D<num_type> & s, int32_t color)
    {
        VertexList vertexList;
        vertexList.color = color;
        vertexList.indices.push_back(AddVertex(toPoint2D(s[0])));
        vertexList.indices.push_back(AddVertex(toPoint2D(s[1])));
        models.emplace_back(std::move(vertexList));
    }

    template <typename num_type>
    void AddTriangle2D(const Triangle2D<num_type> & tri, int32_t color)
    {
        VertexList vertexList;
        vertexList.color = color;
        for(size_t i = 0; i < 3; ++i)
            vertexList.indices.push_back(AddVertex(toPoint2D(tri[i])));
        models.emplace_back(std::move(vertexList));
    }

    template <typename num_type>
    void AddPolygon2D(const Polygon2D<num_type> & polygon, int32_t color)
    {
        VertexList vertexList;
        vertexList.color = color;
        size_t size = polygon.Size();
        size_t end  = polygon.ConstFront() == polygon.ConstBack() ? size - 1 : size;
        for(size_t i = 0; i < end; ++i)
            vertexList.indices.push_back(AddVertex(toPoint2D(polygon[i])));
        models.emplace_back(std::move(vertexList));
    }

    template <typename num_type>
    void AddPolygonWithHoles2D(const PolygonWithHoles2D<num_type> & pwh, int32_t color)
    {
        AddPolygon2D(pwh.outline, color);
        for(const auto & hole : pwh.holes){
            AddPolygon2D(hole, color);
        }
    }
};

class FrameModel3D : public FrameModel, public Model3D
{
public:
    FrameModel3D(){}
    virtual~FrameModel3D();
    
    virtual void Clear();

    template <typename num_type>
    point3d_t toPoint3D(const Point2D<num_type> & point, num_type zRef)
    {
        auto p = point.template Cast<coor_t>();
        return point3d_t(p[0], p[1], coor_t(zRef));
    }

    template <typename num_type>
    void AddSphere(const Point3D<num_type> & center, num_type radius, int32_t color)
    {
        const auto div = constant::circle_div;
        float_t ang = math::pi_2 / div;
        for(size_t i = 0; i < 3; ++i){
            VertexList vertexList;
            vertexList.color = color;
            for(size_t j = 0; j < div; ++j){
                Point3D<num_type> p;
                auto idx1 = (i + 0) % 3;
                auto idx2 = (i + 1) % 3;
                auto idx3 = (i + 2) % 3;
                p[idx1] = center[idx1] + std::sin(ang * j) * radius;
                p[idx2] = center[idx2] + std::cos(ang * j) * radius;
                p[idx3] = center[idx3];
                auto index = AddVertex(p);
                vertexList.indices.push_back(index);
            }
            models.emplace_back(std::move(vertexList));
        }
    }

    template <typename num_type>
    size_t AddPolygon2D(const Polygon2D<num_type> & polygon, num_type zRef, int32_t color)
    {
        VertexList vertexList;
        vertexList.color = color;
        size_t size = polygon.Size();
        size_t end  = polygon.ConstFront() == polygon.ConstBack() ? size - 1 : size;
        for(size_t i = 0; i < end; ++i){
            auto index = AddVertex(toPoint3D(polygon[i], zRef));
            vertexList.indices.push_back(index);
        }
        models.emplace_back(std::move(vertexList));
        return models.back().indices.front();
    }

    template <typename num_type>
    void AddPolygon3D(const Polygon2D<num_type> & polygon, num_type zRef, num_type height, int32_t color)
    {
        auto bot = AddPolygon2D(polygon, zRef, color);
        auto top = AddPolygon2D(polygon, zRef + height, color);
        auto size = top - bot;

        for(size_t i = 0; i < size; ++i){
            auto offset1 = i % size;
            auto offset2 = (i + 1) % size;
            VertexList vertexList;
            vertexList.color = color;
//            vertexList.indices = { bot + offset1, bot + offset2, top + offset2, top + offset1 };
            vertexList.indices = { bot + offset1, top + offset1 };
            models.emplace_back(std::move(vertexList));
        }
    }

    template <typename num_type>
    void AddPolygonWithHoles3D(const PolygonWithHoles2D<num_type> & pwh, num_type zRef, num_type height, int32_t color)
    {
        AddPolygon3D(pwh.outline, zRef, height, color);
        for(const auto & hole : pwh.holes){
            AddPolygon3D(hole, zRef, height, color);
        }
    }

    tet::PiecewiseLinearComplex<Point3D<coor_t> > toPLC(bool normalized) const
    {
        tet::PiecewiseLinearComplex<Point3D<coor_t> > plc;
        if(normalized) plc.points = vertices;
        else {
            auto inv = invTansform();
            plc.points.reserve(vertices.size());
            for(const auto & vertex : vertices){
                plc.points.push_back(inv * vertex);
            }
        }

        plc.surfaces.resize(models.size());
        for(size_t i = 0; i < models.size(); ++i){
            plc.surfaces[i].faces.resize(1);
            auto & face = plc.surfaces[i].faces.front();
            face.reserve(models[i].indices.size());
            face.insert(face.end(), models[i].indices.begin(), models[i].indices.end());
        }

        return plc;
    }

};

using SurfVertex = geometry::tri::IndexVertex;
using SurfTriangle = geometry::tri::IndexTriangle;
using SurfVertexGraph = geometry::tri::VertexGraph;

template <typename point_t>
using SurfTriangulation = geometry::tri::Triangulation<point_t>;

class SurfaceModel
{
public:
    using Line = typename SurfVertexGraph::Edge;
    using LineStrips = typename SurfVertexGraph::Edges;

    std::vector<int32_t> colors;
    std::unique_ptr<SurfVertexGraph> vertexGraph;

    SurfaceModel(){}
    virtual ~SurfaceModel();

    virtual void Clear();
    virtual void GetLineStrips(LineStrips & lines) const;
};

class SurfaceModel2D : public SurfaceModel, public Model2D
{
public:
    SurfaceModel2D(){}
    virtual ~SurfaceModel2D();

    virtual void Clear();
};

class SurfaceModel3D : public SurfaceModel, public Model3D
{
public:

    SurfaceModel3D(){}
    virtual ~SurfaceModel3D();

    virtual void Clear();
};

class TetrahedronModel : public Model3D
{
public:
    using Element = tet::IndexEle4;
    std::vector<int32_t> colors;
    std::vector<Element> elements;

    TetrahedronModel(){}
    virtual ~TetrahedronModel();

    virtual void Clear();
};

inline transform2d_t makeNormalized(std::vector<point2d_t> & points)
{
    using namespace geometry;
    box2d_t box;
    for(const auto & point : points)
        box |= point;

    vector2d_t shift = -box.Center();
    coor_t maxLength = std::max(box.Length(), box.Width());
    coor_t scale = 1.0 / maxLength;
    transform2d_t trans = makeScaleTransform2D(scale) * makeShiftTransform2D<coor_t>(shift);
    geometry::Transform<point2d_t>(points.begin(), points.end(), trans);
    return trans;
}

inline transform3d_t makeNormalized(std::vector<point3d_t> & points)
{
    using namespace geometry;
    box3d_t box;
    for(const auto & point : points)
        box |= point;

    vector3d_t shift = -box.Center();
    coor_t maxLength = std::max(box.Length(), std::max(box.Width(), box.Height()));
    coor_t scale = 1.0 / maxLength;

    transform3d_t trans = makeScaleTransform3D(scale) * makeShiftTransform3D<coor_t>(shift);
    geometry::Transform<point3d_t>(points.begin(), points.end(), trans);
    return trans;
}

template <typename polygon2d, typename iterator,
          typename std::enable_if<std::is_same<polygon2d,
          typename std::iterator_traits<iterator>::value_type>::value, bool>::type = true>
inline std::unique_ptr<FrameModel2D> makeFrameModelFromPolygon2D(iterator begin, iterator end)
{
    auto model = new FrameModel2D;
    size_t size = std::distance(begin, end);
    model->vertices.reserve(size * 12);
    model->models.reserve(size);

    for(auto iter_polygon = begin; iter_polygon != end; ++iter_polygon){
        const polygon2d & polygon = *iter_polygon;
        model->AddPolygon2D(polygon, color::white);
    }
    model->Normalize();
    return std::unique_ptr<FrameModel2D>(model);
}

template <typename polygon_with_holes2d, typename iterator,
          typename std::enable_if<std::is_same<polygon_with_holes2d,
          typename std::iterator_traits<iterator>::value_type>::value, bool>::type = true>
inline std::unique_ptr<Model> makeFrameModelFromPolygonWithHoles2D(iterator begin, iterator end)
{
    auto model = new FrameModel2D;
    size_t size = std::distance(begin, end);
    model->vertices.reserve(size * 12);
    model->models.reserve(size);

    for(auto iter_pwh = begin; iter_pwh != end; ++iter_pwh){
        const polygon_with_holes2d & pwh = *iter_pwh;
        model->AddPolygonWithHoles2D(pwh, color::white);
    }
    model->Normalize();
    return std::unique_ptr<FrameModel2D>(model);
}

template <typename polygon_with_holes2d, typename iterator,
          typename std::enable_if<std::is_same<polygon_with_holes2d,
          typename std::iterator_traits<iterator>::value_type>::value, bool>::type = true>
inline std::unique_ptr<Model> makeFrameModel3DFromPolygonWithHoles2D(iterator begin, iterator end, 
                                                                     typename polygon_with_holes2d::coor_t zRef,
                                                                     typename polygon_with_holes2d::coor_t height)
{
    auto model = new FrameModel3D;
    size_t size = std::distance(begin, end);
    model->vertices.reserve(size * 12);
    model->models.reserve(size);

    for(auto iter_pwh = begin; iter_pwh != end; ++iter_pwh){
        const polygon_with_holes2d & pwh = *iter_pwh;
        model->AddPolygonWithHoles3D(pwh, zRef, height, color::white);
    }
    model->Normalize();
    return std::unique_ptr<FrameModel3D>(model);
}

template <typename point_t, typename std::enable_if<point_t::dim == 3, bool>::type = true>
inline std::unique_ptr<Model> makeFrameModel3DFromPiecewiseLinearComplex(const geometry::tet::PiecewiseLinearComplex<point_t> & plc)
{
    auto model = new FrameModel3D;
    model->vertices = plc.points;
    for(const auto & surface : plc.surfaces){
        for(const auto & face : surface.faces){
            model->AddVertexList(face.begin(), face.end(), color::white);
        }
        for(const auto & hole : surface.holes){
            //model->AddSphere(hole, 0.2, color::red);
        }
    }
    model->Normalize();
    return std::unique_ptr<FrameModel3D>(model);
}

template<typename point_t> std::unique_ptr<Model>
makeSurfaceModelFromTriangulation(const tri::Triangulation<point_t > & tri);

template <typename polygon_with_holes2d, typename iterator,
          typename std::enable_if<std::is_same<polygon_with_holes2d,
          typename std::iterator_traits<iterator>::value_type>::value, bool>::type = true>
inline std::unique_ptr<Model> makeSurfaceModelFromPolygonWithHoles2D(iterator begin, iterator end)
{
    using coor_t = typename polygon_with_holes2d::coor_t;
    geometry::GeoTopology2D<coor_t> geoTopo;
    for(auto iter_pwh = begin; iter_pwh != end; ++iter_pwh){
        const polygon_with_holes2d & pwh = *iter_pwh;
        geoTopo.AddGeometry(pwh);
    }

    using Edge = std::pair<size_t, size_t>;
    using Point = Point2D<coor_t>;
    using Triangulator = tri::Triangulator2D<coor_t>;
    using Triangulation = tri::Triangulation<Point>;

    std::list<Edge > edges;
    geoTopo.GetAllEdges(edges);
    const auto & points = geoTopo.GetAllPoints();

    Triangulation tri;
    Triangulator triangulator(tri);
    try {
        triangulator.InsertVertices(points.begin(), points.end(), [](const Point & p){ return p[0]; }, [](const Point & p){ return p[1]; });
        triangulator.InsertEdges(edges.begin(), edges.end(), [](const Edge & e){ return e.first; }, [](const Edge & e){ return e.second; });
        triangulator.EraseSuperTriangle();
    }
    catch (...) { return nullptr; }

    return makeSurfaceModelFromTriangulation(tri);
}

template<typename point_t>
inline  std::unique_ptr<Model> makeSurfaceModelFromTriangulation(const tri::Triangulation<point_t> & tri, const tri::TriIdxSet & ignored = {})
{
    auto surfaceModel = new SurfaceModel2D;
    surfaceModel->colors.assign(tri.points.size(), 0xFFFFFFFF);
    surfaceModel->vertices.reserve(tri.points.size());
    for(const auto & point : tri.points)
        surfaceModel->vertices.emplace_back(point.template Cast<coor_t>());
    surfaceModel->vertexGraph.reset(new SurfVertexGraph(tri, ignored));
    surfaceModel->Normalize();
    return std::unique_ptr<SurfaceModel2D>(surfaceModel);
}

template <typename point_t>
inline std::unique_ptr<Model> makeSurfaceModelFromNodesAndEdges(std::vector<point_t> nodes, const std::vector<geometry::tet::IndexEdge> & edges)
{
    auto nSize = nodes.size();
    SurfaceModel * surfaceModel = nullptr;
    if constexpr (2 == point_t::dim){
        surfaceModel = new SurfaceModel2D;
        auto model2d = dynamic_cast<Model2D *>(surfaceModel);
        if(nullptr == model2d) return nullptr;
        model2d->vertices = std::move(nodes);
        model2d->Normalize();
    }
    else{
        surfaceModel = new SurfaceModel3D;
        auto model3d = dynamic_cast<Model3D *>(surfaceModel);
        if(nullptr == model3d) return nullptr;
        model3d->vertices = std::move(nodes);
        model3d->Normalize();
    }
    surfaceModel->colors.assign(nSize, 0xFFFFFFFF);
    surfaceModel->vertexGraph.reset((new SurfVertexGraph));
    for(const auto & edge : edges){
        surfaceModel->vertexGraph->AddEdge(edge.v1(), edge.v2());
    }
    return std::unique_ptr<Model>(dynamic_cast<Model *>(surfaceModel));
}

template <typename point_t, typename std::enable_if<point_t::dim == 3, bool>::type = true>
inline std::unique_ptr<Model> makeSurfaceModelFromNodesAndEle4s(std::vector<point_t> nodes, const std::vector<tet::IndexEle4> & ele4s)
{
    auto nSize = nodes.size();
    auto surfaceModel = new SurfaceModel3D;
    surfaceModel->vertices = std::move(nodes);
    surfaceModel->Normalize();

    surfaceModel->colors.assign(nSize, 0xFFFFFFFF);
    surfaceModel->vertexGraph.reset((new SurfVertexGraph));
    for(const auto & ele4 : ele4s){
        for(size_t i = 0; i < 4; ++i){
            for(size_t j = i + 1; j < 4; ++j){
                surfaceModel->vertexGraph->AddEdge(ele4[i], ele4[j]);
            }
        }
    }
    return std::unique_ptr<Model>(dynamic_cast<Model *>(surfaceModel));
}

template <typename point_t, typename std::enable_if<point_t::dim == 3, bool>::type = true>
inline std::unique_ptr<Model> makeTetrahedronModelFromNodesAndEle4s(std::vector<point_t> nodes, std::vector<tet::IndexEle4> ele4s)
{
    auto nSize = nodes.size();
    auto tetrahedronModel = new TetrahedronModel;
    tetrahedronModel->vertices = std::move(nodes);
    tetrahedronModel->elements = std::move(ele4s);
    tetrahedronModel->colors.assign(nSize, color::blue);
    tetrahedronModel->Normalize();
    return std::unique_ptr<Model>(dynamic_cast<Model *>(tetrahedronModel));
}
}//namespace view

#endif//VIEW_MODEL
