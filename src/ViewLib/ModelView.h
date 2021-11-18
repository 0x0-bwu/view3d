#ifndef VIEW_MODELVIEW_H
#define VIEW_MODELVIEW_H
#include "View3D.h"
#include "Model.h"
#include <memory>
namespace generic{
namespace tree {
namespace bvh {
template <typename coor_t> class BVH;
} }
namespace geometry{
namespace tri {
template <typename point_t> class TriangleEvaluator;
} } }

namespace view {

class ModelView : public View3D
{
public:
    ModelView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    virtual ~ModelView();

protected:
    virtual void PerformKeyBoardAction(KeyBoardAction ka);
    virtual void draw();

protected:
    virtual void mouseMoveEvent(QMouseEvent *);
    virtual void mousePressEvent(QMouseEvent *);

protected:
    QPoint m_currPos;
    std::unique_ptr<Model> m_model;
};

class FrameModelView : public ModelView
{
public:
    FrameModelView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    virtual ~FrameModelView();

private:
    void InitFromWktFile();
    void InitFromPolyFile();
    void InitFromDomDmcFile();
    void InitFromConnectivityTest();

};

class SurfaceModelView : public ModelView
{
public:
    SurfaceModelView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    virtual ~SurfaceModelView();

private:
    void InitFromNodeEdgeFiles();
    void InitFromNodeEle4Files();

protected:
    virtual void PerformKeyBoardAction(KeyBoardAction ka);
    virtual void draw();
};

class SurfaceMeshView : public SurfaceModelView
{
    using coor_t = int64_t;
    using float_t = float_type<coor_t>;
    using BVH = tree::bvh::BVH<coor_t>;
    using Triangulator2D = geometry::tri::Triangulator2D<coor_t>;
    using Triangulation = geometry::tri::Triangulation<Point2D<coor_t> >;
    using DelaunayRefinement = geometry::tri::DelaunayRefinement2D<coor_t>;
//    using CurrentRefineMethod = geometry::tri::RuppertRefinement2D<coor_t>;
//    using CurrentRefineMethod = geometry::tri::ChewSecondRefinement2D<coor_t>;
    using CurrentRefineMethod = geometry::tri::JonathanRefinement2D<coor_t>;
    using TriangleEvaluator = geometry::tri::TriangleEvaluator<Point2D<coor_t> >;
    using TriangleEvaluation = geometry::tri::TriangleEvaluation<TriangleEvaluator::bins>;
    using TriangulationUtility = geometry::tri::TriangulationUtility<Point2D<coor_t> >;

public:
    SurfaceMeshView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    ~SurfaceMeshView();

protected:
    void PerformKeyBoardAction(KeyBoardAction ka);
    void ShowInfo();
    void ShowAxis();
    void draw();

private:
    void InitFromWktFile();
    void InitFromMshFile();
    void InitFromDomDmcFile();
    void InitFromTopologyFile();
    void InitFromTriangulationArchive(bool loadRefineState);
    void PreTriangulation(const geometry::GeoTopology2D<coor_t> & geoTopo, coor_t mergePointDist = 0);
    void PerformNextAction();
    void PerformPrevAction();
    void PerformTestAction();
    void PerformUpdateAction();
    void UpdateModels();
    void UpdateEvaluation();
    void BuildBVH();

private:
    std::unique_ptr<BVH > m_bvh;
    std::unique_ptr<FrameModel2D> m_highlightModel;
    std::unique_ptr<Triangulation> m_triangulation;
    std::unique_ptr<DelaunayRefinement> m_refinement;
    TriangleEvaluation m_evaluation;
    size_t m_iteration = 0;
};


class TetrahedronModelView : public ModelView
{
public:
    TetrahedronModelView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    virtual ~TetrahedronModelView();

private:
    void InitFromNodeEle4Files();

protected:
    virtual void PerformKeyBoardAction(KeyBoardAction ka);
    virtual void draw();
};

}//namespace view
#endif//VIEW_MODELVIEW_H
