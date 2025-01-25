#include "ModelView.h"
#include "Camera.h"
#include "generic/geometry/TetrahedralizationIO.hpp"
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Connectivity.hpp"
#include "generic/geometry/GeometryIO.hpp"
#include "generic/geometry/Mesh2D.hpp"
#include "generic/tree/BVHUtilityMT.hpp"
#include "generic/tools/Tools.hpp"
#include "Painter.h"
#include <QApplication>
#include <QMouseEvent>
#include <QFileInfo>
#include <QString>
#include <QDebug>
using namespace view;
using namespace generic::geometry;
ModelView::ModelView(QWidget * parent, Qt::WindowFlags flags)
 : View3D(parent, flags), m_model(nullptr)
{
}

ModelView::~ModelView()
{
}

void ModelView::draw()
{
    if(m_model) m_painter->DrawMode(m_model.get());
}

void ModelView::PerformKeyBoardAction(KeyBoardAction ka)
{
    View3D::PerformKeyBoardAction(ka);
}

void ModelView::mouseMoveEvent(QMouseEvent * e)
{
    View3D::mouseMoveEvent(e);
    m_currPos = e->pos();
}

void ModelView::mousePressEvent(QMouseEvent *e)
{
    View3D::mousePressEvent(e);
}

FrameModelView::FrameModelView(QWidget * parent, Qt::WindowFlags flags)
 : ModelView(parent, flags)
{
}

FrameModelView::~FrameModelView()
{

}

void FrameModelView::ShowAxis()
{
    QFont ft = QApplication::font();

    auto camera = GetCamera();
    auto width = camera->ScreenWidth();
    auto height = camera->ScreenHeight();
    auto trans = m_model->Transform();

    auto inside = [&width, &height](const point3d_t & p)
    {
      return 0 < p[0] && p[0] < width && 0 < p[1] && p[1] < height;
    };

    auto model = dynamic_cast<FrameModel3D*>(m_model.get());
    if(nullptr == model) return;

    //vertices
    for(auto iv = 0; iv < model->vertices.size(); ++iv){
        const auto & p = model->vertices.at(iv);
        auto p3d = camera->ProjectedCoordinatesOf(p);
        if(!inside(p3d)) continue;
        m_painter->DrawText(p3d[0], p3d[1], QString::number(iv + 1), ft, Qt::red);//index start from 1
    }
}

SurfaceModelView::SurfaceModelView(QWidget * parent, Qt::WindowFlags flags)
 : ModelView(parent, flags)
{
}

SurfaceModelView::~SurfaceModelView()
{
}

void SurfaceModelView::PerformKeyBoardAction(KeyBoardAction ka)
{
    ModelView::PerformKeyBoardAction(ka);
}

void SurfaceModelView::ShowAxis()
{
    QFont ft = QApplication::font();

    auto camera = GetCamera();
    auto width = camera->ScreenWidth();
    auto height = camera->ScreenHeight();
    auto trans = m_model->Transform();

    auto inside = [&width, &height](const point3d_t & p)
    {
      return 0 < p[0] && p[0] < width && 0 < p[1] && p[1] < height;
    };

    auto model = dynamic_cast<SurfaceModel3D*>(m_model.get());
    if(nullptr == model) return;

    //vertices
    for(auto iv = 0; iv < model->vertices.size(); ++iv){
        const auto & p = model->vertices.at(iv);
        auto p3d = camera->ProjectedCoordinatesOf(p);
        if(!inside(p3d)) continue;
        m_painter->DrawText(p3d[0], p3d[1], QString::number(iv + 1), ft, Qt::red);//index start from 1
    }
}

void SurfaceModelView::draw()
{
    ModelView::draw();
}

SurfaceMeshView::SurfaceMeshView(QWidget * parent, Qt::WindowFlags flags)
 : SurfaceModelView(parent, flags)
 , m_triangulation(new Triangulation)
{
    InitFromWktFile();
    UpdateModels();
}

SurfaceMeshView::~SurfaceMeshView()
{
}

void SurfaceMeshView::InitFromWktFile()
{
    using namespace generic::geometry;
    QString fileName = QApplication::applicationDirPath() + "/../../../data/sic.wkt";
    fileName = QFileInfo(fileName).canonicalFilePath();
    qDebug() << "Info: loading wkt file " << fileName;
    std::vector<Polygon2D<coor_t> > polygons;
    if(GeometryIO::ReadWKT<Polygon2D<coor_t> >(fileName.toStdString(), std::back_inserter(polygons))){
        qDebug() << "Info: " << polygons.size() << " polygons loaded";
        PreTriangulation(polygons, 0);
    }
    else {
        qDebug() << "Error: fail to load file: " << fileName;
    }
    m_refinement.reset(new CurrentRefineMethod(*m_triangulation));
    m_refinement->SetParas(math::Rad(15), 1e3, 3e6);
}

void SurfaceMeshView::PreTriangulation(const std::vector<Polygon2D<coor_t> > & polygons, coor_t mergePointDist)
{
    mesh2d::IndexEdgeList edges;
    mesh2d::Point2DContainer points;
    mesh2d::Segment2DContainer segments;
    mesh2d::ExtractIntersections(polygons, segments);
    mesh2d::ExtractTopology(segments, points, edges);
    // points.reserve(points.size() + steinerPoints.size());
    // points.insert(points.end(), steinerPoints.begin(), steinerPoints.end());
    mesh2d::MergeClosePointsAndRemapEdge(points, edges, mergePointDist);
    mesh2d::TriangulatePointsAndEdges(points, edges, *m_triangulation);
}

void SurfaceMeshView::PerformKeyBoardAction(KeyBoardAction ka)
{
    SurfaceModelView::PerformKeyBoardAction(ka);
    switch (ka) {
    case KeyBoardAction::NextAction :
        PerformNextAction();
        break;
    case KeyBoardAction::PrevAction :
        PerformPrevAction();
        break;
    case KeyBoardAction::TestAction :
        PerformTestAction();
        break;
    case KeyBoardAction::UpdateAction :
        PerformUpdateAction();
        break;
    default :
        break;
    }
}

void SurfaceMeshView::ShowInfo()
{
    QStringList infos;
    infos << QString("Min angle: %1deg").arg(math::Deg(m_evaluation.minAngle));
    infos << QString("Max angle: %1deg").arg(math::Deg(m_evaluation.maxAngle));
    infos << QString("Min edge: %1").arg(m_evaluation.minEdgeLen);
    infos << QString("Max edge: %1").arg(m_evaluation.maxEdgeLen);

    auto sProgress = [](float_t progress, size_t len = 50)
    {
        QString progressBar;
        for(size_t i = 0; i < progress * len; ++i)
            progressBar.append('#');
        return progressBar;
    };

    const auto & histogram = m_evaluation.triAngleHistogram;
    float_t step = 60.0 / histogram.size();
    float_t begin(0), end(0);
    for(size_t i = 0; i < histogram.size(); ++i){
        end = begin + step;
        infos<< QString(" %1-%2: %3")
                .arg((int)begin, 2, 10, QLatin1Char('0'))
                .arg((int)end, 2, 10, QLatin1Char('0'))
                .arg(sProgress(histogram.at(i)));
        begin = end;
    }

    infos << QString("Encroached Edge: %1").arg(m_refinement->CurrEncroachedEdge().has_value() ? "true" : "false");
    infos << QString("Skinny Triangle: %1").arg(m_refinement->CurrSkinnyTriangle().has_value() ? "true" : "false");
    infos << QString("Total Nodes: %1").arg(m_refinement->GetOp().CurrentVertexSize());
    infos << QString("Total Elements: %1").arg(m_refinement->GetOp().CurrentTriangleSize());
    infos << QString("Iteration: %1").arg(m_iteration);

    int fh = 1.5 * QApplication::font().pixelSize() > 0 ? QApplication::font().pixelSize() : QApplication::font().pointSize();
    int x = 10, y = 0.5 * fh;
    for(auto i = 0; i < infos.size(); ++i){
        y += fh;
        m_painter->DrawText(x, y + 1.5 * fh, infos.at(i));
    }
}

void SurfaceMeshView::ShowAxis()
{
    QFont ft = QApplication::font();

    auto camera = GetCamera();
    auto width = camera->ScreenWidth();
    auto height = camera->ScreenHeight();
    auto trans = m_model->Transform();

    auto inside = [&width, &height](const point3d_t & p)
    {
      return 0 < p[0] && p[0] < width && 0 < p[1] && p[1] < height;
    };

    //vertices
    for(auto iv = 0; iv < m_triangulation->vertices.size(); ++iv){
        if(m_refinement->GetOp().isVertexRemoved(iv)) continue;
        const auto & v = m_triangulation->vertices[iv];
        const auto & p = m_triangulation->points[v.index];
        auto p3d = trans * point3d_t(p[0], p[1], 0);
        p3d = camera->ProjectedCoordinatesOf(p3d);
        if(!inside(p3d)) continue;
        m_painter->DrawText(p3d[0], p3d[1], QString::number(iv), ft, Qt::red);
    }

    //triangles
    for(auto it = 0; it < m_triangulation->triangles.size(); ++it){
        if(m_refinement->GetOp().isTriangleRemoved(it)) continue;
        const auto & p = TriangulationUtility::GetCenter(*m_triangulation, it);
        auto p3d = trans * point3d_t(p[0], p[1], 0);
        p3d = camera->ProjectedCoordinatesOf(p3d);
        if(!inside(p3d)) continue;
        m_painter->DrawText(p3d[0], p3d[1], QString::number(it), ft, Qt::yellow);
    }
}

void SurfaceMeshView::draw()
{
    if(m_highlightModel)
        m_painter->DrawFrameModel2D(m_highlightModel.get());
    SurfaceModelView::draw();
}

void SurfaceMeshView::PerformNextAction()
{
//    for(size_t i = 0; i < 300; ++i){
//        QImage img = FrameBufferSnapshot();
//        QString fileName = QString("/home/bing/test/snapshot/%1.png").arg(i + 1);
//        img.save(fileName);
//        m_refinement->Refine(100);
//        UpdateModels();
//    }
    size_t step = 1;
    m_iteration += step;
    m_refinement->Refine(step);
    UpdateModels();
}

void SurfaceMeshView::PerformPrevAction()
{
    // m_refinement->Undo();
    size_t step = 200;
    m_iteration += step;
    m_refinement->Refine(step);
    UpdateModels();
}

void SurfaceMeshView::PerformTestAction()
{
//    QString archive = QApplication::applicationDirPath() + "/../../../test/archive.txt";
//    QString stateArchive = QApplication::applicationDirPath() + "/../../../test/state.txt";
//    bool res = geometry::tri::TriangulationUtility<Point2D<coor_t> >::Write(*m_triangulation, archive.toStdString());
//    res = res && m_refinement->SaveState(stateArchive.toStdString());
//    if(res) std::cout << "write archive to " << archive.toStdString() << " successfully!" << std::endl;
//    size_t step = 10;
//    m_iteration += step;
//    m_refinement->Refine(step);
    m_refinement->MergeShortestEdge();
    UpdateModels();
}

void SurfaceMeshView::PerformUpdateAction()
{
    //m_refinement->UpdateState();
    UpdateModels();
}

void SurfaceMeshView::UpdateModels()
{
    m_highlightModel.reset();
    m_model.reset(makeSurfaceModelFromTriangulation(*m_triangulation, m_refinement->GetOp().GetRemovedTriangles()).release());

    if(m_model){
        m_highlightModel.reset(new FrameModel2D);
        auto shortest = m_refinement->CurrShortestEdge();
        auto edge = m_refinement->CurrEncroachedEdge();
        auto tri = m_refinement->CurrSkinnyTriangle();
        auto p = m_refinement->CurrVertexPoint();
        if(shortest.has_value()){
            m_highlightModel->AddSegment2D(shortest.value(), color::red);
            auto circle = DiametralCircle(shortest.value());
            circle.r = 1000;
            m_highlightModel->AddCircle2D(circle, color::blue);
        }
        if(edge.has_value()){
            m_highlightModel->AddSegment2D(edge.value(), color::red);
            m_highlightModel->AddCircle2D(DiametralCircle(edge.value()), color::green);
        }
        if(tri.has_value()){
            m_highlightModel->AddTriangle2D(tri.value(), color::red);
            m_highlightModel->AddCircle2D(CircumCircle(tri.value()), color::red);
        }
        if(p.has_value()){
            m_highlightModel->AddPoint2D(p.value(), coor_t(5), color::blue);
        }

        if(isShowGrid()){
            for(const auto & e : m_triangulation->fixedEdges){
                m_highlightModel->AddSegment2D(TriangulationUtility::GetSegment(*m_triangulation, e), color::red);
            }
        }

        m_highlightModel->SetTransform(m_model->Transform().GetTransform2D());
    }

    UpdateEvaluation();
    update();
}

void SurfaceMeshView::UpdateEvaluation()
{
    const auto & skipT = m_refinement->GetOp().GetRemovedTriangles();
    const auto & skipV = m_refinement->GetOp().GetRemovedVertices();
    TriangleEvaluator evaluator(*m_triangulation, skipT, skipV);
    m_evaluation = evaluator.Report();

    // std::cout << " " << m_iteration;
    // std::cout << " " << evaluator.VertexSize();
    // std::cout << " " << evaluator.TriangleSize();
    // std::cout << " " << evaluator.AveEdgeLength();
    // std::cout << " " << math::Deg(evaluator.AveMinimumAngle());
    // std::cout << std::endl;
}

void SurfaceMeshView::BuildBVH()
{
    m_bvh.reset(new BVH);
}

TetrahedronModelView::TetrahedronModelView(QWidget * parent, Qt::WindowFlags flags)
 : ModelView(parent, flags)
{
    InitFromNodeEle4Files();
}

TetrahedronModelView::~TetrahedronModelView()
{
}

void TetrahedronModelView::InitFromNodeEle4Files()
{
    using namespace geometry;
    QString dirPath = QApplication::applicationDirPath() + "/../../../thirdpart/internal/testdata/elenode";
    QString nodeFile = dirPath + "fccsp.node";
    QString eleFile = dirPath + "fccsp.ele";

    bool res(true);
    std::vector<tet::IndexEle4> eles;
    std::vector<Point3D<coor_t> > points;
    res = res && tet::LoadPointsFromNodeFile(nodeFile.toStdString(), points);
    res = res && tet::LoadElementsFromEleFile(eleFile.toStdString(), eles);
    if(res){
        m_model = makeTetrahedronModelFromNodesAndEle4s(std::move(points), std::move(eles));
    }
}

void TetrahedronModelView::PerformKeyBoardAction(KeyBoardAction ka)
{
    ModelView::PerformKeyBoardAction(ka);
}

void TetrahedronModelView::draw()
{
    ModelView::draw();
}
