#include "ModelView.h"
#include "Camera.h"
#include "generic/geometry/TetrahedralizationIO.hpp"
#include "generic/geometry/TriangleEvaluator.hpp"
#include "generic/geometry/Connectivity.hpp"
#include "generic/tree/BVHUtilityMT.hpp"
#include "generic/tools/Tools.hpp"
#include "Painter.h"
#include "Mesher2D.h"
#include "MeshFileUtility.h"
#include <QApplication>
#include <QMouseEvent>
#include <QFileInfo>
#include <QString>
#include <QDebug>
using namespace view;
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
    InitFromWktFile();
    // InitFromPolyFile();
    // InitFromDomDmcFile();
    // InitFromConnectivityTest();
}

FrameModelView::~FrameModelView()
{

}

void FrameModelView::InitFromWktFile()
{
    using namespace generic::geometry;
    QString fileName = QApplication::applicationDirPath() + "/../../../test/wkt/layer_1.wkt";
    fileName = QFileInfo(fileName).canonicalFilePath();
    std::vector<PolygonWithHoles2D<coor_t> > outs;
    if(GeometryIO::Read<PolygonWithHoles2D<coor_t> >(fileName.toStdString(), std::back_inserter(outs))){
        auto trans = makeScaleTransform2D<coor_t>(1e-3);
        for(auto & out : outs) {
            Polygon2D<coor_t> simplified;
            boost::geometry::simplify(out.outline, simplified, 100);
            out.outline = simplified;
            for(auto & hole : out.holes){
                Polygon2D<coor_t> simplified;
                boost::geometry::simplify(hole, simplified, 100);
                hole = simplified;
            }
            out = trans * out;
        }
        m_model = makeFrameModel3DFromPolygonWithHoles2D<PolygonWithHoles2D<coor_t> >(outs.begin(), outs.end(), coor_t(0), coor_t(0.25 * 1e3));

        auto model3d = dynamic_cast<FrameModel3D *>(m_model.get());
        if(model3d){
            QString outFile = QApplication::applicationDirPath() + "/../../../test/tet/layer_1";
            auto plc = model3d->toPLC(true);
            tet::WritePlcToNodeAndEdgeFiles(outFile.toStdString(), plc);
            model3d->Normalize();//wbtest
        }
    }
}

void FrameModelView::InitFromPolyFile()
{
    using namespace geometry;
    using namespace geometry::tet;
    QString fileName = QApplication::applicationDirPath() + "/../../../test/tet/test.poly";
    fileName = QFileInfo(fileName).canonicalFilePath();

    PiecewiseLinearComplex<Point3D<double> > plc;
    if(LoadPlcFromPolyFile(fileName.toStdString(), plc)){
        m_model = makeFrameModel3DFromPiecewiseLinearComplex(plc);
    }
}

void FrameModelView::InitFromConnectivityTest()
{
    using namespace geometry;

    size_t layers = 39;
    size_t geometries = 0;
    std::vector<std::vector<PolygonWithHoles2D<int64_t> > > layerPwhs(layers);
    QString baseName = QApplication::applicationDirPath() + "/../../../test/wkt/odb_";
    for(size_t i = 0; i < layers; ++i){
        std::string filename = baseName.toStdString() + std::to_string(i + 1) + ".wkt";
        GeometryIO::Read<PolygonWithHoles2D<int64_t> >(filename, std::back_inserter(layerPwhs[i]));
        std::cout << "read "<< layerPwhs[i].size() << " geometries from " << filename << std::endl;
        geometries += layerPwhs[i].size();
    }
    std::cout << "total geometries: " << geometries << std::endl;

    auto pwhGetter  = [](const PolygonWithHoles2D<int64_t> & pwh) { return pwh; };
    ConnectivityExtractor<int64_t> extractor;

    int64_t layerH = -500000;
    std::vector<int64_t> zRef(layers);
    zRef[0] = 0;
    for(size_t i = 1; i < layers; ++i)
        zRef[i] += zRef[i - 1] + layerH;
    std::unordered_map<size_t, std::pair<size_t, size_t> > idxMap;
    for(size_t i = 0; i < layers; ++i){
        auto [begin, end] = extractor.AddObjects(i, layerPwhs[i].begin(), layerPwhs[i].end(), pwhGetter);
        extractor.AddLayerConnection(i, i + 1);
        for(size_t id = begin, j = 0; id <= end; ++id, ++j){
            idxMap.insert(std::make_pair(id, std::make_pair(i, j)));
        }
    }

    std::unique_ptr<GeomConnGraph> graph = nullptr;
    {
        generic::tools::ProgressTimer t;
        graph = extractor.Extract(32);
    }

    if(nullptr == graph){
        std::cout << "fail to extract" << std::endl;
    }
    else{
        std::cout << "extract successful" << std::endl;
        m_model = std::unique_ptr<FrameModel3D>(new FrameModel3D);
        std::list<index_t> c;
        std::vector<std::list<index_t> > cc;
        topology::ConnectedComponent(*graph, 473, c);
        topology::ConnectedComponents(*graph, cc);
        // std::cout << "component:";
        // for(auto i : c) std::cout << " " << i;
        // std::cout << std::endl;

        std::vector<size_t> indices;
        for(size_t i = 0; i < cc.size(); ++i){
            indices.push_back(i);
            // std::cout << "cc " << i + 1 << ":";
            // for(auto j : cc[i]) std::cout << " " << j;
            // std::cout << std::endl; 
        }

        auto sortFunc = [&cc](size_t i, size_t j){return cc[i].size() > cc[j].size(); };
        std::sort(indices.begin(), indices.end(), sortFunc);
        
        FrameModel3D * model3D = dynamic_cast<FrameModel3D *>(m_model.get());
        if(model3D){
            for(auto i : cc[indices[2]]){
                auto [layer, index] = idxMap.at(i);
                model3D->AddPolygonWithHoles3D(layerPwhs[layer][index], zRef[layer], layerH, color::red);
            }

            for(auto i : cc[indices[3]]){
                auto [layer, index] = idxMap.at(i);
                model3D->AddPolygonWithHoles3D(layerPwhs[layer][index], zRef[layer], layerH, color::green);
            }

            for(auto i : cc[indices[4]]){
                auto [layer, index] = idxMap.at(i);
                model3D->AddPolygonWithHoles3D(layerPwhs[layer][index], zRef[layer], layerH, color::blue);
            }

            for(size_t layer = 0; layer < layers; ++layer){
                if(layer != layers - 1) continue;
                const auto & pwhs = layerPwhs[layer];
                for(const auto & pwh : pwhs)
                    model3D->AddPolygonWithHoles3D(pwh, zRef[layer], layerH, color::white);
            }
            model3D->Normalize();
        }
    }
}

void FrameModelView::InitFromDomDmcFile()
{
    using namespace emesh;
    using namespace geometry;
    QString fileName = QApplication::applicationDirPath() + "/../../../test/mesh/2";
    fileName = QFileInfo(fileName).absoluteFilePath();

    std::list<Polygon2D<int64_t> > polygons;
    MeshFlow2D::LoadGeometryFiles(fileName.toStdString(), FileFormat::DomDmc, 100, polygons);
    m_model = makeFrameModelFromPolygon2D<Polygon2D<int64_t> >(polygons.begin(), polygons.end());
}

SurfaceModelView::SurfaceModelView(QWidget * parent, Qt::WindowFlags flags)
 : ModelView(parent, flags)
{
     InitFromNodeEdgeFiles();
//    InitFromNodeEle4Files();
}

SurfaceModelView::~SurfaceModelView()
{
}

void SurfaceModelView::InitFromNodeEdgeFiles()
{
    using namespace emesh;
    using namespace geometry;
    QString dirPath = QApplication::applicationDirPath() + "/../../../test/tet/";
    QString nodeFile = dirPath + "fccsp.node";
    nodeFile = QFileInfo(nodeFile).canonicalFilePath();
    QString edgeFile = dirPath + "fccsp.edge";
    edgeFile = QFileInfo(edgeFile).canonicalFilePath();

    bool res(true);
    std::vector<tet::IndexEdge> edges;
    std::vector<Point3D<coor_t> > points;
    res = res && tet::LoadPointsFromNodeFile(nodeFile.toStdString(), points);
    res = res && tet::LoadEdgesFromEdgeFile(edgeFile.toStdString(), edges);
    if(res){
        m_model = makeSurfaceModelFromNodesAndEdges(std::move(points), edges);
    }
}

void SurfaceModelView::InitFromNodeEle4Files()
{
    using namespace emesh;
    using namespace geometry;
    QString dirPath = QApplication::applicationDirPath() + "/../../../test/tet/results/";
    QString nodeFile = dirPath + "result.node";
    QString eleFile = dirPath + "result.ele";

    bool res(true);
    std::vector<tet::IndexEle4> eles;
    std::vector<Point3D<coor_t> > points;
    res = res && tet::LoadPointsFromNodeFile(nodeFile.toStdString(), points);
    res = res && tet::LoadElementsFromEleFile(eleFile.toStdString(), eles);
    if(res){
        m_model = makeSurfaceModelFromNodesAndEle4s(std::move(points), eles);
    }
}

void SurfaceModelView::PerformKeyBoardAction(KeyBoardAction ka)
{
    ModelView::PerformKeyBoardAction(ka);
}

void SurfaceModelView::draw()
{
    ModelView::draw();
}

SurfaceMeshView::SurfaceMeshView(QWidget * parent, Qt::WindowFlags flags)
 : SurfaceModelView(parent, flags)
 , m_triangulation(new Triangulation)
{
    //InitFromWktFile();
    InitFromMshFile();
    //InitFromDomDmcFile();
    //InitFromTopologyFile();
    //InitFromTriangulationArchive(true);
    UpdateModels();
}

SurfaceMeshView::~SurfaceMeshView()
{
}

void SurfaceMeshView::InitFromWktFile()
{
    using namespace generic::geometry;
    QString fileName = QApplication::applicationDirPath() + "/../../../test/wkt/layer_1.wkt";
    fileName = QFileInfo(fileName).canonicalFilePath();
    std::vector<PolygonWithHoles2D<coor_t> > outs;
    if(GeometryIO::Read<PolygonWithHoles2D<coor_t> >(fileName.toStdString(), std::back_inserter(outs))){

        geometry::GeoTopology2D<coor_t> geoTopo;
        for(auto iter_pwh = outs.begin(); iter_pwh != outs.end(); ++iter_pwh){
            const auto & pwh = *iter_pwh;
            geoTopo.AddGeometry(pwh);
        }
        PreTriangulation(geoTopo, 500);
    }
    else {
        qDebug() << "Error: fail to load file: " << fileName;
    }
    m_refinement.reset(new CurrentRefineMethod(*m_triangulation));
    m_refinement->SetParas(math::Rad(20), 25, 1e10);
}

void SurfaceMeshView::InitFromMshFile()
{
    using namespace generic::geometry;
    QString fileName = QApplication::applicationDirPath() + "/../../../test/msh/test.msh";
    fileName = QFileInfo(fileName).canonicalFilePath();

    emesh::MeshFileUtility::ImportMshFile(fileName.toStdString(), *m_triangulation, 1e6);
    m_refinement.reset(new CurrentRefineMethod(*m_triangulation));
    m_refinement->SetParas(math::Rad(20), 25, 1e10);    
}

void SurfaceMeshView::InitFromDomDmcFile()
{
    using namespace emesh;
    using namespace geometry;
    
    MeshCtrl meshCtrl;
    meshCtrl.scale2Int = 100;
    meshCtrl.tolerance = 10;

    QString fileName = QApplication::applicationDirPath() + "/../../../test/mesh/wei";
    fileName = QFileInfo(fileName).absoluteFilePath();

    auto polygons = std::make_unique<std::list<Polygon2D<coor_t> > >();
    MeshFlow2D::LoadGeometryFiles(fileName.toStdString(), FileFormat::DomDmc, meshCtrl.scale2Int, *polygons);
    auto outline = ConvexHull(*polygons);
    // polygons->push_back(outline);

    Box2D<coor_t> bbox = Extent(outline);
    // polygons->push_back(toPolygon(bbox));//wbtest
    meshCtrl.minEdgeLen = 30;
    // meshCtrl.minEdgeLen = std::max(bbox.Length(), bbox.Width()) / 3000;
    meshCtrl.maxEdgeLen = std::max(bbox.Length(), bbox.Width()) / 3;
    meshCtrl.minAlpha = math::Rad(30);

    auto segments = std::make_unique<std::vector<Segment2D<coor_t> > >();
    MeshFlow2D::ExtractIntersections(*polygons, *segments);
    polygons.reset();
    
    auto points = std::make_unique<std::vector<Point2D<coor_t> > >();
    auto edges = std::make_unique<std::list<IndexEdge> >();
    MeshFlow2D::ExtractTopology(*segments, *points, *edges);
    segments.reset();

    MeshFlow2D::MergeClosePointsAndRemapEdge(*points, *edges, meshCtrl.tolerance);
//    MeshFlow2D::SplitOverlengthEdges(*points, *edges, meshCtrl.maxEdgeLen);

//    size_t threshold = 10;//wbtest
//    MeshFlow2D::AddPointsFromBalancedQuadTree(outline, *points, threshold);

    MeshFlow2D::TriangulatePointsAndEdges(*points, *edges, *m_triangulation);
    points.reset();
    edges.reset();

    m_refinement.reset(new CurrentRefineMethod(*m_triangulation));
    m_refinement->SetParas(math::Rad(15), meshCtrl.minEdgeLen, meshCtrl.maxEdgeLen);
}

void SurfaceMeshView::InitFromTopologyFile()
{
    using namespace geometry;
    geometry::GeoTopology2D<coor_t> geoTopo;
    QString topFile = QApplication::applicationDirPath() + "/../../../test/topology/test.top";
    geoTopo.Read(topFile.toStdString());
    PreTriangulation(geoTopo);
    m_refinement.reset(new CurrentRefineMethod(*m_triangulation));
    m_refinement->SetParas(math::Rad(27), 5, 50);
}

void SurfaceMeshView::InitFromTriangulationArchive(bool loadRefineState)
{
    using namespace geometry;
    // QString archive = QApplication::applicationDirPath() + "/../../../test/archive.txt";
    // geometry::tri::TriangulationUtility<Point2D<coor_t> >::Read(archive.toStdString(), *m_triangulation);
    // m_refinement.reset(new DelaunayRefinement(*m_triangulation));
    // if(loadRefineState){
    //     QString state = QApplication::applicationDirPath() + "/../../../test/state.txt";
    //     m_refinement->RestoreState(state.toStdString());
    // }
}

void SurfaceMeshView::PreTriangulation(const geometry::GeoTopology2D<coor_t> & geoTopo, coor_t mergePointDist)
{
    using IndexEdge = geometry::tri::IndexEdge;
    using Edge = std::pair<size_t, size_t>;
    using Point = Point2D<coor_t>;

    std::list<Edge > edges;
    geoTopo.GetAllEdges(edges);
    std::vector<Point> points = geoTopo.GetAllPoints();

    try {
        std::list<IndexEdge> indexEdges;
        for(const auto & edge : edges) indexEdges.emplace_back(IndexEdge(edge.first, edge.second));

//        geometry::tri::RemoveDuplicatesAndRemapEdges(points, indexEdges, coor_t(10));

        Triangulator2D triangulator(*m_triangulation);
        triangulator.InsertVertices(points.begin(), points.end(), [](const Point & p){ return p[0]; }, [](const Point & p){ return p[1]; });
        triangulator.InsertEdges(indexEdges.begin(), indexEdges.end(), [](const IndexEdge & e){ return e.v1(); }, [](const IndexEdge & e){ return e.v2(); });
        triangulator.EraseOuterTriangles();
//        triangulator.EraseSuperTriangle();
    }
    catch (...) {
        qDebug() << "Error: fail to trangulate the surface.";
        m_triangulation->Clear();
    }
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

        m_highlightModel->SetTransform(m_model->Transform().GetTransfrom2D());
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
    QString dirPath = QApplication::applicationDirPath() + "/../../../test/tet/";
    QString nodeFile = dirPath + "fccsp.1.node";
    QString eleFile = dirPath + "fccsp.1.ele";

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
