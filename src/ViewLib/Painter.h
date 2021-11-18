#ifndef VIEW_PAINTER_H
#define VIEW_PAINTER_H
#include "Config.h"
#include "Model.h"
#include <QOpenGLWidget>
namespace view {
class View3D;
class Painter
{
public:
    Painter(View3D * view);
    ~Painter();

    void DrawTest();
    void DrawText(int x, int y, const QString & text, const QFont & font = QFont(), const QColor & color = Qt::white);
    void DrawAxis(coor_t length, gl_coef_t lineWidth = 2.0);
    void DrawArrow(coor_t length = 1.0, coor_t radius = -1.0, int nDiv = 12);

    void DrawHistogram();
    void DrawMode(const Model * model);
    void DrawFrameModel2D(const FrameModel2D * frameModel, gl_coef_t lineWidth = 1.0);
    void DrawFrameModel3D(const FrameModel3D * frameModel, gl_coef_t lineWidth = 1.0);
    void DrawSurfaceModel2D(const SurfaceModel2D * surfModel, gl_coef_t lineWidth = 1.0);
    void DrawSurfaceModel3D(const SurfaceModel3D * surfModel, gl_coef_t lineWidth = 1.0);
    void DrawTetrahedronModel(const TetrahedronModel * tetModel, gl_coef_t lineWidth = 1.0);
private:
    View3D * m_view;
};
}//namespace view
#endif//VIEW_PAINTER_H
