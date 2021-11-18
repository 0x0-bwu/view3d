#ifndef VIEW_VIEW3D_H
#define VIEW_VIEW3D_H
#include "Config.h"
#include "Action.h"
#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QMap>
#include <memory>
namespace view {

class Camera;
class Painter;
class ManipulatedFrame;
class View3D : public QOpenGLWidget
{
    Q_OBJECT
    struct DisplayFlags
    {
        bool showGrid = false;
        bool showAxis = false;
        bool showInfo = true;
        bool fullScreen = false;
    };

    struct ViewFps {
        coef_t fps = 0.0;
        size_t count = 0;
        QString str = "1Hz";
        QElapsedTimer time;
    };

public:
    View3D(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::Dialog);
    virtual ~View3D();

public:
    void SetCamera(Camera * camera);
    void SetPainter(Painter * painter);
    void SetSceneRadius(coef_t radius);

    Camera * GetCamera() { return m_camera.get(); }
    bool isFullScreen() const { return m_dispFlags.fullScreen; }
    coor_t SceneRadius() const;
    point3d_t SceneCenter() const;

    QColor ForegroundColor() const { return m_foregroundColor; }
    QColor BackgroundColor() const { return m_backgroundColor; }

    bool isShowAxis() const { return m_dispFlags.showAxis; }
    bool isShowInfo() const { return m_dispFlags.showInfo; }
    bool isShowGrid() const { return m_dispFlags.showGrid; }

    void SetDefaultShortcuts();
    void SetDefaultMouseBindings();
    QImage FrameBufferSnapshot();

    virtual QSize sizeHint() const;

public slots:
    void SetFullScreen(bool fullScreen = true);
    void SetBackgroundColor(const QColor & color);
    void SetForegroundColor(const QColor & color);
    void ShowEntireScene();
    void SetShowAxis(bool show = true);
    void SetShowGrid(bool show = true);
    void SetShowInfo(bool show = true);
    void SetStandardView(ViewDirection direction);
    void FitAll();

signals:
    void viewerInitialized();
    void drawNeeded();
    void drawFinished(bool automatic);

protected:
    virtual void resizeGL(int width, int height);
    virtual void initializeGL();
    virtual void init();

    virtual void paintGL();
    virtual void preDraw();
    virtual void postDraw();
    virtual void draw();

protected:
    virtual void mousePressEvent(QMouseEvent *);
    virtual void mouseMoveEvent(QMouseEvent *);
    virtual void mouseReleaseEvent(QMouseEvent *);
    virtual void mouseDoubleClickEvent(QMouseEvent *);
    virtual void wheelEvent(QWheelEvent *);
    virtual void keyPressEvent(QKeyEvent *);
    virtual void keyReleaseEvent(QKeyEvent *);
    virtual void timerEvent(QTimerEvent *);
    virtual void closeEvent(QCloseEvent *);

protected:
    virtual void Init();
    virtual void ShowInfo();
    virtual void ShowAxis();

    virtual void PerformClickAction(ClickAction ca, const QMouseEvent * e);
    virtual void PerformKeyBoardAction(KeyBoardAction ka);

protected slots:
    virtual void DelayedFullScreen();

protected:
    std::unique_ptr<Camera> m_camera;
    std::unique_ptr<Painter> m_painter;
    ViewFps m_fps;
    ActionBinding m_actionBinding;
    Qt::Key m_currPressedKey = Qt::Key(0);
    bool m_cameraIsEdited;
    QColor m_backgroundColor;
    QColor m_foregroundColor;
    DisplayFlags m_dispFlags;
    QPoint m_prevPos;
};

inline QSize View3D::sizeHint() const
{
    return QSize(600, 400);
}
}//namespace view
#endif//VIEW_VIEW3D_H
