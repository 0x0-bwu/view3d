#include "View3D.h"
#include "Painter.h"
#include "Camera.h"
#include <QApplication>
#include <QMouseEvent>
#include <QString>
using namespace view;
View3D::View3D(QWidget * parent, Qt::WindowFlags flags)
    : QOpenGLWidget(parent, flags)
    , m_camera(nullptr)
{
    Init();
}

View3D::~View3D()
{
}

void View3D::Init()
{
    SetCamera(new Camera);
    SetPainter(new Painter(this));

    SetDefaultShortcuts();
    SetDefaultMouseBindings();

    m_fps.time.start();
    SetSceneRadius(1.0);
    ShowEntireScene();
    SetFullScreen(false);
}

void View3D::ShowInfo()
{
    //fps
    size_t maxCounter = 20;
    if (++m_fps.count == maxCounter) {
      m_fps.fps = 1000.0 * maxCounter / m_fps.time.restart();
      m_fps.str = QString("%1Hz").arg(m_fps.fps, 0, 'f', ((m_fps.fps < 10.0) ? 1 : 0));
      m_fps.count = 0;
    }

    //rotation
    QString infoQuat, infoTrans;
    Quaternion rot = GetCamera()->GetFrame()->Rotation();
    infoQuat = QString("R: %1+%2i+%3j+%4k") .arg(rot[0], 0, 'g', 3)
                                            .arg(rot[1], 0, 'g', 3)
                                            .arg(rot[2], 0, 'g', 3)
                                            .arg(rot[3], 0, 'g', 3);
    //translation
    vector3d_t trans = GetCamera()->GetFrame()->Translation();
    infoTrans = QString("T: %1, %2, %3").arg(trans[0], 0, 'g', 3)
                                        .arg(trans[1], 0, 'g', 3)
                                        .arg(trans[2], 0, 'g', 3);

    int x = 10, y = 0;
    int fh = QApplication::font().pixelSize() > 0 ? QApplication::font().pixelSize() : QApplication::font().pointSize();
    m_painter->DrawText(x, y + 1.5 * fh, m_fps.str);
    m_painter->DrawText(x, y + 2.5 * fh, infoQuat);
    m_painter->DrawText(x, y + 3.5 * fh, infoTrans);
}

void View3D::ShowAxis()
{
    m_painter->DrawAxis(GetCamera()->SceneRadius());
}

void View3D::PerformClickAction(ClickAction ca, const QMouseEvent * e)
{
    switch(ca) {
    case ClickAction::NoAction :
        break;
    default :
        break;
    }
}

void View3D::PerformKeyBoardAction(KeyBoardAction ka)
{
    switch(ka){
    case KeyBoardAction::ShowAxis :
        SetShowAxis(!m_dispFlags.showAxis);
        break;
    case KeyBoardAction::ShowGrid :
        SetShowGrid(!m_dispFlags.showGrid);
        break;
    case KeyBoardAction::ShowInfo :
        SetShowInfo(!m_dispFlags.showInfo);
        break;
    case KeyBoardAction::FitAll :
        FitAll();
        break;
    case KeyBoardAction::ViewFront :
        SetStandardView(ViewDirection::Front);
        break;
    case KeyBoardAction::ViewEnd :
        SetStandardView(ViewDirection::Back);
        break;
    case KeyBoardAction::ViewTop :
        SetStandardView(ViewDirection::Top);
        break;
    case KeyBoardAction::ViewBot :
        SetStandardView(ViewDirection::Bottom);
        break;
    case KeyBoardAction::ViewLeft :
        SetStandardView(ViewDirection::Left);
        break;
    case KeyBoardAction::ViewRight :
        SetStandardView(ViewDirection::Right);
        break;
    default :
        break;
    }
}

void View3D::SetCamera(Camera * camera)
{
    if(nullptr == camera) return;

    if(m_camera){
        camera->SetSceneRadius(SceneRadius());
        camera->SetSceneCenter(SceneCenter());
        disconnect(GetCamera()->GetFrame(), SIGNAL(manipulated()), this, SLOT(update()));
    }

    m_camera.reset(camera);
    GetCamera()->SetScreenWidthAndHeight(width(), height());
    connect(GetCamera()->GetFrame(), SIGNAL(manipulated()), this, SLOT(update()));
}

void View3D::SetPainter(Painter * painter)
{
    m_painter.reset(painter);
}

void View3D::SetSceneRadius(coef_t radius)
{
    m_camera->SetSceneRadius(radius);
}

coor_t View3D::SceneRadius() const
{
    return m_camera->SceneRadius();
}

point3d_t View3D::SceneCenter() const
{
    return m_camera->SceneCenter();
}

void View3D::resizeGL(int width, int height)
{
    QOpenGLWidget::resizeGL(width, height);
    glViewport(0, 0, GLint(width), GLint(height));
    GetCamera()->SetScreenWidthAndHeight(this->width(), this->height());
}

void View3D::initializeGL()
{
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);

    // Default colors
    SetForegroundColor(QColor(255, 255, 255));
    SetBackgroundColor(QColor(0, 0, 0));

    // Clear the buffer where we're going to draw
    if (format().stereo()) {
        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDrawBuffer(GL_BACK_LEFT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    init();

    if (isFullScreen())
        QTimer::singleShot(100, this, SLOT(DelayedFullScreen()));
}

void View3D::init()
{
    emit viewerInitialized();
}

void View3D::paintGL()
{
    preDraw();
    draw();
    postDraw();
    emit drawFinished(true);
}

void View3D::preDraw()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);

    GetCamera()->LoadProjectionMatrix();
    GetCamera()->LoadModelViewMatrix();
    emit drawNeeded();
}

void View3D::postDraw()
{ 
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    GetCamera()->LoadModelViewMatrix();

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_TEXTURE_1D);
    glDisable(GL_TEXTURE_2D);

    glDisable(GL_TEXTURE_GEN_Q);
    glDisable(GL_TEXTURE_GEN_R);
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);

    glDisable(GL_COLOR_MATERIAL);
    glColor4f(ForegroundColor().redF(), ForegroundColor().greenF(),
                ForegroundColor().blueF(), ForegroundColor().alphaF());

    float color[4];
    color[0] = ForegroundColor().red() / 255.0f;
    color[1] = ForegroundColor().green() / 255.0f;
    color[2] = ForegroundColor().blue() / 255.0f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    //Axis
    if(isShowAxis()) ShowAxis();


    //Info Text
    if(isShowInfo()) ShowInfo();

    // Restore GL state
    glPopAttrib();
    glPopMatrix();

    emit drawFinished(true);
}

void View3D::draw()
{
    m_painter->DrawTest();
}

void View3D::mousePressEvent(QMouseEvent * e)
{
    ClickBinding c(e->modifiers(), e->button(), Qt::NoButton, false, m_currPressedKey);
    if(m_actionBinding.clickBindings.count(c))
        PerformClickAction(m_actionBinding.clickBindings[c], e);
    else{
        MouseBinding m(e->modifiers(), e->button(), m_currPressedKey);
        if(m_actionBinding.mouseBindings.count(m)){
            MouseAction ma = m_actionBinding.mouseBindings[m];
            GetCamera()->GetFrame()->StartAction(ma);
            GetCamera()->GetFrame()->mousePressEvent(e, GetCamera());
        }
    }
}

void View3D::mouseMoveEvent(QMouseEvent * e)
{
    if(GetCamera()->GetFrame()->isManipulated()){
        GetCamera()->GetFrame()->mouseMoveEvent(e, GetCamera());
    }
}

void View3D::mouseReleaseEvent(QMouseEvent * e)
{
    if(GetCamera()->GetFrame()->isManipulated()){
        GetCamera()->GetFrame()->mouseReleaseEvent(e, GetCamera());
    }
    else { e->ignore(); }
    update();
}

void View3D::mouseDoubleClickEvent(QMouseEvent *)
{

}

void View3D::wheelEvent(QWheelEvent * e)
{
    WheelBinding wb(e->modifiers(), m_currPressedKey);
    if(m_actionBinding.wheelBindings.count(wb)){
        MouseAction ma = m_actionBinding.wheelBindings.value(wb);
        GetCamera()->GetFrame()->StartAction(ma);
        GetCamera()->GetFrame()->wheelEvent(e, GetCamera());
    }
    else { e->ignore(); }
}

void View3D::keyPressEvent(QKeyEvent * e)
{
    const Qt::Key key = Qt::Key(e->key());
    auto it = m_actionBinding.keyboardBindings.constBegin();
    for(; it != m_actionBinding.keyboardBindings.constEnd(); ++it){
        if(it.value() == key) PerformKeyBoardAction(it.key());
    }
}

void View3D::keyReleaseEvent(QKeyEvent * e)
{
    Q_UNUSED(e)
    m_currPressedKey = Qt::Key(0);
}

void View3D::timerEvent(QTimerEvent *)
{

}

void View3D::closeEvent(QCloseEvent *)
{

}



void View3D::SetFullScreen(bool fullScreen)
{
    if (m_dispFlags.fullScreen == fullScreen) return;

    m_dispFlags.fullScreen = fullScreen;
    QWidget * top = topLevelWidget();

    if(isFullScreen()) {
        m_prevPos = topLevelWidget()->pos();
        top->showFullScreen();
        top->move(0, 0);
    }
    else {
        top->showNormal();
        top->move(m_prevPos);
    }
}


void View3D::SetBackgroundColor(const QColor & color)
{
    m_backgroundColor = color;
    glClearColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

void View3D::SetForegroundColor(const QColor & color)
{
    m_foregroundColor = color;
}

void View3D::ShowEntireScene()
{
    GetCamera()->ShowEntireScene();
    update();
}

void View3D::SetShowAxis(bool show)
{
    m_dispFlags.showAxis = show;
    update();
}

void View3D::SetShowGrid(bool show)
{
    m_dispFlags.showGrid = show;
    update();
}

void View3D::SetShowInfo(bool show)
{
    m_dispFlags.showInfo = show;
    update();
}

void View3D::SetStandardView(ViewDirection direction)
{
    switch(direction){
    case ViewDirection::Front :
        m_camera->GetFrame()->SetOrientation(0, 0, 0, -1);
        break;
    case ViewDirection::Back :
        m_camera->GetFrame()->SetOrientation(0, 0, 0, 1);
        break;
    case ViewDirection::Top :
        m_camera->GetFrame()->SetOrientation(0, 0, -1, 0);
        break;
    case ViewDirection::Bottom :
        m_camera->GetFrame()->SetOrientation(0, 0, 1, 0);
        break;
    case ViewDirection::Left :
        m_camera->GetFrame()->SetOrientation(0, 1, 0, 0);
        break;
    case ViewDirection::Right :
        m_camera->GetFrame()->SetOrientation(0, -1, 0, 0);
        break;
    default :
        m_camera->GetFrame()->SetOrientation(1, 0, 0, 0);
        break;
    }
    m_camera->GetFrame()->SetTranslation(0, 0, 0);
    FitAll();
}

void View3D::FitAll()
{
    m_camera->ShowEntireScene();
    update();
}

void View3D::DelayedFullScreen()
{
    move(m_prevPos);
    SetFullScreen();
}

void View3D::SetDefaultShortcuts()
{
    m_actionBinding.SetShortcut(KeyBoardAction::ShowAxis,       Qt::Key_A);
    m_actionBinding.SetShortcut(KeyBoardAction::ShowGrid,       Qt::Key_G);
    m_actionBinding.SetShortcut(KeyBoardAction::ShowInfo,       Qt::Key_I);
    m_actionBinding.SetShortcut(KeyBoardAction::FitAll,         Qt::Key_F);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewTop,        Qt::Key_1);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewBot,        Qt::Key_2);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewFront,      Qt::Key_3);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewEnd,        Qt::Key_4);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewLeft,       Qt::Key_5);
    m_actionBinding.SetShortcut(KeyBoardAction::ViewRight,      Qt::Key_6);
    m_actionBinding.SetShortcut(KeyBoardAction::NextAction,     Qt::Key_N);
    m_actionBinding.SetShortcut(KeyBoardAction::PrevAction,     Qt::Key_P);
    m_actionBinding.SetShortcut(KeyBoardAction::TestAction,     Qt::Key_T);
    m_actionBinding.SetShortcut(KeyBoardAction::UpdateAction,   Qt::Key_U);
}

void View3D::SetDefaultMouseBindings()
{
    const Qt::KeyboardModifiers modifiers = Qt::NoModifier;
    m_actionBinding.SetMouseBinding(modifiers, Qt::LeftButton, MouseAction::Translate);
    m_actionBinding.SetMouseBinding(modifiers, Qt::MiddleButton, MouseAction::Rotate);
    m_actionBinding.SetMouseBinding(modifiers, Qt::RightButton, MouseAction::Zoom);

    m_actionBinding.SetWheelBinding(modifiers, MouseAction::Zoom);
}

QImage View3D::FrameBufferSnapshot() {
  makeCurrent();
  raise();
  return QOpenGLWidget::grabFramebuffer();
}
