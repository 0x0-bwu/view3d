#include "Camera.h"
#include <QOpenGLWidget>
#include "GL/glu.h"
using namespace view;
Camera::Camera()
{
    SetFrame(new ManipulatedFrame);
    SetSceneRadius(1.0);

    m_params.orthoCoef = std::tan(0.5 * FieldOfView());

    for(size_t i = 0; i < 16; ++i){
        m_params.modelViewMatrix[i] = (i % 5 == 0) ? 1.0 : 0.0;
        m_params.projectionMatrix[i] = 0.0;
    }
    ComputeProjectionMatrix();
}

Camera::~Camera()
{
}

point3d_t Camera::Position() const
{
    return GetFrame()->Position();
}

vector3d_t Camera::UpVector() const
{
    return GetFrame()->InverseTransformOf(vector3d_t(0, 1, 0));
}

vector3d_t Camera::ViewDirection() const
{
    return GetFrame()->InverseTransformOf(vector3d_t(0, 0, -1));
}

vector3d_t Camera::RightVector() const
{
    return GetFrame()->InverseTransformOf(vector3d_t(1, 0, 0));
}


quaternion_t Camera::Orientation() const
{
    return GetFrame()->Orientation();
}

coor_t Camera::zNear() const
{
    const coor_t zNearScene = zClippingCoefficient() * SceneRadius();
    coor_t z = DistanceToSceneCenter() - zNearScene;

    const coor_t zMin = zNearCoefficient() * zNearScene;
    if(z < zMin){
        switch (Type()) {
        case CameraType::Perspective : {
            z = zMin;
            break;
        }
        case CameraType::Orthographic : {
            z = 0.0;
            break;
        }
        }
    }
    return z;
}

coor_t Camera::zFar() const
{
    return DistanceToSceneCenter() + zClippingCoefficient() * SceneRadius();
}

void Camera::GetOrthoWidthHeight(gl_coor_t & halfWidth, gl_coor_t & halfHeight) const
{
    const coor_t dist = m_params.orthoCoef * fabs(CameraCoordinatesOf(PivotPoint())[2]);
    halfWidth  = dist * ((AspectRatio() < 1.0) ? 1.0 : AspectRatio());
    halfHeight = dist * ((AspectRatio() < 1.0) ? 1.0 / AspectRatio() : 1.0);
}

coef_t Camera::AspectRatio() const
{
    return coef_t(ScreenWidth()) / ScreenHeight();
}

void Camera::GetViewport(GLint viewport[4]) const
{
    viewport[0] = 0;
    viewport[1] = ScreenHeight();
    viewport[2] = ScreenWidth();
    viewport[3] = -ScreenHeight();
}

point3d_t Camera::CameraCoordinatesOf(const point3d_t & src) const
{
    return GetFrame()->CoordinatesOf(src);
}

point3d_t Camera::WorldCoordinatesOf(const point3d_t & src) const
{
    return GetFrame()->InverseCoordinatesOf(src);
}

coor_t Camera::DistanceToSceneCenter() const
{
    point3d_t p = GetFrame()->CoordinatesOf(SceneCenter());
    return std::fabs(p[2]);
}

coef_t Camera::HorizontalFieldOfView() const
{
    return 2.0 * atan(tan(0.5 * FieldOfView() * AspectRatio()));
}

void Camera::SetScreenWidthAndHeight(int width, int height)
{
    m_screen[0] = width  > 0 ? width  : 1;
    m_screen[1] = height > 0 ? height : 1;
    m_params.projectionMatrixIsUpToDate = false;
}

void Camera::SetSceneRadius(coef_t radius)
{
    assert(radius > 0.0);
    m_params.sceneRadius = radius;
    m_params.projectionMatrixIsUpToDate = false;

    SetFocusDistance(SceneRadius() / tan(0.5 * FieldOfView()));
}

void Camera::SetSceneCenter(point3d_t center)
{
    m_params.screenCenter = center;
    SetPivotPoint(SceneCenter());
    m_params.projectionMatrixIsUpToDate = false;
}

void Camera::SetPivotPoint(point3d_t point)
{
    coor_t prevDist = std::fabs(CameraCoordinatesOf(PivotPoint())[2]);
    GetFrame()->SetPivotPoint(point);

    coor_t currDist = std::fabs(CameraCoordinatesOf(PivotPoint())[2]);
    if(prevDist > constant::epsilon && currDist > constant::epsilon)
        m_params.orthoCoef *= prevDist / currDist;
    m_params.projectionMatrixIsUpToDate = false;
}


void Camera::SetFocusDistance(coor_t distance)
{
    m_focusDistance = distance;
}

void Camera::SetFrame(ManipulatedFrame * frame)
{
    if(nullptr == frame) return;

    if(m_frame)
        disconnect(GetFrame(), SIGNAL(modified()), this, SLOT(OnFrameModified()));

    m_frame.reset(frame);
    connect(GetFrame(), SIGNAL(modified()), this, SLOT(OnFrameModified()));
    OnFrameModified();
}

void Camera::LoadProjectionMatrix(bool reset) const
{
    glMatrixMode(GL_PROJECTION);

    if(reset)
        glLoadIdentity();

    ComputeProjectionMatrix();
    glMultMatrixd(m_params.projectionMatrix);
}

void Camera::LoadModelViewMatrix(bool reset) const
{
    glMatrixMode(GL_MODELVIEW);
    ComputeModelViewMatrix();
    if(reset)
        glLoadMatrixd(m_params.modelViewMatrix);
    else
        glMultMatrixd(m_params.modelViewMatrix);
}

void Camera::ComputeProjectionMatrix() const
{
    if (m_params.projectionMatrixIsUpToDate) return;

    const coor_t near = zNear();
    const coor_t far = zFar();
    switch (Type()) {
    case CameraType::Perspective : {
        const coor_t f = 1.0 / tan(0.5 * FieldOfView());
        m_params.projectionMatrix[0 ] = f / AspectRatio();
        m_params.projectionMatrix[5 ] = f;
        m_params.projectionMatrix[10] = (near + far) / (near - far);
        m_params.projectionMatrix[11] = -1.0;
        m_params.projectionMatrix[14] = 2.0 * near * far / (near - far);
        m_params.projectionMatrix[15] = 0.0;
        break;
    }
    case CameraType::Orthographic : {
        gl_coor_t w, h;
        GetOrthoWidthHeight(w, h);
        m_params.projectionMatrix[0 ] = 1.0 / w;
        m_params.projectionMatrix[5 ] = 1.0 / h;
        m_params.projectionMatrix[10] = -2.0 / (zFar() - near);
        m_params.projectionMatrix[11] = 0.0;
        m_params.projectionMatrix[14] = -(far + near) / (far - near);
        m_params.projectionMatrix[15] = 1.0;
        break;
    }
  }

  m_params.projectionMatrixIsUpToDate = true;
}

void Camera::ComputeModelViewMatrix() const
{
    if (m_params.modelViewMatrixIsUpToDate) return;

    const quaternion_t q = GetFrame()->Orientation();

    const coef_t q11 = 2.0 * q[1] * q[1];
    const coef_t q22 = 2.0 * q[2] * q[2];
    const coef_t q33 = 2.0 * q[3] * q[3];

    const coef_t q12 = 2.0 * q[1] * q[2];
    const coef_t q13 = 2.0 * q[1] * q[3];
    const coef_t q10 = 2.0 * q[1] * q[0];

    const coef_t q23 = 2.0 * q[2] * q[3];
    const coef_t q20 = 2.0 * q[2] * q[0];

    const coef_t q30 = 2.0 * q[3] * q[0];

    m_params.modelViewMatrix[ 0] = 1.0 - q22 - q33;
    m_params.modelViewMatrix[ 1] = q12 - q30;
    m_params.modelViewMatrix[ 2] = q13 + q20;
    m_params.modelViewMatrix[ 3] = 0.0;

    m_params.modelViewMatrix[ 4] = q12 + q30;
    m_params.modelViewMatrix[ 5] = 1.0 - q33 - q11;
    m_params.modelViewMatrix[ 6] = q23 - q10;
    m_params.modelViewMatrix[ 7] = 0.0;

    m_params.modelViewMatrix[ 8] = q13 - q20;
    m_params.modelViewMatrix[ 9] = q23 + q10;
    m_params.modelViewMatrix[10] = 1.0 - q22 - q11;
    m_params.modelViewMatrix[11] = 0.0;

    const vector3d_t t = q.InverseRotate(GetFrame()->Position());

    m_params.modelViewMatrix[12] = -t[0];
    m_params.modelViewMatrix[13] = -t[1];
    m_params.modelViewMatrix[14] = -t[2];
    m_params.modelViewMatrix[15] = 1.0;

    m_params.modelViewMatrixIsUpToDate = true;
}


void Camera::FitSphere(const point3d_t & center, coor_t radius)
{
    coor_t distance = 0.0;
    switch(Type()){
        case CameraType::Perspective : {
            coor_t x = radius / sin(0.5 * FieldOfView());
            coor_t y = radius / sin(0.5 * HorizontalFieldOfView());
            distance = std::max(x, y);
            break;
        }
        case CameraType::Orthographic : {
            distance = (center - PivotPoint()).Dot(ViewDirection()) + (radius / m_params.orthoCoef);
            break;
        }
    }
    point3d_t newPos(center - ViewDirection() * distance);
    GetFrame()->SetPositionWithConstraint(newPos);
}

point3d_t Camera::PointUnderPixel(const QPoint & pixel, bool & found) const
{
    float depth;
    glReadPixels(pixel.x(), ScreenHeight() - 1 - pixel.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    found = depth < 1.0f;
    point3d_t point(pixel.x(), pixel.y(), depth);
    point = UnprojectedCoordinatesOf(point);
    return point;
}

point3d_t Camera::ProjectedCoordinatesOf(const point3d_t & src) const
{
    gl_coor_t x, y, z;
    GLint viewport[4];
    GetViewport(viewport);

    gluProject(src[0], src[1], src[2], m_params.modelViewMatrix,
            m_params.projectionMatrix, viewport, &x, &y, &z);
    return point3d_t(x, y, z);
}

point3d_t Camera::UnprojectedCoordinatesOf(const point3d_t & src) const
{
    gl_coor_t x, y, z;
    static GLint viewport[4];
    GetViewport(viewport);
    gluUnProject(src[0], src[1], src[2], m_params.modelViewMatrix, m_params.projectionMatrix, viewport, &x, &y, &z);
    return point3d_t(x, y, z);
}

void Camera::OnFrameModified()
{
    m_params.projectionMatrixIsUpToDate = false;
    m_params.modelViewMatrixIsUpToDate = false;
}

void Camera::ShowEntireScene()
{
    FitSphere(SceneCenter(), SceneRadius());
}
