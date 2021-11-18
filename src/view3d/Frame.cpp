#include "Frame.h"
#include "Camera.h"
#include <QMouseEvent>
#include <QWheelEvent>
using namespace view;
Frame::Frame()
{
}

Frame::Frame(const point3d_t & position, const quaternion_t & orientation)
 : m_trans(position), m_quat(orientation)
{
}

Frame::~Frame()
{

}

void Frame::SetPosition(const point3d_t & position)
{
    SetTranslation(position);
}

void Frame::SetPosition(coor_t x, coor_t y, coor_t z)
{
    SetPosition(point3d_t(x, y, z));
}
void Frame::SetPositionWithConstraint(point3d_t & position)
{
    SetTranslationWithConstraint(position);
}

void Frame::SetOrientation(const quaternion_t & orientation)
{
    SetRotation(orientation);
}

void Frame::SetOrientation(coef_t q0, coef_t q1, coef_t q2, coef_t q3)
{
    SetOrientation(quaternion_t(q0, q1, q2, q3));
}

void Frame::SetOrientationWithConstraint(quaternion_t & orientation)
{
    SetRotationWithConstraint(orientation);
}

void Frame::SetRotation(const quaternion_t & rotation)
{
    m_quat = rotation;

    emit modified();
}

void Frame::SetRotationWithConstraint(quaternion_t & rotation)
{
    quaternion_t deltaQ = Rotation().Inverse() * rotation;
    deltaQ.Normalize();

    SetRotation(Rotation() * deltaQ);
    m_quat.Normalize();
    rotation = Rotation();
}

void Frame::SetPositionAndOrientation(const point3d_t & position, const quaternion_t & orientation)
{
    m_trans = position;
    m_quat = orientation;

    emit modified();
}

void Frame::SetPositionAndOrientationWithConstraint(point3d_t & position, quaternion_t & orientation)
{
    SetTranslationAndRotationWithConstraint(position, orientation);
}

void Frame::SetTranslation(coor_t x, coor_t y, coor_t z)
{
    SetTranslation(vector3d_t(x, y, z));
}

void Frame::SetTranslation(const vector3d_t & translation)
{
   m_trans = translation;
   emit modified();
}

void Frame::SetTranslationWithConstraint(vector3d_t & translation)
{
    vector3d_t deltaT = translation - this->Translation();
    SetTranslation(this->Translation() + deltaT);
    m_trans = Translation();
}

void Frame::SetTranslationAndRotationWithConstraint(vector3d_t & translation, quaternion_t & rotation)
{
    vector3d_t deltaT = translation - Translation();
    quaternion_t deltaQ = Rotation().Inverse() * rotation;

    deltaQ.Normalize();

    m_trans += deltaT;
    m_quat = m_quat * deltaQ;
    m_quat.Normalize();

    translation = Translation();
    rotation = Rotation();

    emit modified();
}

void Frame::Translate(vector3d_t & trans)
{
    m_trans += trans;
    emit modified();
}

void Frame::Translate(const vector3d_t & trans)
{
    vector3d_t t = trans;
    Translate(t);
}

point3d_t Frame::Position() const
{
    return m_trans;
}

quaternion_t Frame::Orientation() const
{
    return Rotation();
}

vector3d_t Frame::TransformOf(const vector3d_t & src) const
{
      return LocalTransformOf(src);
}

vector3d_t Frame::InverseTransformOf(const vector3d_t & src) const
{
    return LocalInverseTransformOf(src);
}

vector3d_t Frame::LocalTransformOf(const vector3d_t & src) const
{
    return Rotation().InverseRotate(src);
}

vector3d_t Frame::LocalInverseTransformOf(const vector3d_t & src) const
{
    return Rotation().Rotate(src);
}

point3d_t Frame::CoordinatesOf(const point3d_t & src) const
{
    return LocalCoordinatesOf(src);
}


point3d_t Frame::InverseCoordinatesOf(const point3d_t & src) const
{
    return LocalInverseCoordinatesOf(src);
}

point3d_t Frame::LocalCoordinatesOf(const point3d_t & src) const
{
    return Rotation().InverseRotate(src - Translation());
}

point3d_t Frame::LocalInverseCoordinatesOf(const point3d_t & src) const
{
    return Rotation().Rotate(src) + Translation();
}

ManipulatedFrame::ManipulatedFrame()
{
    m_action = MouseAction::NoAction;
}

ManipulatedFrame::~ManipulatedFrame()
{
}

void ManipulatedFrame::SetPivotPoint(point3d_t point)
{
    m_pivotPoint= point;
}

bool ManipulatedFrame::isManipulated() const
{
    return m_action != MouseAction::NoAction;
}

void ManipulatedFrame::mousePressEvent(const QMouseEvent * event, const Camera * camera)
{
    Q_UNUSED(camera)
    m_prevPos = m_pressPos = event->pos();
}

void ManipulatedFrame::mouseMoveEvent(const QMouseEvent * event, const Camera * camera)
{
    switch(m_action){
    case MouseAction::Translate : {
        const QPoint delta = m_prevPos - event->pos();
        vector3d_t trans(delta.x(), -delta.y(), 0.0);
        switch(camera->Type()){
        case CameraType::Perspective :
            trans *= 2.0 * std::tan(0.5 * camera->FieldOfView()) *
                    std::fabs((camera->GetFrame()->CoordinatesOf(PivotPoint()))[2]) /
                    camera->ScreenHeight();
            break;
        case CameraType::Orthographic :
            gl_coor_t w, h;
            camera->GetOrthoWidthHeight(w, h);
            trans[0] *= 2.0 * w / camera->ScreenWidth();
            trans[1] *= 2.0 * h / camera->ScreenHeight();
            break;
        }
        Translate(InverseTransformOf(trans * TranslationSensitivity()));
        break;
    }
    case MouseAction::Zoom : {
        Zoom(DeltaWithPrevPos(event, camera), camera);
        break;
    }
    case MouseAction::Rotate : {
        quaternion_t rot;
        if(m_rotAroundUpVec){
            coor_t dx = 2.0 * RotationSensitivity() * (m_prevPos.x() - event->x()) /
                    camera->ScreenWidth();
            coor_t dy = 2.0 * RotationSensitivity() * (m_prevPos.y() - event->y()) /
                    camera->ScreenHeight();
            if(m_constrainedRotReversed) dx = -dx;
            vector3d_t verticalAxis = TransformOf(m_sceneUpVec);
            rot = quaternion_t(verticalAxis, dx) * quaternion_t(vector3d_t(1, 0, 0), dy);
        }
        else{
            vector3d_t trans = camera->ProjectedCoordinatesOf(PivotPoint());
            rot = DeformedBallQuaternion(event->x(), event->y(), trans[0], trans[1], camera);
        }
        Rotate(rot);
        break;
    }
    default :
        break;
    }

    if(m_action != MouseAction::NoAction){
        m_prevPos = event->pos();

        emit manipulated();
    }
}
void ManipulatedFrame::mouseReleaseEvent(const QMouseEvent * event, const Camera * camera)
{
    m_action = MouseAction::NoAction;
}

void ManipulatedFrame::mouseDoubleClickEvent(const QMouseEvent * event, const Camera * camera)
{

}

void ManipulatedFrame::wheelEvent(const QWheelEvent * event, const Camera * camera)
{
    switch(m_action){
    case MouseAction::Zoom : {
        Zoom(WheelDelta(event), camera);
        emit manipulated();
        break;
    }
    default :
        break;
    }
    m_action = MouseAction::NoAction;
}

void ManipulatedFrame::StartAction(MouseAction ma)
{
    m_action = ma;
    switch(m_action){
    case MouseAction::Rotate:
        m_constrainedRotReversed = TransformOf(m_sceneUpVec)[1] < 0;
        break;
    default:
        break;
    }
}

quaternion_t ManipulatedFrame::DeformedBallQuaternion(int x, int y, coor_t cx, coor_t cy, const Camera * camera)
{
    coor_t px = RotationSensitivity() * (m_prevPos.x() - cx) / camera->ScreenWidth();
    coor_t py = RotationSensitivity() * (cy - m_prevPos.y()) / camera->ScreenHeight();
    coor_t dx = RotationSensitivity() * (x - cx) / camera->ScreenWidth();
    coor_t dy = RotationSensitivity() * (cy - y) / camera->ScreenHeight();

    point3d_t p1(px, py, CalculateProjectOnBall(px, py));
    point3d_t p2(dx, dy, CalculateProjectOnBall(dx, dy));

    vector3d_t axis = p2.CrossProduct(p1);
    coef_t angle = 5.0 * std::asin(std::sqrt(axis.NormSquare() / p1.NormSquare() / p2.NormSquare()));
    return quaternion_t(axis, angle);
}

coef_t ManipulatedFrame::DeltaWithPrevPos(const QMouseEvent * event, const Camera * camera) const
{
    coor_t dx = coor_t(event->x() - m_prevPos.x()) / camera->ScreenWidth();
    coor_t dy = coor_t(event->y() - m_prevPos.y()) / camera->ScreenHeight();
    coor_t delta = std::fabs(dx) > std::fabs(dy) ? dx : dy;
    return delta * ZoomSensitivity();
}

coef_t ManipulatedFrame::WheelDelta(const QWheelEvent * event) const
{
    return event->delta() * m_sens.wheel;
}

void ManipulatedFrame::Zoom(coef_t delta, const Camera * camera)
{
    coor_t sceneRadius = camera->SceneRadius();
    if(m_zoomOnPivotPoint){
        vector3d_t direction = Position() - camera->PivotPoint();
        if(direction.Norm2() > 0.02 * sceneRadius || delta > 0)
            Translate(direction * delta);
    }
    else {
        coor_t z = camera->GetFrame()->CoordinatesOf(camera->PivotPoint())[2];
        coef_t coef = std::max(std::fabs(z), 0.2 * sceneRadius);
        vector3d_t trans(0.0, 0.0, -delta * coef);
        Translate(InverseTransformOf(trans));
    }
}

void ManipulatedFrame::RotateAroundPoint(quaternion_t rot, point3d_t pos)
{
    m_quat *= rot;
    m_quat.Normalize();
    vector3d_t trans = pos + quaternion_t(InverseTransformOf(rot.Axis()), rot.Angle()).Rotate(Position() - pos) - m_trans;
    m_trans += trans;

    emit modified();
}

void ManipulatedFrame::Rotate(quaternion_t rot)
{
    RotateAroundPoint(rot, PivotPoint());
}
