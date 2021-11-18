#ifndef VIEW_FRAME_H
#define VIEW_FRAME_H
#include "Config.h"
#include "Action.h"
#include <QElapsedTimer>
#include <QObject>
#include <QPoint>
#include <QTimer>
class QMouseEvent;
class QWheelEvent;
namespace view {
class View3D;
class Camera;
class Constraint;
class Frame : public QObject
{
    Q_OBJECT
public:
    Frame();
    Frame(const point3d_t & position, const quaternion_t & orientation);
    virtual ~Frame();

    void SetPosition(const point3d_t & position);
    void SetPosition(coor_t x, coor_t y, coor_t z);
    void SetPositionWithConstraint(point3d_t & position);

    void SetOrientation(const quaternion_t & orientation);
    void SetOrientation(coef_t q0, coef_t q1, coef_t q2, coef_t q3);
    void SetOrientationWithConstraint(quaternion_t & orientation);

    void SetRotation(const quaternion_t & rotation);
    void SetRotationWithConstraint(quaternion_t & rotation);

    void SetPositionAndOrientation(const point3d_t & position, const quaternion_t & orientation);
    void SetPositionAndOrientationWithConstraint(point3d_t & position, quaternion_t & orientation);

    void SetTranslation(coor_t x, coor_t y, coor_t z);
    void SetTranslation(const vector3d_t & translation);
    void SetTranslationWithConstraint(vector3d_t & translation);
    void SetTranslationAndRotationWithConstraint(vector3d_t & translation, quaternion_t & rotation);

    void Translate(vector3d_t & trans);
    void Translate(const vector3d_t & trans);

    point3d_t Position() const;
    quaternion_t Orientation() const;

    vector3d_t Translation() const { return m_trans; }
    quaternion_t Rotation() const { return m_quat; }

    void GetPosition(coor_t & x, coor_t & y, coor_t & z) const;
    void GetOrientation(coor_t & q0, coor_t & q1, coor_t & q2, coor_t & q3) const;

    vector3d_t TransformOf(const vector3d_t & src) const;
    vector3d_t InverseTransformOf(const vector3d_t & src) const;
    vector3d_t LocalTransformOf(const vector3d_t & src) const;
    vector3d_t LocalInverseTransformOf(const vector3d_t & src) const;

    point3d_t CoordinatesOf(const point3d_t & src) const;
    point3d_t InverseCoordinatesOf(const point3d_t & src) const;
    point3d_t LocalCoordinatesOf(const point3d_t & src) const;
    point3d_t LocalInverseCoordinatesOf(const point3d_t & src) const;

signals:
    void modified();
    void interpolated();

protected:
    vector3d_t m_trans;
    quaternion_t m_quat;
};

struct ManipulateSensitivety
{
    coef_t rotation = 1;
    coef_t translation = 1;
    coef_t wheel = 1e-3;
    coef_t zoom = 0.1;
};

class ManipulatedFrame : public Frame
{
    Q_OBJECT
    friend class Camera;
    friend class View3D;
public:
    ManipulatedFrame();
    virtual ~ManipulatedFrame();

    void SetPivotPoint(point3d_t point);
    point3d_t PivotPoint() const { return m_pivotPoint; }

signals:
    void manipulated();
public:
    coef_t TranslationSensitivity() const { return m_sens.translation; }
    coef_t RotationSensitivity() const { return m_sens.rotation; }
    coef_t ZoomSensitivity() const { return m_sens.zoom; }
    bool isManipulated() const;

protected:
    virtual void mousePressEvent(const QMouseEvent * event, const Camera * camera);
    virtual void mouseMoveEvent(const QMouseEvent * event, const Camera * camera);
    virtual void mouseReleaseEvent(const QMouseEvent * event,const Camera * camera);
    virtual void mouseDoubleClickEvent(const QMouseEvent * event,const Camera * camera);
    virtual void wheelEvent(const QWheelEvent * event, const Camera * camera);

protected:
    virtual void StartAction(MouseAction ma);
    quaternion_t DeformedBallQuaternion(int x, int y, coor_t cx, coor_t cy, const Camera * camera);
    coef_t DeltaWithPrevPos(const QMouseEvent * event, const Camera * camera) const;
    coef_t WheelDelta(const QWheelEvent * event) const;

private:
    void Zoom(coef_t delta, const Camera * camera);
    void RotateAroundPoint(quaternion_t rot, point3d_t pos);
    void Rotate(quaternion_t rot);

protected:
    ManipulateSensitivety m_sens;

    vector3d_t m_sceneUpVec = {0, 1, 0};
    point3d_t m_pivotPoint = {0, 0, 0};
    bool m_constrainedRotReversed = false;
    bool m_zoomOnPivotPoint = false;
    bool m_rotAroundUpVec = false;

    QPoint m_prevPos;
    QPoint m_pressPos;
    MouseAction m_action;
};

inline coor_t CalculateProjectOnBall(coor_t x, coor_t y)
{
    coor_t size = 1.0;
    coor_t size2 = size * size;
    coor_t sizeLimit = size2 * 0.5;
    coor_t dist2 = x * x + y * y;
    return dist2 < sizeLimit ? std::sqrt(size2 - dist2) : sizeLimit / std::sqrt(dist2);
}
}//namespace view
#endif//VIEW_FRAME_Hs
