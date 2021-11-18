#ifndef VIEW_CAMERA_H
#define VIEW_CAMERA_H
#include "Config.h"
#include "Frame.h"
#include <QObject>
#include <utility>
#include <memory>
#include <map>
namespace view {

enum class CameraType { Perspective = 0, Orthographic = 1 };
struct CameraParameters
{
    CameraType type = CameraType::Orthographic;
    coor_t fieldOfView = math::pi_quarter;
    coor_t sceneRadius = 1.0;
    coor_t zNearCoef = 0.005;
    coor_t zClippingCoef = 1.732;
    coor_t orthoCoef = tan(math::pi / 8);
    point3d_t screenCenter = {0.0, 0.0, 0.0};
    gl_coor_t modelViewMatrix[16];
    gl_coor_t projectionMatrix[16];
    bool modelViewMatrixIsUpToDate = false;
    bool projectionMatrixIsUpToDate = false;
};

class Frame;
class Camera : public QObject
{
    Q_OBJECT
    friend class View3D;
public:
    Camera();
    virtual ~Camera();

    point3d_t Position() const;
    vector3d_t UpVector() const;
    vector3d_t ViewDirection() const;
    vector3d_t RightVector() const;
    quaternion_t Orientation() const;

    virtual coor_t zNear() const;
    virtual coor_t zFar() const;
    virtual void GetOrthoWidthHeight(gl_coor_t & halfWidth, gl_coor_t & halfHeight) const;

    point3d_t CameraCoordinatesOf(const point3d_t & src) const;
    point3d_t WorldCoordinatesOf(const point3d_t & src) const;

    coor_t DistanceToSceneCenter() const;
    coef_t HorizontalFieldOfView() const;

    coef_t AspectRatio() const;
    int ScreenWidth() const { return m_screen[0]; }
    int ScreenHeight() const { return m_screen[1]; }
    void GetViewport(GLint viewport[4]) const;

    CameraType Type() const { return m_params.type; }
    coef_t FieldOfView() const { return m_params.fieldOfView; }
    coef_t SceneRadius() const { return m_params.sceneRadius; }
    coef_t zNearCoefficient() const { return m_params.zNearCoef; }
    coef_t zClippingCoefficient() const { return m_params.zClippingCoef; }
    point3d_t SceneCenter() const { return m_params.screenCenter; }
    ManipulatedFrame * GetFrame() const { return m_frame.get(); }
    
    void SetScreenWidthAndHeight(int width, int height);
    void SetSceneRadius(coor_t radius);
    void SetSceneCenter(point3d_t center);
    void SetPivotPoint(point3d_t pivot);
    void SetFocusDistance(coor_t distance);
    void SetFrame(ManipulatedFrame * frame);

    virtual void LoadProjectionMatrix(bool reset = true) const;
    virtual void LoadModelViewMatrix(bool reset = true) const;
    void ComputeProjectionMatrix() const;
    void ComputeModelViewMatrix() const;    
    void FitSphere(const point3d_t & center, coor_t radius);

    point3d_t PivotPoint() const { return m_frame->PivotPoint(); }

    point3d_t PointUnderPixel(const QPoint & pixel, bool & found) const;
    point3d_t ProjectedCoordinatesOf(const point3d_t & src) const;
    point3d_t UnprojectedCoordinatesOf(const point3d_t & src) const;

public slots:
    void OnFrameModified();
    void ShowEntireScene();

protected:
    int m_screen[2] = {600, 400};
    coor_t m_focusDistance;
    mutable CameraParameters m_params;
    std::unique_ptr<ManipulatedFrame> m_frame;
};
}//namespace view
#endif//VIEW_CAMERA_H
