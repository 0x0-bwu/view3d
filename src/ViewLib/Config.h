#ifndef VIEW_CONFIG_H
#define VIEW_CONFIG_H
#include "generic/common/Macros.hpp"
#include "generic/geometry/Transform.hpp"
#include "generic/geometry/Vector.hpp"
#include "generic/geometry/Point.hpp"
#include "generic/geometry/Box.hpp"
#include "generic/math/Numbers.hpp"
#include "GL/glew.h"
#include "GL/gl.h"
#include <limits>
using namespace generic;
namespace view {
using coef_t = double;
using coor_t = double;
using gl_coef_t = GLdouble;
using gl_coor_t = GLdouble;
using box2d_t = geometry::Box2D<coor_t>;
using box3d_t = geometry::Box3D<coor_t>;
using point2d_t = geometry::Point2D<coor_t>;
using point3d_t = geometry::Point3D<coor_t>;
using vector2d_t = geometry::Point2D<coor_t>;
using vector3d_t = geometry::Vector3D<coor_t>;
using quaternion_t = geometry::Quaternion<coor_t>;
using transform2d_t = geometry::Transform2D<coor_t>;
using transform3d_t = geometry::Transform3D<coor_t>;

enum class ViewDirection { UnKnown = 0, Front = 1, Back, Top, Bottom, Right, Left };

namespace constant {
inline static constexpr int frequency = 25;//Hz
inline static constexpr int period = 1000 / frequency;//ms
inline static constexpr coor_t epsilon = std::numeric_limits<coor_t>::epsilon();
inline static constexpr size_t circle_div = 64;
}//namespace

}//namespace view
#endif//VIEW_CONFIG_H
