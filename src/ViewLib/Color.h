#ifndef VIEW_COLOR_H
#define VIEW_COLOR_H
#include <cstdint>
#include "math/MathUtility.hpp"
namespace view {

namespace color{

//a|R|G|B
inline static constexpr int32_t black = 0x00000000;
inline static constexpr int32_t white = 0xFFFFFFFF;
inline static constexpr int32_t red   = 0xFFFF0000;
inline static constexpr int32_t green = 0xFF00FF00;
inline static constexpr int32_t blue  = 0xFF0000FF;

inline int RGBToInt(int r, int g, int b)
{
    int32_t c = 0xFF;
    c = (c << 8) | r;
    c = (c << 8) | g;
    c = (c << 8) | b;
    return c;
}

inline int RGBaToInt(int r, int g, int b, int a)
{
    int32_t c = RGBToInt(r, g, b);
    c &= 0xFFFFFF;
    c |= a << 24;
    return c;
}

inline void RGBFromInt(int c, int & r, int & g, int & b)
{
    b = 0xFF & c; c >>= 8;
    g = 0xFF & c; c >>= 8;
    r = 0xFF & c; c >>= 8;
}

inline void RGBaFromInt(int c, int & r, int & g, int & b, int & a)
{
    RGBFromInt(c, r, g, b);
    a = 0xFF & (c >> 24);
}

inline void RandomRGB(int & r, int & g, int & b)
{
    r = 0xFF & math::Random(0, 255);
    g = 0xFF & math::Random(0, 255);
    b = 0xFF & math::Random(0, 255);
}

}//namesapce color
}//namespace view
#endif//VIEW_COLOR_H
