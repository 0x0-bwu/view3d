#ifndef VIEW_SHADERVIEW_H
#define VIEW_SHADERVIEW_H
#include "View3D.h"
#include <memory>
namespace view {
namespace renderer {
class Shader;
class Renderer;
}
using namespace renderer;
class ShaderView : public View3D
{
public:
    ShaderView(QWidget * parent = nullptr, Qt::WindowFlags flags = Qt::SubWindow);
    virtual~ShaderView();

protected:
    virtual void init();
    virtual void preDraw();
    virtual void postDraw();
    virtual void draw();

protected:
    std::unique_ptr<Shader> m_shader;
    std::unique_ptr<Renderer> m_renderer;
};
}//namespace view
#endif//VIEW_SHADERVIEW_H
