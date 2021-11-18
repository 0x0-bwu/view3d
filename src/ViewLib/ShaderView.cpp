#include "ShaderView.h"
#include "renderer/VertexBuffer.h"
#include "renderer/Renderer.h"
#include "renderer/Texture.h"
#include "renderer/Shader.h"
#include <QApplication>
#include <QFileInfo>
#include <iostream>
using namespace view;
using namespace view::renderer;
ShaderView::ShaderView(QWidget * parent, Qt::WindowFlags flags)
 : View3D(parent, flags), m_shader(nullptr), m_renderer(new Renderer)
{
}

ShaderView::~ShaderView()
{
}

void ShaderView::init()
{
    if (glewInit() != GLEW_OK)
		std::cout << "Error! GLEW is not initialized properly!" << std::endl;
	else {
		std::cout << "GLEW   version: " << glewGetString(GLEW_VERSION) << std::endl;
		std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
	}

    QString shaderFile = QApplication::applicationDirPath() + "/../../../src/view3d/renderer/res/shaders/Basic.shader";
    shaderFile = QFileInfo(shaderFile).canonicalFilePath();

    m_shader.reset(new Shader(shaderFile.toStdString()));
    m_shader->Bind();

    QString textureFile = QApplication::applicationDirPath() + "../../../src/view3d/renderer/res/textures/Logo.png";
    Texture texture(textureFile.toStdString());
    texture.Bind(0);
    m_shader->SetUniform1i("u_Texture", 0);
}

void ShaderView::preDraw()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void ShaderView::postDraw()
{

}

void ShaderView::draw()
{
    float_t vertices[] = {
        0.0, 0.0, 0.0, 0.0, 0.0,
        160.0, 0.0, 0.0, 1.0, 0.0,
        160.0, 160.0, 0.0, 1.0, 1.0,
        0.0, 160.0, 0.0, 0.0, 1.0
    };

    unsigned int indices[] = { 0, 1, 2, 2, 3, 0 };

    GLclampf color[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

    VertexArray vertexArray;
    VertexBuffer vertexBuffer(4 * 5 * sizeof(double), vertices);

    VertexBufferLayout layout;
    layout.Push<double>(3);
    layout.Push<double>(2);
    vertexArray.AddBuffer(vertexBuffer, layout);

    IndexBuffer indexBuffer(2 * 3 * sizeof(unsigned int), indices);

    m_renderer->Clean();
    m_renderer->SetClearColor(color);

    
    // shaderProgram.SetUniformMat("u_MVP", mvp);
    m_renderer->Draw(vertexArray, indexBuffer, *m_shader);
}
