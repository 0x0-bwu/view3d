#include "Painter.h"
#include "View3D.h"
#include <QPainter>
#include "GL/glu.h"
using namespace view;
Painter::Painter(View3D * view) : m_view(view)
{
}

Painter::~Painter()
{

}

void Painter::DrawTest()
{
    const float nbSteps = 200.0;

    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i < nbSteps; ++i) {
      const float ratio = i / nbSteps;
      const float angle = 21.0 * ratio;
      const float c = cos(angle);
      const float s = sin(angle);
      const float r1 = 1.0 - 0.8f * ratio;
      const float r2 = 0.8f - 0.8f * ratio;
      const float alt = ratio - 0.5f;
      const float nor = 0.5f;
      const float up = sqrt(1.0 - nor * nor);
      glColor3f(1.0 - ratio, 0.2f, ratio);
      glNormal3f(nor * c, up, nor * s);
      glVertex3f(r1 * c, alt, r1 * s);
      glVertex3f(r2 * c, alt + 0.05f, r2 * s);
    }
    glEnd();
}

void Painter::DrawText(int x, int y, const QString & text, const QFont & font, const QColor & color)
{
    // Render text
    QPainter painter(m_view);
    painter.setPen(color);
    painter.setFont(font);
    painter.drawText(x, y, text);
    painter.end();
}

void Painter::DrawAxis(coor_t length, gl_coef_t lineWidth)
{
    const coor_t charWidth = length / 40.0;
    const coor_t charHeight = length / 30.0;
    const coor_t charShift = 1.04 * length;

    GLboolean lighting, colorMaterial;
    glGetBooleanv(GL_LIGHTING, &lighting);
    glGetBooleanv(GL_COLOR_MATERIAL, &colorMaterial);

    glDisable(GL_LIGHTING);

    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

    glBegin(GL_LINES);
    // The X
    glVertex3d(charShift,  charWidth, -charHeight);
    glVertex3d(charShift, -charWidth,  charHeight);
    glVertex3d(charShift, -charWidth, -charHeight);
    glVertex3d(charShift,  charWidth,  charHeight);
    // The Y
    glVertex3d( charWidth, charShift,  charHeight);
    glVertex3d(       0.0, charShift,         0.0);
    glVertex3d(-charWidth, charShift,  charHeight);
    glVertex3d(       0.0, charShift,         0.0);
    glVertex3d(       0.0, charShift,         0.0);
    glVertex3d(       0.0, charShift, -charHeight);
    // The Z
    glVertex3d(-charWidth,  charHeight, charShift);
    glVertex3d( charWidth,  charHeight, charShift);
    glVertex3d( charWidth,  charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d(-charWidth, -charHeight, charShift);
    glVertex3d( charWidth, -charHeight, charShift);
    glEnd();

    glEnable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    float color[4];
    color[0] = 0.7f;
    color[1] = 0.7f;
    color[2] = 1.0f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    DrawArrow(length, 0.01 * length);

    color[0] = 1.0f;
    color[1] = 0.7f;
    color[2] = 0.7f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    DrawArrow(length, 0.01 * length);
    glPopMatrix();

    color[0] = 0.7f;
    color[1] = 1.0f;
    color[2] = 0.7f;
    color[3] = 1.0f;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color);
    glPushMatrix();
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    DrawArrow(length, 0.01 * length);
    glPopMatrix();

    glLineWidth(prevWidth);

    if (colorMaterial)
      glEnable(GL_COLOR_MATERIAL);
    if (!lighting)
      glDisable(GL_LIGHTING);
}

void Painter::DrawArrow(coor_t length, coor_t radius, int nDiv)
{
    static GLUquadric * quadric = gluNewQuadric();

    if (radius < 0.0)
      radius = 0.05 * length;

    const coor_t head = 2.5 * (radius / length) + 0.1;
    const coor_t coneRadiusCoef = 4.0 - 5.0 * head;

    gluCylinder(quadric, radius, radius, length * (1.0 - head / coneRadiusCoef), nDiv, 1);
    glTranslated(0.0, 0.0, length * (1.0 - head));
    gluCylinder(quadric, coneRadiusCoef * radius, 0.0, head * length, nDiv, 1);
    glTranslated(0.0, 0.0, -length * (1.0 - head));
}

void Painter::DrawHistogram()
{

}

void Painter::DrawMode(const Model * model)
{
	if(nullptr == model) return;
	if(dynamic_cast<const FrameModel2D *>(model)) DrawFrameModel2D(dynamic_cast<const FrameModel2D *>(model));
    else if(dynamic_cast<const FrameModel3D *>(model)) DrawFrameModel3D(dynamic_cast<const FrameModel3D *>(model));
    else if(dynamic_cast<const SurfaceModel2D *>(model)) DrawSurfaceModel2D(dynamic_cast<const SurfaceModel2D *>(model));
    else if(dynamic_cast<const SurfaceModel3D *>(model)) DrawSurfaceModel3D(dynamic_cast<const SurfaceModel3D *>(model));
    else if(dynamic_cast<const TetrahedronModel *>(model)) DrawTetrahedronModel(dynamic_cast<const TetrahedronModel *>(model));
}

void Painter::DrawFrameModel2D(const FrameModel2D * frameModel, gl_coef_t lineWidth)
{
	if(nullptr == frameModel) return;
    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

    for(const auto & model : frameModel->models){
		if(!model.indices.size()) continue;

        int r, b, g;
        color::RGBFromInt(model.color, r, g, b);
        glColor3ub(r, g, b);

        glBegin(GL_LINE_STRIP);
        for(size_t i : model.indices){
            const auto & vertex = frameModel->vertices[i];
            glVertex2d(vertex[0], vertex[1]);
        }
        const auto & vertex = frameModel->vertices[model.indices.front()];
        glVertex2d(vertex[0], vertex[1]); 
        glEnd();
    }
    glLineWidth(prevWidth);
}

void Painter::DrawFrameModel3D(const FrameModel3D * frameModel, gl_coef_t lineWidth)
{
   	if(nullptr == frameModel) return;
    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

    for(const auto & model : frameModel->models){
		if(!model.indices.size()) continue;

        int r, b, g;
        color::RGBFromInt(model.color, r, g, b);
        glColor3ub(r, g, b);

        glBegin(GL_LINE_STRIP);
        for(size_t i : model.indices){
            const auto & vertex = frameModel->vertices[i];
            glVertex3d(vertex[0], vertex[1], vertex[2]);
        }
        const auto & vertex = frameModel->vertices[model.indices.front()];
        glVertex3d(vertex[0], vertex[1], vertex[2]); 
        glEnd();
    }
    glLineWidth(prevWidth); 
}

void Painter::DrawSurfaceModel2D(const SurfaceModel2D * surfModel, gl_coef_t lineWidth)
{
	if(nullptr == surfModel) return;
	
    int r, g, b;
	const auto & colors = surfModel->colors;
    const auto & vertices = surfModel->vertices;

//    gl_coef_t prevPtSize;
//    glGetDoublev(GL_POINT_SIZE, &prevPtSize);
//    glPointSize(5.0 * lineWidth);

//    glBegin(GL_POINTS);
//    for(size_t i = 0; i < vertices.size(); ++i){
//        color::RGBFromInt(colors[i], r, g, b);
//        glColor3ub(r, g, b);
//        glVertex2d(vertices[i][0], vertices[i][1]);
//    }
//    glEnd();
//    glPointSize(prevPtSize);


    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

	typename SurfaceModel2D::LineStrips lines;
	surfModel->GetLineStrips(lines);
	for(const auto & line : lines){
		glBegin(GL_LINE_STRIP);
		const auto & color1 = colors[line.v1()];
        const auto & vertex1 = vertices[line.v1()];
        color::RGBFromInt(color1, r, g, b);
		glColor3ub(r, g, b);
        glVertex2d(vertex1[0], vertex1[1]);

		const auto & color2 = colors[line.v2()];
        const auto & vertex2 = vertices[line.v2()];
        color::RGBFromInt(color2, r, g, b);
        glVertex2d(vertex2[0], vertex2[1]);
		glEnd();
	}
	glLineWidth(prevWidth);
}

void Painter::DrawSurfaceModel3D(const SurfaceModel3D * surfModel, gl_coef_t lineWidth)
{
    if(nullptr == surfModel) return;

    int r, g, b;
    const auto & colors = surfModel->colors;
    const auto & vertices = surfModel->vertices;

//    gl_coef_t prevPtSize;
//    glGetDoublev(GL_POINT_SIZE, &prevPtSize);
//    glPointSize(5.0 * lineWidth);

//    glBegin(GL_POINTS);
//    for(size_t i = 0; i < vertices.size(); ++i){
//        color::RGBFromInt(colors[i], r, g, b);
//        glColor3ub(r, g, b);
//        glVertex2d(vertices[i][0], vertices[i][1]);
//    }
//    glEnd();
//    glPointSize(prevPtSize);


    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

    typename SurfaceModel3D::LineStrips lines;
    surfModel->GetLineStrips(lines);
    for(const auto & line : lines){
        glBegin(GL_LINE_STRIP);
        const auto & color1 = colors[line.v1()];
        const auto & vertex1 = vertices[line.v1()];
        color::RGBFromInt(color1, r, g, b);
        glColor3ub(r, g, b);
        glVertex3d(vertex1[0], vertex1[1], vertex1[2]);

        const auto & color2 = colors[line.v2()];
        const auto & vertex2 = vertices[line.v2()];
        color::RGBFromInt(color2, r, g, b);
        glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
        glEnd();
    }
    glLineWidth(prevWidth);
}

void Painter::DrawTetrahedronModel(const TetrahedronModel * tetModel, gl_coef_t lineWidth)
{
    if(nullptr == tetModel) return;

    int r, g, b;
    const auto & colors = tetModel->colors;
    const auto & vertices = tetModel->vertices;

    gl_coef_t prevWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevWidth);
    glLineWidth(lineWidth);

    auto isShow = [&vertices](const typename TetrahedronModel::Element & ele)
    {
        auto ct = vertices[ele[0]] + vertices[ele[1]] + vertices[ele[2]] + vertices[ele[3]];
//        if(ct[0] < 0.25 && ct[1] < 0.25 && ct[2] > 0) return true;
//        return false;
        return true;
    };

    //outline
    auto lineColor = color::white;
    color::RGBFromInt(lineColor, r, g, b);
    glColor3ub(r, g, b);
    for(const auto & element : tetModel->elements){
        if(!isShow(element)) continue;
        for(size_t i = 0; i < 4; ++i){
            for(size_t j = i + 1; j < 4; ++j){
                const auto & vertex1 = vertices[element[i]];
                const auto & vertex2 = vertices[element[j]];
                glBegin(GL_LINE_STRIP);
                glVertex3d(vertex1[0], vertex1[1], vertex1[2]);
                glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
                glEnd();
            }
        }
    }

    //face
    auto faceColor = color::blue;
    color::RGBFromInt(faceColor, r, g, b);
    glColor3ub(r, g, b);
    for(const auto & element : tetModel->elements){
        if(!isShow(element)) continue;
        const auto & vertex1 = vertices[element[0]];
        const auto & vertex2 = vertices[element[1]];
        const auto & vertex3 = vertices[element[2]];
        const auto & vertex4 = vertices[element[3]];
        glBegin(GL_TRIANGLE_STRIP);
        glVertex3d(vertex1[0], vertex1[1], vertex1[2]);
        glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
        glVertex3d(vertex3[0], vertex3[1], vertex3[2]);
        glVertex3d(vertex4[0], vertex4[1], vertex4[2]);
        glVertex3d(vertex1[0], vertex1[1], vertex1[2]);
        glVertex3d(vertex2[0], vertex2[1], vertex2[2]);
        glEnd();
    }
    glLineWidth(prevWidth);
}
