#ifndef GEOMETRYENGINE_HPP
#define GEOMETRYENGINE_HPP

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

class GeometryEngine : protected QOpenGLFunctions
{
	QOpenGLBuffer arrayBuf;
	QOpenGLBuffer indexBuf;
	QVector3D minCoord, maxCoord;
	QVector3D sizeVec, centerVec;
	QVector3D modelColor, lightColor;
	int vertexCount=0;
	int indexCount=0;
	void updateBounds(const QVector3D &v);
public:
	void loadModelFromStlFile(const QString &filename);
	GeometryEngine();
	virtual ~GeometryEngine();
	void setModelColor(const QVector3D &model_color);
	void setLightColor(const QVector3D &light_color);
	void drawGeometry(QOpenGLShaderProgram *program, const QMatrix4x4 &model_matrix);
	const QVector3D &center() const;
	const QVector3D &size() const;
};

#endif // GEOMETRYENGINE_HPP
