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
	int vertexCount = 0;
	int indexCount = 0;
	void updateBounds(const QVector3D &v);
public:
	void loadGeometryFromStlFile(const QString &filename);
	GeometryEngine();
	virtual ~GeometryEngine();
	void drawGeometry(QOpenGLShaderProgram *program, const QMatrix4x4 &model_matrix);
	QVector3D center() const;
	QVector3D size() const;
};

#endif // GEOMETRYENGINE_HPP
