#ifndef GEOMETRYENGINE_HPP
#define GEOMETRYENGINE_HPP

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

class GeometryEngine : protected QOpenGLFunctions
{
	QOpenGLBuffer arrayBuf;
	QOpenGLBuffer geometryIBuf;
	QOpenGLBuffer mTrajectoryVBuf;
	QVector3D minCoord, maxCoord;
	QVector3D sizeVec, centerVec;
	QVector3D mModelColor, mLightColor;
	QVector3D mTrajectoryColor;
	int vertexCount=0;
	int indexCount=0;
	int mTrajectoryVertexCount=0;
	void updateBounds(const QVector3D &v);
public:
	void loadModelFromStlFile(const QString &filename);
	GeometryEngine();
	virtual ~GeometryEngine();
	void setModelColor(const QVector3D &model_color);
	void setLightColor(const QVector3D &light_color);
	void setTrajectoryColor(const QVector3D &trajectory_color);
	void setTrajectoryPoints(const QVector<QVector3D> &points);
	void clearTrajectory();
	void drawTriangles(QOpenGLShaderProgram *program);
	void drawLineStrip(QOpenGLShaderProgram *program);
	const QVector3D &center() const;
	const QVector3D &size() const;
};

#endif // GEOMETRYENGINE_HPP
