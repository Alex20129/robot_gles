#include "geometryengine.hpp"

#include <QVector2D>
#include <QVector3D>
#include <QFile>

struct VertexData
{
	QVector3D pos;
	QVector3D normal;
};

GeometryEngine::GeometryEngine()
{
	geometryIBuf=QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
	mTrajectoryVBuf=QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);

	minCoord=QVector3D( 1e9, 1e9, 1e9);
	maxCoord=QVector3D(-1e9, -1e9, -1e9);

	mModelColor=QVector3D(1.0f, 1.0f, 1.0f);
	mLightColor=QVector3D(1.0f, 1.0f, 1.0f);
	mTrajectoryColor=QVector3D(0.2f, 1.0f, 0.2f);

	initializeOpenGLFunctions();
	arrayBuf.create();
	geometryIBuf.create();
	mTrajectoryVBuf.create();
}

GeometryEngine::~GeometryEngine()
{
	arrayBuf.destroy();
	geometryIBuf.destroy();
	mTrajectoryVBuf.destroy();
}

void GeometryEngine::updateBounds(const QVector3D &v)
{
	minCoord.setX(qMin(minCoord.x(), v.x()));
	minCoord.setY(qMin(minCoord.y(), v.y()));
	minCoord.setZ(qMin(minCoord.z(), v.z()));

	maxCoord.setX(qMax(maxCoord.x(), v.x()));
	maxCoord.setY(qMax(maxCoord.y(), v.y()));
	maxCoord.setZ(qMax(maxCoord.z(), v.z()));

	centerVec=(minCoord + maxCoord) / 2.0;
	sizeVec=maxCoord - minCoord;
}

void GeometryEngine::loadModelFromStlFile(const QString &filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		qWarning() << "Cannot open STL file:" << filename;
		return;
	}

	QDataStream stream(&file);
	stream.setByteOrder(QDataStream::LittleEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

	QByteArray header(80, 0);
	stream.readRawData(header.data(), 80);

	quint32 numTriangles=0;
	stream >> numTriangles;

	if (numTriangles < 4 || numTriangles > 8000000)
	{
		qWarning() << "Invalid triangle count:" << numTriangles;
		return;
	}

	QVector<VertexData> vertices;
	QVector<quint32>   indices;

	vertices.reserve(numTriangles * 3);
	indices.reserve (numTriangles * 3);

	for (quint32 i=0; i < numTriangles; ++i)
	{
		QVector3D normal, v1, v2, v3;

		stream >> normal;

		stream >> v1;
		stream >> v2;
		stream >> v3;

		updateBounds(v1);
		updateBounds(v2);
		updateBounds(v3);

		quint16 attribute;
		stream >> attribute;

		int base=vertices.size();

		vertices << VertexData{v1, normal};
		vertices << VertexData{v2, normal};
		vertices << VertexData{v3, normal};

		indices << base << base+1 << base+2;
	}

	file.close();

	vertexCount=vertices.size();
	indexCount=indices.size();

	arrayBuf.bind();
	arrayBuf.allocate(vertices.constData(), vertexCount * sizeof(VertexData));

	geometryIBuf.bind();
	geometryIBuf.allocate(indices.constData(), indexCount * sizeof(quint32));
}

void GeometryEngine::setModelColor(const QVector3D &model_color)
{
	mModelColor=model_color;
}

void GeometryEngine::setLightColor(const QVector3D &light_color)
{
	mLightColor=light_color;
}

void GeometryEngine::setTrajectoryColor(const QVector3D &trajectory_color)
{
	mTrajectoryColor=trajectory_color;
}

void GeometryEngine::setTrajectoryPoints(const QVector<QVector3D> &points)
{
	mTrajectoryVertexCount=points.size();
	if (mTrajectoryVertexCount<1)
	{
		return;
	}
	mTrajectoryVBuf.bind();
	mTrajectoryVBuf.allocate(points.constData(), mTrajectoryVertexCount * sizeof(QVector3D));
}

void GeometryEngine::clearTrajectory()
{
	mTrajectoryVertexCount=0;
}

void GeometryEngine::drawTriangles(QOpenGLShaderProgram *program)
{
	arrayBuf.bind();
	geometryIBuf.bind();
	program->setUniformValue("u_materialColor",	mModelColor);
	program->setUniformValue("u_shininess", 64.0f);
	program->setUniformValue("u_lightDirection", QVector3D(0.0f, 1.0f, 1.0f).normalized());
	program->setUniformValue("u_lightColor", mLightColor);
	program->setUniformValue("u_ambientStrength", 0.12f);
	int vertexLocation=program->attributeLocation("a_position");
	program->enableAttributeArray(vertexLocation);
	program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));
	int normalLocation=program->attributeLocation("a_normal");
	program->enableAttributeArray(normalLocation);
	program->setAttributeBuffer(normalLocation, GL_FLOAT, 3 * sizeof(float), 3, sizeof(VertexData));
	glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
	program->disableAttributeArray(vertexLocation);
	program->disableAttributeArray(normalLocation);
}

void GeometryEngine::drawLineStrip(QOpenGLShaderProgram *program)
{
	mTrajectoryVBuf.bind();
	program->setUniformValue("u_color", mTrajectoryColor);
	int positionLocation=program->attributeLocation("a_position");
	program->enableAttributeArray(positionLocation);
	program->setAttributeBuffer(positionLocation, GL_FLOAT, 0, 3, sizeof(QVector3D));
	glLineWidth(3.0f);
	glDrawArrays(GL_LINE_STRIP, 0, mTrajectoryVertexCount);
	program->disableAttributeArray(positionLocation);
}

const QVector3D &GeometryEngine::center() const
{
	return centerVec;
}

const QVector3D &GeometryEngine::size() const
{
	return sizeVec;
}
