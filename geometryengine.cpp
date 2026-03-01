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
	indexBuf=QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

	minCoord = QVector3D( 1e9, 1e9, 1e9);
	maxCoord = QVector3D(-1e9, -1e9, -1e9);

	initializeOpenGLFunctions();
	arrayBuf.create();
	indexBuf.create();
}

GeometryEngine::~GeometryEngine()
{
	arrayBuf.destroy();
	indexBuf.destroy();
}

void GeometryEngine::updateBounds(const QVector3D &v)
{
	minCoord.setX(qMin(minCoord.x(), v.x()));
	minCoord.setY(qMin(minCoord.y(), v.y()));
	minCoord.setZ(qMin(minCoord.z(), v.z()));

	maxCoord.setX(qMax(maxCoord.x(), v.x()));
	maxCoord.setY(qMax(maxCoord.y(), v.y()));
	maxCoord.setZ(qMax(maxCoord.z(), v.z()));

	centerVec = (minCoord + maxCoord) / 2.0;
	sizeVec = maxCoord - minCoord;
}

void GeometryEngine::loadGeometryFromStlFile(const QString &filename)
{
	arrayBuf.destroy();
	indexBuf.destroy();

	arrayBuf.create();
	indexBuf.create();

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly))
	{
		qWarning() << "Cannot open STL file:" << filename;
		return;
	}

	QDataStream stream(&file);
	stream.setByteOrder(QDataStream::LittleEndian);
	stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

	// Пропускаем 80-байтный заголовок
	QByteArray header(80, 0);
	stream.readRawData(header.data(), 80);

	quint32 numTriangles = 0;
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

	for (quint32 i = 0; i < numTriangles; ++i)
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

		int base = vertices.size();

		vertices << VertexData{v1, normal};
		vertices << VertexData{v2, normal};
		vertices << VertexData{v3, normal};

		indices << base << base+1 << base+2;
	}

	file.close();

	vertexCount = vertices.size();
	indexCount = indices.size();

	arrayBuf.bind();
	arrayBuf.allocate(vertices.constData(), vertexCount * sizeof(VertexData));

	indexBuf.bind();
	indexBuf.allocate(indices.constData(), indexCount * sizeof(quint32));
}

void GeometryEngine::drawGeometry(QOpenGLShaderProgram *program, const QMatrix4x4 &model_matrix)
{
	arrayBuf.bind();
	indexBuf.bind();

	program->setUniformValue("model_matrix", model_matrix);
	program->setUniformValue("normal_matrix", model_matrix.normalMatrix());

	program->setUniformValue("u_materialColor",  QVector3D(0.75f, 0.95f, 0.95f));
	program->setUniformValue("u_shininess",    64.0f);
	program->setUniformValue("u_lightDirection", QVector3D(0.0f, 1.0f, 1.0f).normalized());
	program->setUniformValue("u_lightColor",   QVector3D(1.0f, 1.0f, 1.0f));
	program->setUniformValue("u_ambientStrength", 0.12f);

	int vertexLocation = program->attributeLocation("a_position");
	program->enableAttributeArray(vertexLocation);
	program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, sizeof(VertexData));

	int normalLocation = program->attributeLocation("a_normal");
	program->enableAttributeArray(normalLocation);
	program->setAttributeBuffer(normalLocation, GL_FLOAT, 3 * sizeof(float), 3, sizeof(VertexData));

	glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
}

QVector3D GeometryEngine::center() const
{
	return centerVec;
}

QVector3D GeometryEngine::size() const
{
	return sizeVec;
}
