#ifndef ROBOTVIEWWIDGET_HPP
#define ROBOTVIEWWIDGET_HPP

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "geometryengine.hpp"
#include "robot.hpp"

namespace Ui
{
	class RobotViewWidget;
}

class RobotViewWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
	QOpenGLShaderProgram program;
	QVector <GeometryEngine *> mModelGeometry;
	QRobot *robot=nullptr;
	QMatrix4x4 projectionMatrix;
	QVector2D mousePressPosition;
	QVector3D rotationAxis;
	qreal mZoom=-400.0;
	bool cameraRotation = false;
	QQuaternion mCameraRotationQ;
	float mCameraPitch=0;
	float mCameraYaw=0;
	Ui::RobotViewWidget *ui;

private slots:
	void onRobotConfigurationChanged();
	void onRobotTargetPositionChanged();

protected:
	void mousePressEvent(QMouseEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void initializeGL() override;
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void initShaders();

public:
	RobotViewWidget(QWidget *parent = nullptr);
	~RobotViewWidget();
	void attachRobot(QRobot *robot);
};

#endif // ROBOTVIEWWIDGET_HPP
