#ifndef ROBOTVIEWWIDGET_HPP
#define ROBOTVIEWWIDGET_HPP

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include "geometryengine.hpp"
#include "robot.hpp"
#include "trajectoryplanner.hpp"

namespace Ui
{
	class RobotViewWidget;
}

class RobotViewWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT
	QOpenGLShaderProgram mRobotShaderProgram;
	QOpenGLShaderProgram mLineShaderProgram;
	QVector <GeometryEngine *> mRobotGeometry;
	QVector<QVector3D> mTrajectoryPoints;
	GeometryEngine *mTargetGeometry, *mPathGeometry;
	QRobot *mRobot=nullptr;
	QTrajectoryPlanner *mTrajectoryPlanner=nullptr;
	QMatrix4x4 projectionMatrix;
	QMatrix4x4 viewMatrix;
	QVector2D mousePressPosition;
	QVector3D rotationAxis;
	qreal mZoom=-400.0;
	bool cameraRotation = false;
	QQuaternion mCameraRotationQ;
	float mCameraPitch=0;
	float mCameraYaw=0;
	Ui::RobotViewWidget *ui;
	void updateViewMatrix();

private slots:
	void onRobotConfigurationChanged();
	void onTrajectoryPlannerTrajectoryChanged();

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
	void attachTrajectoryPlanner(QTrajectoryPlanner *trajectory_planner);
};

#endif // ROBOTVIEWWIDGET_HPP
