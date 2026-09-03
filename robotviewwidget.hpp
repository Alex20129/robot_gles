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
	GeometryEngine *mToolGeometry;
	GeometryEngine *mTooltipPathGeometry, *mWristPathGeometry;
	QRobot *mRobot=nullptr;
	QTrajectoryPlanner *mTrajectoryPlanner=nullptr;
	QMatrix4x4 projectionMatrix;
	QMatrix4x4 viewMatrix;
	QVector2D mousePressPosition;
	QVector3D rotationAxis;
	float mCameraShiftZ=-400.0;
	float mCameraShiftX=0.0;
	float mCameraShiftY=0.0;
	bool mCameraRotation = false;
	bool mCameraShifting = false;
	QQuaternion mCameraRotationQ;
	float mCameraPitch=0;
	float mCameraYaw=0;
	float mCameraSensitivity=0.2;
	Ui::RobotViewWidget *ui;
	void updateViewMatrix();

private slots:
	void onRobotConfigurationChanged();
	// void onTrajectoryPlannerTrajectoryChanged();
	void onTrajectoryPlannerPlanningFinished();

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
	void setCameraSensitivity(float camera_sensitivity);
};

#endif // ROBOTVIEWWIDGET_HPP
