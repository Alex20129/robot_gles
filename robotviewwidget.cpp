#include <QMouseEvent>
#include "robotviewwidget.hpp"
#include "ui_robotviewwidget.h"

RobotViewWidget::RobotViewWidget(QWidget *parent) : QOpenGLWidget(parent)
{
	mCameraRotationQ = QQuaternion::fromAxisAndAngle(1, 0, 0, -90.0);
	ui=new Ui::RobotViewWidget;
	ui->setupUi(this);
}

RobotViewWidget::~RobotViewWidget()
{
	makeCurrent();
	qDeleteAll(mModelGeometry);
	doneCurrent();
	delete ui;
}

void RobotViewWidget::attachRobot(QRobot *robot)
{
	if(nullptr==robot)
	{
		return;
	}
	if(nullptr!=this->robot)
	{
		QObject::disconnect(this->robot, nullptr, this, nullptr);
		QObject::disconnect(this, nullptr, this->robot, nullptr);
	}
	this->robot=robot;
	QObject::connect(robot, &QRobot::configurationChanged, this, &RobotViewWidget::onRobotConfigurationChanged);
	QObject::connect(robot, &QRobot::targetPositionChanged, this, &RobotViewWidget::onRobotTargetPositionChanged);
	onRobotConfigurationChanged();
}

void RobotViewWidget::onRobotConfigurationChanged()
{
	update();
}

void RobotViewWidget::onRobotTargetPositionChanged()
{
	update();
}

void RobotViewWidget::mousePressEvent(QMouseEvent *event)
{
	event->accept();
	mousePressPosition = QVector2D(event->localPos());
	cameraRotation=true;
}

void RobotViewWidget::mouseMoveEvent(QMouseEvent *event)
{
	if(!cameraRotation)
	{
		return;
	}
	event->accept();
	const float sensitivity = 0.25f;
	QVector2D currentPos = QVector2D(event->localPos());
	QVector2D delta = currentPos - mousePressPosition;
	mousePressPosition = currentPos;
	float pitchCorrection = delta.y() * sensitivity;
	float yawCorrection = delta.x() * sensitivity;
	mCameraPitch+=pitchCorrection;
	mCameraYaw+=yawCorrection;
	QQuaternion pitchQ = QQuaternion::fromAxisAndAngle(1, 0, 0, mCameraPitch);
	QQuaternion yawQ = QQuaternion::fromAxisAndAngle(0, 1, 0, mCameraYaw);
	mCameraRotationQ = QQuaternion::fromAxisAndAngle(1, 0, 0, -90.0);
	mCameraRotationQ = yawQ * mCameraRotationQ;
	mCameraRotationQ = pitchQ * mCameraRotationQ;
	update();
}

void RobotViewWidget::mouseReleaseEvent(QMouseEvent *event)
{
	event->accept();
	cameraRotation=false;
}

void RobotViewWidget::wheelEvent(QWheelEvent *event)
{
	event->accept();
	qreal zShiftCorrection=5.25;
	if(event->angleDelta().ry()>0)
	{
		mZoom+=zShiftCorrection;
	}
	else
	{
		mZoom-=zShiftCorrection;
	}
	update();
}

void RobotViewWidget::initializeGL()
{
	static const QStringList mSTLFiles=
	{
		QString("robot_00.stl"),
		QString("robot_01.stl"),
		QString("robot_02.stl"),
		QString("robot_03.stl"),
		QString("robot_04.stl"),
		QString("robot_05.stl"),
		QString("target.stl"),
	};
	static const QVector<QVector3D> mColors=
	{
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.97f, 0.21f, 0.21f),
	};
	for(int modelID=0; modelID<mSTLFiles.size(); modelID++)
	{
		GeometryEngine *newModelGeometry=new GeometryEngine;
		newModelGeometry->loadModelFromStlFile(mSTLFiles.at(modelID));
		newModelGeometry->setModelColor(mColors.at(modelID));
		mModelGeometry.append(newModelGeometry);
	}
	initializeOpenGLFunctions();
	initShaders();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);
}

void RobotViewWidget::initShaders()
{
	if(!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
	{
		close();
	}
	if(!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
	{
		close();
	}
	if(!program.link())
	{
		close();
	}
	if(!program.bind())
	{
		close();
	}
}

void RobotViewWidget::resizeGL(int w, int h)
{
	const qreal fov = 70.0;
	qreal aspect = qreal(w) / qreal(h ? h : 1);
	// Reset projection
	projectionMatrix.setToIdentity();
	// Set perspective projection
	float zNear = 10.0, zFar = 1200.0;
	projectionMatrix.perspective(fov, aspect, zNear, zFar);
}

void RobotViewWidget::paintGL()
{
	if(!robot)
	{
		return;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	QMatrix4x4 view;
	view.translate(0, 0, mZoom);
	view.rotate(mCameraRotationQ);

	size_t i;
	for (i = 0; i < robot->numOfJoints; ++i)
	{
		QMatrix4x4 linkMatrix = view * robot->getLinkMatrix(i);
		QMatrix4x4 mvp = projectionMatrix * linkMatrix;
		program.setUniformValue("mvp_matrix", mvp);
		mModelGeometry.at(i)->drawGeometry(&program, linkMatrix);
	}

	QMatrix4x4 targetMatrix = view * robot->getTargetMatrix();
	QMatrix4x4 mvp = projectionMatrix * targetMatrix;
	program.setUniformValue("mvp_matrix", mvp);
	mModelGeometry.at(i)->drawGeometry(&program, targetMatrix);
}
