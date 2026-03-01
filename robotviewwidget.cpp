#include <QMouseEvent>
#include "robotviewwidget.hpp"
#include "ui_robotviewwidget.h"

RobotViewWidget::RobotViewWidget(QWidget *parent) : QOpenGLWidget(parent)
{
	robot=nullptr;
	mCameraRotationQ = QQuaternion::fromAxisAndAngle(1, 0, 0, -90.0);
	ui=new Ui::RobotViewWidget;
	ui->setupUi(this);
	//animationTimer.start(10, this);
}

RobotViewWidget::~RobotViewWidget()
{
	makeCurrent();
	qDeleteAll(mLinkGeometry);
	doneCurrent();
	delete ui;
}

void RobotViewWidget::onRobotConfigurationChanged()
{
	//QVector3D toolPositionVec=robot->getToolPosition();
	//qDebug()<<toolPositionVec;
	//qDebug()<<toolPositionVec.length();
	update();
}

void RobotViewWidget::attachRobot(QRobot *robot)
{
	this->robot=robot;
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

void RobotViewWidget::timerEvent(QTimerEvent *event)
{
	event->accept();
	if(!robot)
	{
		return;
	}
	//animationTimer.stop();
}

void RobotViewWidget::initializeGL()
{
	static const QStringList mLinksSTLFiles=
	{
		QString("robot_00.stl"),
		QString("robot_01.stl"),
		QString("robot_02.stl"),
		QString("robot_03.stl"),
		QString("robot_04.stl"),
		QString("robot_05.stl"),
	};
	for(int linkID=0; linkID<QRobot::numOfJoints; linkID++)
	{
		GeometryEngine *newLinkGeometry=new GeometryEngine;
		newLinkGeometry->loadGeometryFromStlFile(mLinksSTLFiles.at(linkID));
		mLinkGeometry.append(newLinkGeometry);
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

	for (size_t i = 0; i < robot->numOfJoints; ++i)
	{
		QMatrix4x4 linkMatrix = view * robot->getLinkMatrix(i);
		QMatrix4x4 mvp = projectionMatrix * linkMatrix;
		program.setUniformValue("mvp_matrix", mvp);
		mLinkGeometry.at(i)->drawGeometry(&program, linkMatrix);
	}
}
