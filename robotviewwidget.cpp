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
	delete mWristPathGeometry;
	delete mTooltipPathGeometry;
	delete mTargetGeometry;
	qDeleteAll(mRobotGeometry);
	doneCurrent();
	delete ui;
}

void RobotViewWidget::attachRobot(QRobot *robot)
{
	if (mRobot)
	{
		QObject::disconnect(mRobot, nullptr, this, nullptr);
	}
	mRobot=robot;
	if (nullptr==robot)
	{
		return;
	}
	QObject::connect(mRobot, &QRobot::configurationChanged, this, &RobotViewWidget::onRobotConfigurationChanged);
}

void RobotViewWidget::attachTrajectoryPlanner(QTrajectoryPlanner *trajectory_planner)
{
	if (mTrajectoryPlanner)
	{
		QObject::disconnect(mTrajectoryPlanner, nullptr, this, nullptr);
	}
	mTrajectoryPlanner=trajectory_planner;
	if (nullptr==trajectory_planner)
	{
		return;
	}
	QObject::connect(mTrajectoryPlanner, &QTrajectoryPlanner::trajectoryChanged, this, &RobotViewWidget::onTrajectoryPlannerTrajectoryChanged);
	QObject::connect(mTrajectoryPlanner, &QTrajectoryPlanner::planningFinished, this, &RobotViewWidget::onTrajectoryPlannerPlanningFinished);
}

void RobotViewWidget::setCameraSensitivity(float camera_sensitivity)
{
	mCameraSensitivity=camera_sensitivity;
}

void RobotViewWidget::onRobotConfigurationChanged()
{
	update();
}

void RobotViewWidget::onTrajectoryPlannerTrajectoryChanged()
{
	if(nullptr==mTrajectoryPlanner)
	{
		return;
	}
	const QVector<TrajectorySegment> &trajectorySegments=mTrajectoryPlanner->getSegments();
	mTrajectoryPoints.clear();
	if(trajectorySegments.size()<2)
	{
		return;
	}
	mTrajectoryPoints.append(trajectorySegments.first().positionA);
	for(const TrajectorySegment &segment : trajectorySegments)
	{
		mTrajectoryPoints.append(segment.positionB);
	}
	mWristPathGeometry->setTrajectoryPoints(mTrajectoryPoints);
}

void RobotViewWidget::onTrajectoryPlannerPlanningFinished()
{
	if(nullptr==mTrajectoryPlanner)
	{
		return;
	}
	mTooltipPathGeometry->setTrajectoryPoints(mTrajectoryPlanner->GetTooltipPath());
}

void RobotViewWidget::updateViewMatrix()
{
	viewMatrix.setToIdentity();
	viewMatrix.translate(mCameraShiftX, mCameraShiftY, mCameraShiftZ);
	viewMatrix.rotate(mCameraRotationQ);
}

void RobotViewWidget::mousePressEvent(QMouseEvent *event)
{
	event->accept();
	mousePressPosition = QVector2D(event->localPos());
	if (event->button() == Qt::MiddleButton)
	{
		mCameraShifting = true;
	}
	else
	{
		mCameraRotation = true;
	}
}

void RobotViewWidget::mouseMoveEvent(QMouseEvent *event)
{
	event->accept();
	QVector2D currentPos = QVector2D(event->localPos());
	QVector2D delta = (currentPos - mousePressPosition) * mCameraSensitivity;
	mousePressPosition = currentPos;
	if (mCameraRotation)
	{
		mCameraYaw += delta.x();
		mCameraPitch += delta.y();
		QQuaternion pitchQ = QQuaternion::fromAxisAndAngle(1, 0, 0, mCameraPitch);
		QQuaternion yawQ = QQuaternion::fromAxisAndAngle(0, 1, 0, mCameraYaw);
		mCameraRotationQ = QQuaternion::fromAxisAndAngle(1, 0, 0, -90.0);
		mCameraRotationQ = yawQ * mCameraRotationQ;
		mCameraRotationQ = pitchQ * mCameraRotationQ;
		updateViewMatrix();
		update();
	}
	else if(mCameraShifting)
	{
		mCameraShiftX += delta.x();
		mCameraShiftY -= delta.y();
		updateViewMatrix();
		update();
	}
}

void RobotViewWidget::mouseReleaseEvent(QMouseEvent *event)
{
	event->accept();
	if (event->button() == Qt::MiddleButton)
	{
		mCameraShifting = false;
	}
	else
	{
		mCameraRotation = false;
	}
}

void RobotViewWidget::wheelEvent(QWheelEvent *event)
{
	event->accept();
	qreal zShiftCorrection=5.25;
	if (event->angleDelta().ry()>0)
	{
		mCameraShiftZ+=zShiftCorrection;
	}
	else
	{
		mCameraShiftZ-=zShiftCorrection;
	}
	updateViewMatrix();
	update();
}

void RobotViewWidget::initializeGL()
{
	for (uint32_t modelID=0; modelID<QRobot::numOfJoints; modelID++)
	{
		GeometryEngine *newModelGeometry=new GeometryEngine;
		mRobotGeometry.append(newModelGeometry);
	}
	mTargetGeometry=new GeometryEngine;
	mWristPathGeometry=new GeometryEngine;
	mTooltipPathGeometry=new GeometryEngine;
	static const QStringList robotSTLFiles=
	{
		QString("robot_00.stl"),
		QString("robot_01.stl"),
		QString("robot_02.stl"),
		QString("robot_03.stl"),
		QString("robot_04.stl"),
		QString("robot_05.stl"),
	};
	static const QVector<QVector3D> mColors=
	{
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
		QVector3D(0.75f, 0.95f, 0.95f),
	};
	for (int modelID=0; modelID<robotSTLFiles.size(); modelID++)
	{
		mRobotGeometry.at(modelID)->loadModelFromStlFile(robotSTLFiles.at(modelID));
		mRobotGeometry.at(modelID)->setModelColor(mColors.at(modelID));
	}
	mTargetGeometry->setModelColor({0.97f, 0.21f, 0.21f});
	mTargetGeometry->loadModelFromStlFile("target.stl");
	mWristPathGeometry->setTrajectoryColor({0.2f, 1.0f, 0.2f});
	mTooltipPathGeometry->setTrajectoryColor({0.2f, 0.2f, 1.0f});
	initializeOpenGLFunctions();
	initShaders();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);
	updateViewMatrix();
}

void RobotViewWidget::initShaders()
{
	if (!mRobotShaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/robotvshader.glsl"))
	{
		close();
	}
	if (!mRobotShaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/robotfshader.glsl"))
	{
		close();
	}
	if (!mRobotShaderProgram.link())
	{
		close();
	}
	if (!mLineShaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/linevshader.glsl"))
	{
		close();
	}
	if (!mLineShaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/linefshader.glsl"))
	{
		close();
	}
	if (!mLineShaderProgram.link())
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
	float zNear = 10.0, zFar = 1600.0;
	projectionMatrix.perspective(fov, aspect, zNear, zFar);
}

void RobotViewWidget::paintGL()
{
	if (nullptr==mRobot)
	{
		return;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (!mRobotShaderProgram.bind())
	{
		return;
	}
	for (uint32_t i=0; i<QRobot::numOfJoints; ++i)
	{
		QMatrix4x4 linkMatrix = viewMatrix * mRobot->getLinkMatrix(i);
		mRobotShaderProgram.setUniformValue("mvp_matrix", projectionMatrix * linkMatrix);
		mRobotShaderProgram.setUniformValue("normal_matrix", linkMatrix.normalMatrix());
		mRobotGeometry.at(i)->drawTriangles(&mRobotShaderProgram);
	}
	// QMatrix4x4 targetMatrix = viewMatrix * mRobot->getTargetMatrix();
	// mRobotShaderProgram.setUniformValue("mvp_matrix", projectionMatrix * targetMatrix);
	// mRobotShaderProgram.setUniformValue("normal_matrix", targetMatrix.normalMatrix());
	// mTargetGeometry->drawTriangles(&mRobotShaderProgram);
	if (!mLineShaderProgram.bind())
	{
		return;
	}
	mLineShaderProgram.setUniformValue("mvp_matrix", projectionMatrix * viewMatrix);
	mWristPathGeometry->drawLineStrip(&mLineShaderProgram);
	mTooltipPathGeometry->drawLineStrip(&mLineShaderProgram);
}
