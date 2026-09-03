#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimerEvent>
#include "trajectoryplanner.hpp"

void QTrajectoryPlanner::timerEvent(QTimerEvent *event)
{
	event->accept();
	if (mAnimationProgress<mPoses.size())
	{
		emit needToSetPose(mPoses[mAnimationProgress]);
		mAnimationProgress++;
	}
	else
	{
		mAnimationTimer.stop();
		mAnimationProgress=0;
		emit animationFinished();
	}
}

QTrajectoryPlanner::QTrajectoryPlanner(QObject *parent) : QObject(parent)
{
}

void QTrajectoryPlanner::attachRobot(QRobot *robot)
{
	if (nullptr!=mRobot)
	{
		QObject::disconnect(mRobot, nullptr, this, nullptr);
		QObject::disconnect(this, nullptr, mRobot, nullptr);
	}
	mRobot=robot;
	if (nullptr==robot)
	{
		return;
	}
	QObject::connect(this, &QTrajectoryPlanner::needToSetPose, mRobot, &QRobot::setPose);
}

void QTrajectoryPlanner::rebuildPoses()
{
	mPoses.clear();
	mWristPath.clear();
	mTooltipPath.clear();
	if(nullptr==mRobot)
	{
		return;
	}
	for(const TrajectorySegment &segment : mSegments)
	{
		switch (segment.type)
		{
			default:
			case TrajectorySegment::SegmentType::Line:
			{
				QQuaternion quOrientationA=QQuaternion::fromEulerAngles(segment.orientationA);
				QQuaternion quOrientationB=QQuaternion::fromEulerAngles(segment.orientationB);
				double distance=(segment.positionA-segment.positionB).length();
				uint steps=distance/mStepSize;
				if(steps<2)
				{
					steps=2;
				}
				for(uint step=0; step<steps; step++)
				{
					double t=(double)(step)/(double)(steps);
					// TODO: handle unsolved position cases
					double PositionError=mRobot->solveIkForPosition(segment.positionA*(1.0-t) + segment.positionB*t);
					// TODO: handle unsolved orientation cases
					double OrientationError=mRobot->solveIkForOrientation(QQuaternion::slerp(quOrientationA, quOrientationB, t));
					mPoses.push_back(mRobot->getPose());
					mWristPath.push_back(mRobot->getWristPosition());
					mTooltipPath.push_back(mRobot->getTooltipPosition());
				}
				break;
			}
			case TrajectorySegment::SegmentType::Arc:
			{
				// TODO: Arc
				QVector3D a=segment.positionA;
				QVector3D b=segment.positionB;
				QVector3D c=segment.positionC;

				QVector3D ca=a-c;
				QVector3D cb=b-c;

				QVector3D normal=QVector3D::crossProduct(ca, cb);
				float normalLengthSquared=normal.lengthSquared();
				float caLengthSquared=ca.lengthSquared();
				float cbLengthSquared=cb.lengthSquared();

				QVector3D cToCenter=QVector3D::crossProduct(
					caLengthSquared * cb - cbLengthSquared * ca, normal) / (2.0f * normalLengthSquared);
				QVector3D center=c + cToCenter;
				float radius=(c - center).length();

				break;
			}
			case TrajectorySegment::SegmentType::Spline:
			{
				// TODO: Spline
				break;
			}
			case TrajectorySegment::SegmentType::Free:
			{
				// TODO: Free
				break;
			}
		}
	}
	if (!mSegments.isEmpty())
	{
		const TrajectorySegment &lastSegment=mSegments.last();
		mRobot->solveIkForPosition(lastSegment.positionB);
		mRobot->solveIkForOrientation(QQuaternion::fromEulerAngles(lastSegment.orientationB));
		mPoses.push_back(mRobot->getPose());
		mWristPath.push_back(mRobot->getWristPosition());
		mTooltipPath.push_back(mRobot->getTooltipPosition());
	}
	emit planningFinished();
}

void QTrajectoryPlanner::clear()
{
	mPoses.clear();
	mWristPath.clear();
	mTooltipPath.clear();
	if(mSegments.size())
	{
		mSegments.clear();
		emit trajectoryChanged();
	}
}

bool QTrajectoryPlanner::loadFromJsonFile(const QString &file)
{
	clear();
	QFile jsonFile(file);
	if (!jsonFile.open(QIODevice::ReadOnly))
	{
		return (false);
	}
	QJsonDocument pathJsonDocument=QJsonDocument::fromJson(jsonFile.readAll());
	QJsonObject pathJsonObject=pathJsonDocument.object();
	QJsonArray pathJsonArray=pathJsonObject.value("path").toArray();
	jsonFile.close();
	if (pathJsonArray.size()<2)
	{
		return (false);
	}
	for (const QJsonValue &segmentJsonValue : pathJsonArray)
	{
		QJsonObject segmentJsonObject=segmentJsonValue.toObject();
		TrajectorySegment segment;
		if(segmentJsonObject.value("type").toString().toLower()==QStringLiteral("free"))
		{
			segment.type=TrajectorySegment::SegmentType::Free;
		}
		else if(segmentJsonObject.value("type").toString().toLower()==QStringLiteral("spline"))
		{
			segment.type=TrajectorySegment::SegmentType::Spline;
		}
		else if(segmentJsonObject.value("type").toString().toLower()==QStringLiteral("arc"))
		{
			segment.type=TrajectorySegment::SegmentType::Arc;
		}
		else // "line" is a default type
		{
			segment.type=TrajectorySegment::SegmentType::Line;
		}
		if (mSegments.size()==0)
		{
			segment.speed=segmentJsonObject.value("speed").toDouble(1.0);
			QJsonValue aPointJsonValue=segmentJsonObject.value("a");
			if(aPointJsonValue.isArray())
			{
				QJsonArray aPointJsonArray=aPointJsonValue.toArray();
				segment.positionA.setX(aPointJsonArray.at(0).toDouble());
				segment.positionA.setY(aPointJsonArray.at(1).toDouble());
				segment.positionA.setZ(aPointJsonArray.at(2).toDouble());
				segment.orientationA.setX(aPointJsonArray.at(3).toDouble());
				segment.orientationA.setY(aPointJsonArray.at(4).toDouble());
				segment.orientationA.setZ(aPointJsonArray.at(5).toDouble());
			}
		}
		else
		{
			segment.speed=segmentJsonObject.value("speed").toDouble(mSegments.back().speed);
			segment.positionA.setX(mSegments.back().positionB.x());
			segment.positionA.setY(mSegments.back().positionB.y());
			segment.positionA.setZ(mSegments.back().positionB.z());
			segment.orientationA.setX(mSegments.back().orientationB.x());
			segment.orientationA.setY(mSegments.back().orientationB.y());
			segment.orientationA.setZ(mSegments.back().orientationB.z());
		}
		QJsonValue bPointJsonValue=segmentJsonObject.value("b");
		if(bPointJsonValue.isArray())
		{
			QJsonArray bPointJsonArray=bPointJsonValue.toArray();
			segment.positionB.setX(bPointJsonArray.at(0).toDouble());
			segment.positionB.setY(bPointJsonArray.at(1).toDouble());
			segment.positionB.setZ(bPointJsonArray.at(2).toDouble());
			segment.orientationB.setX(bPointJsonArray.at(3).toDouble());
			segment.orientationB.setY(bPointJsonArray.at(4).toDouble());
			segment.orientationB.setZ(bPointJsonArray.at(5).toDouble());
		}
		QJsonValue cPointJsonValue=segmentJsonObject.value("c");
		if(cPointJsonValue.isArray())
		{
			QJsonArray cPointJsonArray=cPointJsonValue.toArray();
			segment.positionC.setX(cPointJsonArray.at(0).toDouble());
			segment.positionC.setY(cPointJsonArray.at(1).toDouble());
			segment.positionC.setZ(cPointJsonArray.at(2).toDouble());
			segment.orientationC.setX(cPointJsonArray.at(3).toDouble());
			segment.orientationC.setY(cPointJsonArray.at(4).toDouble());
			segment.orientationC.setZ(cPointJsonArray.at(5).toDouble());
		}
		mSegments.push_back(segment);
	}
	emit trajectoryChanged();
	return (true);
}

// TODO:
bool QTrajectoryPlanner::saveToJsonFile(const QString &file)
{
	return (true);
}

void QTrajectoryPlanner::setStepSize(double step_size)
{
	if(step_size<QTrajectoryPlanner::StepSizeMin)
	{
		step_size=QTrajectoryPlanner::StepSizeMin;
	}
	else if(step_size>QTrajectoryPlanner::StepSizeMax)
	{
		step_size=QTrajectoryPlanner::StepSizeMax;
	}
	mStepSize=step_size;
}

const QVector<TrajectorySegment> &QTrajectoryPlanner::getSegments() const
{
	return (mSegments);
}

const QVector<QRobot::Pose> &QTrajectoryPlanner::getPoses() const
{
	return (mPoses);
}

const QVector<QVector3D> &QTrajectoryPlanner::getWristPath() const
{
	return (mWristPath);
}

const QVector<QVector3D> &QTrajectoryPlanner::getTooltipPath() const
{
	return (mTooltipPath);
}

void QTrajectoryPlanner::addSegment(const TrajectorySegment &segment)
{
	mSegments.push_back(segment);
	emit trajectoryChanged();
}

void QTrajectoryPlanner::setAnimationSpeed(int animation_speed)
{
	if(animation_speed<QTrajectoryPlanner::AnimationSpeedMin)
	{
		animation_speed=QTrajectoryPlanner::AnimationSpeedMin;
	}
	else if(animation_speed>QTrajectoryPlanner::AnimationSpeedMax)
	{
		animation_speed=QTrajectoryPlanner::AnimationSpeedMax;
	}
	int newFrameInterval=(QTrajectoryPlanner::AnimationSpeedMax+1)-animation_speed;
	mAnimationFrameInterval=newFrameInterval;
	if(mAnimationTimer.isActive())
	{
		mAnimationTimer.start(newFrameInterval, this);
	}
}

void QTrajectoryPlanner::startAnimation()
{
	if(mAnimationTimer.isActive())
	{
		return;
	}
	mAnimationTimer.start(mAnimationFrameInterval, this);
}

void QTrajectoryPlanner::stopAnimation()
{
	if(!mAnimationTimer.isActive())
	{
		return;
	}
	mAnimationTimer.stop();
}
