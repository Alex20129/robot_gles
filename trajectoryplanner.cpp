#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimerEvent>
#include "trajectoryplanner.hpp"

void QTrajectoryPlanner::timerEvent(QTimerEvent *event)
{
	event->accept();
	if (mAnimationProgress>=mPoses.size())
	{
		mAnimationTimer.stop();
		mAnimationProgress=0;
		emit animationFinished();
	}
	else
	{
		emit needToSetPose(mPoses[mAnimationProgress++]);
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
	rebuildPoses();
}

void QTrajectoryPlanner::clear()
{
	mSegments.clear();
	mPoses.clear();
}

void QTrajectoryPlanner::rebuildPoses()
{
	mPoses.clear();
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
				}
				break;
			}
			case TrajectorySegment::SegmentType::Arc:
			{
				// TODO: Arc
				break;
			}
			case TrajectorySegment::SegmentType::Spline:
			{
				// TODO: Spline
				break;
			}
		}
	}
	if (!mSegments.isEmpty())
	{
		const TrajectorySegment &lastSegment = mSegments.last();
		mRobot->solveIkForPosition(lastSegment.positionB);
		mRobot->solveIkForOrientation(QQuaternion::fromEulerAngles(lastSegment.orientationB));
		mPoses.push_back(mRobot->getPose());
	}
}

void QTrajectoryPlanner::addPathSegment(const TrajectorySegment &segment)
{
	mSegments.push_back(segment);
}

bool QTrajectoryPlanner::loadFromJsonFile(const QString &file)
{
	QFile jsonFile(file);
	if (!jsonFile.open(QIODevice::ReadOnly))
	{
		return (false);
	}
	QJsonDocument pathJsonDocument = QJsonDocument::fromJson(jsonFile.readAll());
	QJsonObject pathJsonObject = pathJsonDocument.object();
	QJsonArray pathJsonArray = pathJsonObject.value("path").toArray();
	jsonFile.close();
	if (pathJsonArray.size()<2)
	{
		return (false);
	}
	for (const QJsonValue &segmentJsonValue : pathJsonArray)
	{
		QJsonObject segmentJsonObject=segmentJsonValue.toObject();
		TrajectorySegment segment;
		if(segmentJsonObject.value("type").toString()==QStringLiteral("spline"))
		{
			segment.type=TrajectorySegment::SegmentType::Spline;
		}
		else if(segmentJsonObject.value("type").toString()==QStringLiteral("arc"))
		{
			segment.type=TrajectorySegment::SegmentType::Arc;
		}
		else // "line" is a default type
		{
			segment.type=TrajectorySegment::SegmentType::Line;
		}
		segment.speed = segmentJsonObject.value("speed").toDouble(1.0);
		if (mSegments.size()==0)
		{
			segment.positionA.setX(segmentJsonObject.value("a").toArray().at(0).toDouble());
			segment.positionA.setY(segmentJsonObject.value("a").toArray().at(1).toDouble());
			segment.positionA.setZ(segmentJsonObject.value("a").toArray().at(2).toDouble());
			segment.orientationA.setX(segmentJsonObject.value("a").toArray().at(3).toDouble());
			segment.orientationA.setY(segmentJsonObject.value("a").toArray().at(4).toDouble());
			segment.orientationA.setZ(segmentJsonObject.value("a").toArray().at(5).toDouble());
			segment.positionB.setX(segmentJsonObject.value("b").toArray().at(0).toDouble());
			segment.positionB.setY(segmentJsonObject.value("b").toArray().at(1).toDouble());
			segment.positionB.setZ(segmentJsonObject.value("b").toArray().at(2).toDouble());
			segment.orientationB.setX(segmentJsonObject.value("b").toArray().at(3).toDouble());
			segment.orientationB.setY(segmentJsonObject.value("b").toArray().at(4).toDouble());
			segment.orientationB.setZ(segmentJsonObject.value("b").toArray().at(5).toDouble());
			segment.positionC.setX(segmentJsonObject.value("c").toArray().at(0).toDouble());
			segment.positionC.setY(segmentJsonObject.value("c").toArray().at(1).toDouble());
			segment.positionC.setZ(segmentJsonObject.value("c").toArray().at(2).toDouble());
			segment.orientationC.setX(segmentJsonObject.value("c").toArray().at(3).toDouble());
			segment.orientationC.setY(segmentJsonObject.value("c").toArray().at(4).toDouble());
			segment.orientationC.setZ(segmentJsonObject.value("c").toArray().at(5).toDouble());
		}
		else
		{
			segment.positionA.setX(mSegments.back().positionB.x());
			segment.positionA.setY(mSegments.back().positionB.y());
			segment.positionA.setZ(mSegments.back().positionB.z());
			segment.orientationA.setX(mSegments.back().orientationB.x());
			segment.orientationA.setY(mSegments.back().orientationB.y());
			segment.orientationA.setZ(mSegments.back().orientationB.z());
			segment.positionB.setX(segmentJsonObject.value("b").toArray().at(0).toDouble());
			segment.positionB.setY(segmentJsonObject.value("b").toArray().at(1).toDouble());
			segment.positionB.setZ(segmentJsonObject.value("b").toArray().at(2).toDouble());
			segment.orientationB.setX(segmentJsonObject.value("b").toArray().at(3).toDouble());
			segment.orientationB.setY(segmentJsonObject.value("b").toArray().at(4).toDouble());
			segment.orientationB.setZ(segmentJsonObject.value("b").toArray().at(5).toDouble());
			segment.positionC.setX(segmentJsonObject.value("c").toArray().at(0).toDouble());
			segment.positionC.setY(segmentJsonObject.value("c").toArray().at(1).toDouble());
			segment.positionC.setZ(segmentJsonObject.value("c").toArray().at(2).toDouble());
			segment.orientationC.setX(segmentJsonObject.value("c").toArray().at(3).toDouble());
			segment.orientationC.setY(segmentJsonObject.value("c").toArray().at(4).toDouble());
			segment.orientationC.setZ(segmentJsonObject.value("c").toArray().at(5).toDouble());
		}
		addPathSegment(segment);
	}
	rebuildPoses();
	return (true);
}

void QTrajectoryPlanner::setStepSize(double mm)
{
	mStepSize=mm;
}

const QVector<QRobot::Pose> &QTrajectoryPlanner::getPoses() const
{
	return (mPoses);
}

void QTrajectoryPlanner::startAnimation()
{
	mAnimationProgress=0;
	mAnimationTimer.start(5, this);
}
