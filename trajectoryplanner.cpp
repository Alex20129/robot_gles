#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "trajectoryplanner.hpp"

QTrajectoryPlanner::QTrajectoryPlanner(QRobot *robot, QObject *parent) : QObject(parent)
{
	mRobot=robot;
}

void QTrajectoryPlanner::clear()
{
	mSegments.clear();
	mPoses.clear();
}

void QTrajectoryPlanner::generatePoses(const TrajectorySegment &segment)
{
	switch (segment.type)
	{
		default:
		case TrajectorySegment::SegmentType::Line:
		{
			break;
		}
		case TrajectorySegment::SegmentType::Arc:
		{
			break;
		}
		case TrajectorySegment::SegmentType::Spline:
		{
			break;
		}
	}
}

void QTrajectoryPlanner::addSegment(const TrajectorySegment &segment)
{
	mSegments.push_back(segment);
	generatePoses(segment);
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
	bool itsAfirstSegment=true;
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
		if (itsAfirstSegment)
		{
			itsAfirstSegment=false;
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
		addSegment(segment);
	}

	return (true);
}

void QTrajectoryPlanner::setVelocity(double mmps)
{
	mVelocity=mmps;
}

void QTrajectoryPlanner::setStep(double mm)
{
	mStep=mm;
}

const QVector<QRobot::Pose> &QTrajectoryPlanner::getPoses() const
{
	return (mPoses);
}
