#include "trajectoryplanner.hpp"

QTrajectoryPlanner::QTrajectoryPlanner(QRobot *robot, QObject *parent) : QObject(parent)
{
	mRobot=robot;
}

void QTrajectoryPlanner::clear()
{
	mSegments.clear();
	mPoints.clear();
}

void QTrajectoryPlanner::addSegment(const TrajectorySegment &segment)
{
	mSegments.push_back(segment);
}

bool QTrajectoryPlanner::loadFromJsonFile(const QString &file)
{
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

const QVector<TrajectoryPoint> &QTrajectoryPlanner::getPoints() const
{
	return (mPoints);
}
