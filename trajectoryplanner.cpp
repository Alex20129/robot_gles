#include "trajectoryplanner.hpp"

QTrajectoryPlanner::QTrajectoryPlanner(QRobot *robot, QObject *parent) : QObject(parent)
{

}

bool QTrajectoryPlanner::loadFromJsonFile(const QString &file)
{
	return true;
}

void QTrajectoryPlanner::setVelocity(double mmps)
{
	mVelocity=mmps;
}

void QTrajectoryPlanner::setStep(double mm)
{
	mStep=mm;
}
