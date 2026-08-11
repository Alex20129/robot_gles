#ifndef TRAJECTORYPLANNER_HPP
#define TRAJECTORYPLANNER_HPP

#include <QObject>
#include <QVector3D>
#include <QVector>
#include <QString>
#include "robot.hpp"

struct TrajectorySegment
{
	enum SegmentType { Line, Arc, Spline };
	SegmentType type=SegmentType::Line;
	qreal speed=0.0;
	QVector3D positionA, positionB, positionC;
	QVector3D orientationA, orientationB, orientationC;
};

class QTrajectoryPlanner : public QObject
{
	Q_OBJECT

	QRobot *mRobot;
	QVector<TrajectorySegment> mSegments;
	QVector<QRobot::Pose> mPoses;
	double mVelocity=10.0; // mm/s
	double mStep=0.25; // mm

	void generatePoses(const TrajectorySegment &segment);

public:
	QTrajectoryPlanner(QRobot *robot, QObject *parent = nullptr);

	void clear();
	void addSegment(const TrajectorySegment &segment);
	bool loadFromJsonFile(const QString &file);
	void setVelocity(double mmps);
	void setStep(double mm);
	const QVector<QRobot::Pose> &getPoses() const;

signals:
	void planningFinished(int totalPoints, int failedPoints);
	void planningError(const QString &message);
};

#endif // TRAJECTORYPLANNER_HPP
