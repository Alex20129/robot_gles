#ifndef TRAJECTORYPLANNER_HPP
#define TRAJECTORYPLANNER_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QVector>
#include <QString>
#include "robot.hpp"

struct TrajectoryPoint
{
	uint64_t timestamp; // ms
	qreal joints[6]; // deg
};

struct TrajectorySegment
{
	enum Type { Line, Arc, Spline };
	Type type;
	QVector3D a, b, c;
	QVector3D control;
};

class QTrajectoryPlanner : public QObject
{
	Q_OBJECT

	QRobot *mRobot;
	QVector<TrajectorySegment> mSegments;
	QVector<TrajectoryPoint> mPoints;
	double mVelocity=10.0; // mm/s
	double mStep=0.25; // mm

	void generatePoints();

public:
	QTrajectoryPlanner(QRobot *robot, QObject *parent = nullptr);

	void clear();
	void addSegment(const TrajectorySegment &segment);
	bool loadFromJsonFile(const QString &file);
	void setVelocity(double mmps);
	void setStep(double mm);
	const QVector<TrajectoryPoint> &getPoints() const;

signals:
	void planningFinished(int totalPoints, int failedPoints);
	void planningError(const QString &message);
};

#endif // TRAJECTORYPLANNER_HPP
