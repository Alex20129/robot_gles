#ifndef TRAJECTORYPLANNER_HPP
#define TRAJECTORYPLANNER_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QVector>
#include <QString>

class QRobot;

struct TrajectoryPoint
{
	uint64_t timestamp; // s
	double joints[6]; // deg
};

struct Segment
{
	enum Type { Line, Arc, Spline };
	Type type;
	QVector3D from;
	QVector3D to;
	QVector3D center;
	QVector<QVector3D> control;
};

class QTrajectoryPlanner : public QObject
{
	Q_OBJECT

	QRobot *mRobot;
	QVector<Segment> mSegments;
	QVector<TrajectoryPoint> mPoints;
	double mVelocity=10.0; // mm/s
	double mStep=0.25; // mm

	double segmentArcLength(const Segment &seg) const;
	void generatePoints();

public:
	QTrajectoryPlanner(QRobot *robot, QObject *parent = nullptr);

	bool loadFromJsonFile(const QString &file);
	void setVelocity(double mmps);
	void setStep(double mm);
	const QVector<TrajectoryPoint> &getPoints() const;

signals:
	void planningFinished(int totalPoints, int failedPoints);
	void planningError(const QString &message);
};

#endif // TRAJECTORYPLANNER_HPP
