#ifndef TRAJECTORYPLANNER_HPP
#define TRAJECTORYPLANNER_HPP

#include <QObject>
#include <QVector3D>
#include <QVector>
#include <QString>
#include <QBasicTimer>
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
	QBasicTimer mAnimationTimer;
	QVector<TrajectorySegment> mSegments;
	QVector<QRobot::Pose> mPoses;
	QRobot *mRobot=nullptr;
	double mStepSize=0.25; // mm
	int mAnimationProgress=0;
	int mAnimationFrameInterval=50;
	void timerEvent(QTimerEvent *event) override;
	void rebuildPoses();

public:
	static constexpr double StepSizeMin=1.0/1024.0;
	static constexpr double StepSizeMax=1024.0;
	QTrajectoryPlanner(QObject *parent = nullptr);
	void attachRobot(QRobot *robot);
	void clear();
	bool loadFromJsonFile(const QString &file);
	bool saveToJsonFile(const QString &file);
	void setStepSize(double step_size);
	const QVector<QRobot::Pose> &getPoses() const;

public slots:
	void addPathSegment(const TrajectorySegment &segment);
	void setAnimationSpeed(int speed);
	void startAnimation();

signals:
	void planningFinished(int totalPoints, int failedPoints);
	void planningError(const QString &message);
	void needToSetPose(const QRobot::Pose &pose);
	void animationFinished();
};

#endif // TRAJECTORYPLANNER_HPP
