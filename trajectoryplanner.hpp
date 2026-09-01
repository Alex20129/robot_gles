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
	QVector<QVector3D> mTooltipPath;
	QRobot *mRobot=nullptr;
	double mStepSize=0.25; // mm
	int mAnimationProgress=0;
	int mAnimationFrameInterval=50;
	void timerEvent(QTimerEvent *event) override;

public:
	static constexpr double StepSizeMin=1.0/1024.0;
	static constexpr double StepSizeMax=1024.0;
	static constexpr int AnimationSpeedMin=0;
	static constexpr int AnimationSpeedMax=255;
	QTrajectoryPlanner(QObject *parent = nullptr);
	void attachRobot(QRobot *robot);
	void rebuildPoses();
	void clear();
	bool loadFromJsonFile(const QString &file);
	bool saveToJsonFile(const QString &file);
	void setStepSize(double step_size);
	const QVector<TrajectorySegment> &getSegments() const;
	const QVector<QRobot::Pose> &getPoses() const;
	const QVector<QVector3D> &GetTooltipPath() const;

public slots:
	void addSegment(const TrajectorySegment &segment);
	void setAnimationSpeed(int animation_speed);
	void startAnimation();

signals:
	void trajectoryChanged();
	void planningFinished();
	void planningError(const QString &message);
	void needToSetPose(const QRobot::Pose &pose);
	void animationFinished();
};

#endif // TRAJECTORYPLANNER_HPP
