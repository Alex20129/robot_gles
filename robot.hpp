#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QBasicTimer>

class QRobot : public QObject
{
	Q_OBJECT
	QBasicTimer mAnimationTimer;
	QVector <double> mJointAngles; // deg
	QVector <double> mJointLimitMin; // deg
	QVector <double> mJointLimitMax; // deg
	QVector <double> mLinkLengths; // mm
	QVector <QMatrix4x4> mLinkMatrices;
	QMatrix4x4 mTargetMatrix;
	QMatrix4x4 mWristMatrix;
	QMatrix4x4 mFlangeMatrix;
	QVector3D mStartPosition;
	QVector3D mTargetPosition;
	QQuaternion mTargetOrientation;
	uint32_t ikIterationsPerCycle=32;
	double mAnimationProgress;
	double mAnimationStep;
	bool ikSolved=true;
	void recalculateLinkMatrices();
	void recalculateTargetMatrix();
	void timerEvent(QTimerEvent *event) override;

public:
	static constexpr double ikInitialStep=0.5; // deg
	static constexpr double ikSlowdownCoefficient=-0.75;
	static constexpr int numOfJoints=6;
	QRobot(QObject *parent=nullptr);

	double getJointAngle(int joint_index) const;
	QPair<qreal, qreal> getJointLimits(int joint_index) const;
	const QMatrix4x4 &getLinkMatrix(int linkIndex) const;
	const QMatrix4x4 &getTargetMatrix() const;
	QVector3D getFlangePosition() const;
	QQuaternion getFlangeOrientation() const;
	const QVector3D &getTargetPosition() const;
	const QQuaternion &getTargetOrientation() const;
	void setJointLimits(int joint_index, double min_deg, double max_deg);
	void setLinkLength(int link_index, double mm);

public slots:
	void setJointAngle(int joint_index, double deg);
	void setTargetPosition(const QVector3D &target_position);
	void setTargetOrientation(const QQuaternion &target_orientation);
	void solveInverseKinematics(const QVector3D &position);
	void startAnimation();

signals:
	void configurationChanged();
	void targetPositionChanged();
	void animationFinished();
};

#endif // ROBOT_HPP
