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
	QVector <double> mJointAngles; // rad
	QVector <double> mJointLimitMin; // rad
	QVector <double> mJointLimitMax; // rad
	QVector <double> mLinkLengths; // mm
	QVector <QMatrix4x4> mLinkMatrices;
	QMatrix4x4 mTargetMatrix;
	QMatrix4x4 mEEMatrix;
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
	static constexpr double ikInitialStep=1.0/16.0; // rad
	static constexpr double ikSlowdownCoefficient=-0.75;
	static constexpr int numOfJoints=6;
	QRobot(QObject *parent=nullptr);

	void setJointLimits(int joint_index, double min_deg, double max_deg);

	double getJointAngle(int joint_index) const;
	QPair<qreal, qreal> getJointLimits(int joint_index) const;
	const QMatrix4x4 &getLinkMatrix(int linkIndex) const;
	const QMatrix4x4 &getTargetMatrix() const;
	QVector3D getToolPosition() const;
	QQuaternion getToolOrientation() const;
	const QVector3D &getTargetPosition() const;
	const QQuaternion &getTargetOrientation() const;

public slots:
	void setJointAngle(int jointIndex, double deg);
	void setLinkLength(int linkIndex, double mm);
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
