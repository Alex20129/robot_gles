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
	QVector3D mStartPosition;
	QVector3D mTargetPosition;
	QQuaternion mStartOrientation;
	QQuaternion mTargetOrientation;
	double mFlangeOffset=80.0;
	double mToolOffset=0.0;
	double mAnimationProgress=0.0;
	double mAnimationStep=0.0;
	bool mIkPositionSolved=true;
	bool mIkOrientationSolved=true;
	void recalculateLinkMatrices(uint32_t from);
	void recalculateTargetMatrix();
	void timerEvent(QTimerEvent *event) override;

public:
	static constexpr double ikInitialStep=0.5; // deg
	static constexpr double ikSlowdownCoefficient=-0.75;
	static constexpr uint32_t ikIterationsPerCycle=16;
	static constexpr uint32_t numOfJoints=6;
	QRobot(QObject *parent=nullptr);

	double getJointAngle(uint32_t joint_index) const;
	QPair<qreal, qreal> getJointLimits(uint32_t joint_index) const;
	const QMatrix4x4 &getLinkMatrix(int link_index) const;
	const QMatrix4x4 &getTargetMatrix() const;
	QVector3D getWristPosition() const;
	QQuaternion getWristOrientation() const;
	const QVector3D &getTargetPosition() const;
	const QQuaternion &getTargetOrientation() const;
	void setJointLimits(uint32_t joint_index, double min_deg, double max_deg);
	void setLinkLength(uint32_t link_index, double mm);
	void setFlangeOffset(double mm);
	void setToolOffset(double mm);

public slots:
	void setJointAngle(uint32_t joint_index, double deg);
	void setTargetPosition(float x, float y, float z);
	void setTargetOrientation(float pitch, float yaw, float roll);
	void solveIkForPosition(const QVector3D &position);
	void solveIkForOrientation(const QQuaternion &orientation);
	void startAnimation();

signals:
	void configurationChanged();
	void targetPositionChanged();
	void animationFinished();
};

#endif // ROBOT_HPP
