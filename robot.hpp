#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

class QRobot : public QObject
{
	Q_OBJECT
	QVector <double> mJointLimitMin; // deg
	QVector <double> mJointLimitMax; // deg
	QVector <double> mLinkLengths; // mm
	QVector <QMatrix4x4> mLinkMatrices;
	QMatrix4x4 mToolMatrix;
	QVector3D mStartPosition;
	QQuaternion mStartOrientation;
	double mFlangeOffset=80.0;
	double mToolOffset=110.0;
	void recalculateLinkMatrices(uint32_t from);

public:
	static constexpr double ikInitialStep=1.0; // deg
	static constexpr double ikSlowdownCoefficient=-0.75;
	static constexpr uint32_t ikIterationsPerCycle=16;
	static constexpr uint32_t numOfJoints=6;
	struct Pose
	{
		double jointAngles[numOfJoints]; // deg
	};
	QRobot(QObject *parent=nullptr);
	double getJointAngle(uint32_t joint_index) const;
	const QRobot::Pose &getPose() const;
	QPair<qreal, qreal> getJointLimits(uint32_t joint_index) const;
	const QMatrix4x4 &getLinkMatrix(uint32_t link_index) const;
	const QMatrix4x4 &getToolMatrix() const;
	QVector3D getWristPosition() const;
	QVector3D getFlangePosition() const;
	QVector3D getTooltipPosition() const;
	QQuaternion getToolOrientation() const;
	void setJointLimits(uint32_t joint_index, double min_deg, double max_deg);
	void setLinkLength(uint32_t link_index, double mm);
	void setFlangeOffset(double mm);
	void setToolOffset(double mm);
	double solveIkForPosition(const QVector3D &position, uint32_t cycles_limit=UINT32_MAX);
	double solveIkForOrientation(const QQuaternion &orientation, uint32_t cycles_limit=UINT32_MAX);

private:
	QRobot::Pose mPose;

public slots:
	void setJointAngle(uint32_t joint_index, double deg);
	void setPose(const QRobot::Pose &pose);

signals:
	void configurationChanged();
};

#endif // ROBOT_HPP
