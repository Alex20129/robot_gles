#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

class QRobot : public QObject
{
	Q_OBJECT
	QVector <double> mJointAngles; // rad
	QVector <double> mJointMin; // rad
	QVector <double> mJointMax; // rad
	QVector <double> mLinkLengths; // mm
	QVector <QMatrix4x4> mLinkMatrices;

public:
	static const int numOfJoints = 6;
	QRobot(QObject *parent = nullptr);

	double jointAngle(int jointIndex) const;
	void setJointLimits(int jointIndex, float minDeg, float maxDeg);
	const QMatrix4x4 &getLinkMatrix(int linkIndex) const;
	QVector3D getToolPosition() const;
	QQuaternion getToolOrientation() const;

public slots:
	void setJointAngle(int jointIndex, double deg);
	void setLinkLength(int linkIndex, double mm);
	void solveForwardKinematics();

signals:
	void configurationChanged();
};

#endif // ROBOT_HPP
