#include "robot.hpp"
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

QRobot::QRobot(QObject *parent) : QObject(parent)
{
	mJointAngles.resize(numOfJoints);
	mJointMin.resize(numOfJoints);
	mJointMax.resize(numOfJoints);
	mLinkMatrices.resize(numOfJoints);

	mJointMin =
	{
		-180 * DEG2RAD,
		-90 * DEG2RAD,
		-90 * DEG2RAD,
		-180 * DEG2RAD,
		-180 * DEG2RAD,
		-180 * DEG2RAD,
	};

	mJointMax =
	{
		180 * DEG2RAD,
		90 * DEG2RAD,
		90 * DEG2RAD,
		180 * DEG2RAD,
		180 * DEG2RAD,
		180 * DEG2RAD,
	};

	mLinkLengths =
	{
		0.0,
		90.0,
		200.0,
		120.0,
		90.0,
		120.0,
	};

	mJointAngles =
	{
		0.0 * DEG2RAD,
		0.0 * DEG2RAD,
		0.0 * DEG2RAD,
		0.0 * DEG2RAD,
		0.0 * DEG2RAD,
		0.0 * DEG2RAD,
	};

	solveForwardKinematics();
}

void QRobot::solveForwardKinematics()
{
	QMatrix4x4 ikMat;

	ikMat.translate(0, 0, mLinkLengths[0]);
	ikMat.rotate(mJointAngles[0] * RAD2DEG, 0,0,1);
	mLinkMatrices[0] = ikMat;

	ikMat.translate(0, 0, mLinkLengths[1]);
	ikMat.rotate(mJointAngles[1] * RAD2DEG, 1,0,0);
	mLinkMatrices[1] = ikMat;

	ikMat.translate(0, 0, mLinkLengths[2]);
	ikMat.rotate(mJointAngles[2] * RAD2DEG, 1,0,0);
	mLinkMatrices[2] = ikMat;

	ikMat.translate(0, 0, mLinkLengths[3]);
	ikMat.rotate(mJointAngles[3] * RAD2DEG, 0,0,1);
	mLinkMatrices[3] = ikMat;

	ikMat.translate(0, 0, mLinkLengths[4]);
	ikMat.rotate(mJointAngles[4] * RAD2DEG, 1,0,0);
	mLinkMatrices[4] = ikMat;

	ikMat.translate(0, 0, mLinkLengths[5]);
	ikMat.rotate(mJointAngles[5] * RAD2DEG, 0,0,1);
	mLinkMatrices[5] = ikMat;

	emit configurationChanged();
}

void QRobot::setJointAngle(int jointIndex, double deg)
{
	if (jointIndex < 0 || jointIndex >= numOfJoints)
	{
		return;
	}
	double rad = qBound(mJointMin[jointIndex], deg * DEG2RAD, mJointMax[jointIndex]);
	if (mJointAngles[jointIndex] != rad)
	{
		mJointAngles[jointIndex] = rad;
	}
}

void QRobot::setLinkLength(int linkIndex, double mm)
{
	if (linkIndex < 0 || linkIndex >= numOfJoints)
	{
		return;
	}
	if(mLinkLengths[linkIndex] != mm)
	{
		mLinkLengths[linkIndex] = mm;
	}
}

double QRobot::jointAngle(int jointIndex) const
{
	if (jointIndex < 0 || jointIndex >= numOfJoints)
	{
		return 0.0;
	}
	return mJointAngles[jointIndex] * RAD2DEG;
}

void QRobot::setJointLimits(int jointIndex, float minDeg, float maxDeg)
{
	if (jointIndex < 0 || jointIndex >= numOfJoints)
	{
		return;
	}
	mJointMin[jointIndex] = qMin(minDeg, maxDeg) * DEG2RAD;
	mJointMax[jointIndex] = qMax(minDeg, maxDeg) * DEG2RAD;
}

const QMatrix4x4 &QRobot::getLinkMatrix(int linkIndex) const
{
	return mLinkMatrices.at(linkIndex);
}

QVector3D QRobot::getToolPosition() const
{
	QVector3D positionVec(
		mLinkMatrices[5](0, 3), // X
		mLinkMatrices[5](1, 3), // Y
		mLinkMatrices[5](2, 3)); // Z
	return positionVec;
}

QQuaternion QRobot::getToolOrientation() const
{
	float rotData[9] =
	{
		mLinkMatrices[5](0,0), mLinkMatrices[5](0,1), mLinkMatrices[5](0,2),
		mLinkMatrices[5](1,0), mLinkMatrices[5](1,1), mLinkMatrices[5](1,2),
		mLinkMatrices[5](2,0), mLinkMatrices[5](2,1), mLinkMatrices[5](2,2)
	};
	QMatrix3x3 rotM(rotData);
	return QQuaternion::fromRotationMatrix(rotM);
}
