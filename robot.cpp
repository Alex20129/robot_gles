#include "robot.hpp"
#include <cmath>
#include <QTimerEvent>

constexpr double DEG2RAD=M_PI / 180.0;
constexpr double RAD2DEG=180.0 / M_PI;

void QRobot::timerEvent(QTimerEvent *event)
{
	event->accept();
	mAnimationProgress+=0.004;
	if(mAnimationProgress>1.0)
	{
		mAnimationProgress=1.0;
		mAnimationTimer.stop();
	}
	if(ikSolved)
	{
		ikSolved=false;
		solveInverseKinematics(mStartPosition+(mTargetPosition-mStartPosition)*mAnimationProgress);
	}
}

QRobot::QRobot(QObject *parent) : QObject(parent)
{
	mJointAngles.resize(numOfJoints);
	mJointLimitMin.resize(numOfJoints);
	mJointLimitMax.resize(numOfJoints);
	mLinkMatrices.resize(numOfJoints);

	mJointLimitMin =
	{
		-180 * DEG2RAD,
		-90 * DEG2RAD,
		-120 * DEG2RAD,
		-180 * DEG2RAD,
		-90 * DEG2RAD,
		-180 * DEG2RAD,
	};

	mJointLimitMax =
	{
		180.0 * DEG2RAD,
		90.0 * DEG2RAD,
		120.0 * DEG2RAD,
		180.0 * DEG2RAD,
		90.0 * DEG2RAD,
		180.0 * DEG2RAD,
	};

	mLinkLengths =
	{
		90.0,
		200.0,
		120.0,
		90.0,
		120.0,
		205.0,
	};

	mJointAngles =
	{
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
		0.0,
	};

	recalculateLinkMatrices();
}

void QRobot::recalculateLinkMatrices()
{
	QMatrix4x4 fkMatrix;

	fkMatrix.rotate(mJointAngles[0] * RAD2DEG, 0,0,1);
	mLinkMatrices[0]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[0]);

	fkMatrix.rotate(mJointAngles[1] * RAD2DEG, 1,0,0);
	mLinkMatrices[1]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[1]);

	fkMatrix.rotate(mJointAngles[2] * RAD2DEG, 1,0,0);
	mLinkMatrices[2]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[2]);

	fkMatrix.rotate(mJointAngles[3] * RAD2DEG, 0,0,1);
	mLinkMatrices[3]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[3]);

	fkMatrix.rotate(mJointAngles[4] * RAD2DEG, 1,0,0);
	mLinkMatrices[4]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[4]);

	fkMatrix.rotate(mJointAngles[5] * RAD2DEG, 0,0,1);
	mLinkMatrices[5]=fkMatrix;
	fkMatrix.translate(0, 0, mLinkLengths[5]);

	mEEMatrix=fkMatrix;
}

void QRobot::recalculateTargetMatrix()
{
	mTargetMatrix.setToIdentity();
	mTargetMatrix.rotate(mTargetOrientation);
	mTargetMatrix.translate(
		mTargetPosition.x(),
		mTargetPosition.y(),
		mTargetPosition.z());
}

void QRobot::solveInverseKinematics(const QVector3D &position)
{
	QVector<double> ikStepRad(numOfJoints, ikInitialStep);
	bool improved=true;
	recalculateLinkMatrices();
	double currentDistance=(position - getToolPosition()).length();
	while (improved)
	{
		improved=false;
		for (uint32_t ikIteration=0; !improved && ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=0; j < numOfJoints; ++j)
			{
				const double oldAngle=mJointAngles[j];
				mJointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStepRad[j], mJointLimitMax[j]);
				recalculateLinkMatrices();
				const double newDistance=(position - getToolPosition()).length();
				if (currentDistance > newDistance)
				{
					currentDistance=newDistance;
					improved=true;
				}
				else
				{
					mJointAngles[j] = oldAngle;
					ikStepRad[j] *= ikSlowdownCoefficient;
				}
			}
		}
	}
	emit configurationChanged();
	ikSolved=true;
}

void QRobot::startAnimation()
{
	mStartPosition=getToolPosition();
	//mStartOrientation=getToolOrientation();
	mAnimationProgress=0.0;
	mAnimationTimer.start(10, this);
}

void QRobot::setJointAngle(int jointIndex, double deg)
{
	mAnimationTimer.stop();
	if (jointIndex < 0 || jointIndex >= numOfJoints)
	{
		return;
	}
	double rad=qBound(mJointLimitMin[jointIndex], deg * DEG2RAD, mJointLimitMax[jointIndex]);
	if (mJointAngles[jointIndex] != rad)
	{
		mJointAngles[jointIndex]=rad;
		recalculateLinkMatrices();
		emit configurationChanged();
	}
}

void QRobot::setLinkLength(int linkIndex, double mm)
{
	mAnimationTimer.stop();
	if (linkIndex < 0 || linkIndex >= numOfJoints)
	{
		return;
	}
	if(mLinkLengths[linkIndex] != mm)
	{
		mLinkLengths[linkIndex]=mm;
		recalculateLinkMatrices();
		emit configurationChanged();
	}
}

void QRobot::setTargetPosition(const QVector3D &target_position)
{
	if(mTargetPosition!=target_position)
	{
		mTargetPosition=target_position;
		recalculateTargetMatrix();
		emit targetPositionChanged();
		if(mAnimationTimer.isActive())
		{
			startAnimation();
		}
	}
}

void QRobot::setTargetOrientation(const QQuaternion &target_orientation)
{
	QQuaternion newTOnormal=target_orientation.normalized();
	if(mTargetOrientation!=newTOnormal)
	{
		mTargetOrientation=newTOnormal;
		recalculateTargetMatrix();
		emit targetPositionChanged();
		if(mAnimationTimer.isActive())
		{
			startAnimation();
		}
	}
}

double QRobot::jointAngle(int joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return 0.0;
	}
	return mJointAngles[joint_index] * RAD2DEG;
}

QPair<qreal, qreal> QRobot::getJointLimits(int joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return {0.0, 0.0};
	}
	return {mJointLimitMin[joint_index]*RAD2DEG, mJointLimitMax[joint_index]*RAD2DEG};
}

void QRobot::setJointLimits(int joint_index, double min_deg, double max_deg)
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return;
	}
	mJointLimitMin[joint_index]=qMin(min_deg, max_deg) * DEG2RAD;
	mJointLimitMax[joint_index]=qMax(min_deg, max_deg) * DEG2RAD;
}

const QMatrix4x4 &QRobot::getLinkMatrix(int linkIndex) const
{
	return mLinkMatrices[linkIndex];
}

const QMatrix4x4 &QRobot::getTargetMatrix() const
{
	return mTargetMatrix;
}

QVector3D QRobot::getToolPosition() const
{
	QVector3D positionVec(
		mEEMatrix(0, 3), // X
		mEEMatrix(1, 3), // Y
		mEEMatrix(2, 3)); // Z
	return positionVec;
}

QQuaternion QRobot::getToolOrientation() const
{
	float rotData[9] =
	{
		mEEMatrix(0, 0), mEEMatrix(0, 1), mEEMatrix(0, 2),
		mEEMatrix(1, 0), mEEMatrix(1, 1), mEEMatrix(1, 2),
		mEEMatrix(2, 0), mEEMatrix(2, 1), mEEMatrix(2, 2)
	};
	QMatrix3x3 rotM(rotData);
	return QQuaternion::fromRotationMatrix(rotM);
}

const QVector3D &QRobot::getTargetPosition() const
{
	return mTargetPosition;
}

const QQuaternion &QRobot::getTargetOrientation() const
{
	return mTargetOrientation;
}
