#include "robot.hpp"
#include <cmath>
#include <QTimerEvent>

void QRobot::timerEvent(QTimerEvent *event)
{
	event->accept();
	bool animFinished=false;
	if(ikSolved)
	{
		mAnimationProgress+=mAnimationStep;
		if(mAnimationProgress>=1.0)
		{
			mAnimationProgress=1.0;
			mAnimationTimer.stop();
			animFinished=true;
		}
		ikSolved=false;
		solveInverseKinematics(mStartPosition+(mTargetPosition-mStartPosition)*mAnimationProgress);
	}
	if(animFinished)
	{
		emit animationFinished();
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
		-180.0,
		-90.0,
		-120.0,
		-180.0,
		-90.0,
		-180.0,
	};
	mJointLimitMax =
	{
		180.0,
		90.0,
		120.0,
		180.0,
		90.0,
		180.0,
	};
	mLinkLengths =
	{
		90.0,
		200.0,
		210.0,
		0.0,
		0.0,
		0.0,
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
	recalculateLinkMatrices(0);
}

void QRobot::recalculateLinkMatrices(uint32_t from)
{
	switch (from)
	{
		default:
		case 0:
		{
			mLinkMatrices[0].setToIdentity();
			mLinkMatrices[0].rotate(mJointAngles[0], 0, 0, 1);
			mLinkMatrices[0].translate(0, 0, mLinkLengths[0]);
		}
		case 1:
		{
			mLinkMatrices[1]=mLinkMatrices[0];
			mLinkMatrices[1].rotate(mJointAngles[1], 1, 0, 0);
			mLinkMatrices[1].translate(0, 0, mLinkLengths[1]);
		}
		case 2:
		{
			mLinkMatrices[2]=mLinkMatrices[1];
			mLinkMatrices[2].rotate(mJointAngles[2], 1, 0, 0);
			mLinkMatrices[2].translate(0, 0, mLinkLengths[2]);
		}
		case 3:
		{
			mLinkMatrices[3]=mLinkMatrices[2];
			mLinkMatrices[3].rotate(mJointAngles[3], 0, 0, 1);
		}
		case 4:
		{
			mLinkMatrices[4]=mLinkMatrices[3];
			mLinkMatrices[4].rotate(mJointAngles[4], 1, 0, 0);
		}
		case 5:
		{
			mLinkMatrices[5]=mLinkMatrices[4];
			mLinkMatrices[5].rotate(mJointAngles[5], 0, 0, 1);
		}
	}
}

void QRobot::recalculateTargetMatrix()
{
	mTargetMatrix.setToIdentity();
	mTargetMatrix.translate(
		mTargetPosition.x(),
		mTargetPosition.y(),
		mTargetPosition.z());
	mTargetMatrix.rotate(mTargetOrientation);
}

void QRobot::solveInverseKinematics(const QVector3D &position)
{
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	QVector3D wristPosition=getWristPosition();
	bool improved=true;
	double wristPositionAngleDeg = std::atan2(wristPosition.y(), wristPosition.x()) * 180.0 / M_PI;
	double targetPositionAngleDeg = std::atan2(position.y(), position.x()) * 180.0 / M_PI;
	mJointAngles[0]=qBound(mJointLimitMin[0], mJointAngles[0] + targetPositionAngleDeg - wristPositionAngleDeg, mJointLimitMax[0]);
	recalculateLinkMatrices(0);
	double currentDistance=(position - getWristPosition()).length();
	while (improved)
	{
		improved=false;
		for (uint32_t ikIteration=0; !improved && ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=1; j < 3; ++j)
			{
				const double oldAngle=mJointAngles[j];
				mJointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDistance=(position - getWristPosition()).length();
				if (currentDistance > newDistance)
				{
					currentDistance=newDistance;
					improved=true;
				}
				else
				{
					mJointAngles[j] = oldAngle;
					recalculateLinkMatrices(j);
					ikStep[j] *= ikSlowdownCoefficient;
				}
			}
		}
	}
	emit configurationChanged();
	ikSolved=true;
}

void QRobot::startAnimation()
{
	mStartPosition=getWristPosition();
	//mStartOrientation=getWristOrientation();
	mAnimationProgress=0.0;
	float length=(mTargetPosition-mStartPosition).length();
	if(length>0.0)
	{
		mAnimationStep=1.0/length;
	}
	else
	{
		mAnimationStep=1.0/16.0;
	}
	mAnimationTimer.start(10, this);
}

void QRobot::setJointLimits(int joint_index, double min_deg, double max_deg)
{
	mAnimationTimer.stop();
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return;
	}
	mJointLimitMin[joint_index]=qMin(min_deg, max_deg);
	mJointLimitMax[joint_index]=qMax(min_deg, max_deg);
}

void QRobot::setLinkLength(int link_index, double mm)
{
	mAnimationTimer.stop();
	if (link_index < 0 || link_index >= numOfJoints)
	{
		return;
	}
	if(mLinkLengths[link_index] != mm)
	{
		mLinkLengths[link_index]=mm;
	}
}

void QRobot::setFlangeOffset(double mm)
{
	mFlangeOffset=mm;
}

void QRobot::setToolOffset(double mm)
{
	mToolOffset=mm;
}

void QRobot::setJointAngle(int joint_index, double deg)
{
	mAnimationTimer.stop();
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return;
	}
	deg=qBound(mJointLimitMin[joint_index], deg, mJointLimitMax[joint_index]);
	if (mJointAngles[joint_index] != deg)
	{
		mJointAngles[joint_index]=deg;
		recalculateLinkMatrices(joint_index);
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

double QRobot::getJointAngle(int joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return 0.0;
	}
	return mJointAngles[joint_index];
}

QPair<qreal, qreal> QRobot::getJointLimits(int joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return {0.0, 0.0};
	}
	return {mJointLimitMin[joint_index], mJointLimitMax[joint_index]};
}

const QMatrix4x4 &QRobot::getLinkMatrix(int link_index) const
{
	return mLinkMatrices[link_index];
}

const QMatrix4x4 &QRobot::getTargetMatrix() const
{
	return mTargetMatrix;
}

QVector3D QRobot::getWristPosition() const
{
	QVector3D positionVec(
		mLinkMatrices[2](0, 3),
		mLinkMatrices[2](1, 3),
		mLinkMatrices[2](2, 3));
	return positionVec;
}

QQuaternion QRobot::getWristOrientation() const
{
	QMatrix3x3 rotMat=mLinkMatrices[5].toGenericMatrix<3,3>();
	return QQuaternion::fromRotationMatrix(rotMat);
}

const QVector3D &QRobot::getTargetPosition() const
{
	return mTargetPosition;
}

const QQuaternion &QRobot::getTargetOrientation() const
{
	return mTargetOrientation;
}
