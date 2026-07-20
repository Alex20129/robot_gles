#include "robot.hpp"
#include <cmath>
#include <QTimerEvent>

void QRobot::timerEvent(QTimerEvent *event)
{
	event->accept();
	if(ikSolved)
	{
		mAnimationProgress+=mAnimationStep;
		if(mAnimationProgress>1.0)
		{
			mAnimationProgress=1.0;
			mAnimationTimer.stop();
			emit animationFinished();
		}
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
		-180,
		-90,
		-120,
		-180,
		-90,
		-180,
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
		80.0,
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
			mLinkMatrices[3].translate(0, 0, mLinkLengths[3]);
		}
		case 4:
		{
			mLinkMatrices[4]=mLinkMatrices[3];
			mLinkMatrices[4].rotate(mJointAngles[4], 1, 0, 0);
			mLinkMatrices[4].translate(0, 0, mLinkLengths[4]);
		}
		case 5:
		{
			mLinkMatrices[5]=mLinkMatrices[4];
			mLinkMatrices[5].rotate(mJointAngles[5], 0, 0, 1);
			mLinkMatrices[5].translate(0, 0, mLinkLengths[5]);
		}
	}
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
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	bool improved=true;
	recalculateLinkMatrices(0);
	double currentDistance=(position - getFlangePosition()).length();
	while (improved)
	{
		improved=false;
		for (uint32_t ikIteration=0; !improved && ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=0; j < numOfJoints; ++j)
			{
				const double oldAngle=mJointAngles[j];
				mJointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDistance=(position - getFlangePosition()).length();
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
	mStartPosition=getFlangePosition();
	//mStartOrientation=getFlangeOrientation();
	mAnimationProgress=0.0;
	mAnimationStep=1.0/(mTargetPosition-mStartPosition).length();
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

void QRobot::setJointAngle(int joint_index, double deg)
{
	mAnimationTimer.stop();
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return;
	}
	double rad=qBound(mJointLimitMin[joint_index], deg, mJointLimitMax[joint_index]);
	if (mJointAngles[joint_index] != rad)
	{
		mJointAngles[joint_index]=rad;
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

const QMatrix4x4 &QRobot::getLinkMatrix(int linkIndex) const
{
	return mLinkMatrices[linkIndex];
}

const QMatrix4x4 &QRobot::getTargetMatrix() const
{
	return mTargetMatrix;
}

QVector3D QRobot::getFlangePosition() const
{
	QVector3D positionVec(
		mLinkMatrices[5](0, 3), // X
		mLinkMatrices[5](1, 3), // Y
		mLinkMatrices[5](2, 3)); // Z
	return positionVec;
}

QQuaternion QRobot::getFlangeOrientation() const
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
