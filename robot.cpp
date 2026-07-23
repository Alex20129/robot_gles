#include "robot.hpp"
#include <cmath>
#include <QTimerEvent>

void QRobot::timerEvent(QTimerEvent *event)
{
	event->accept();
	bool animFinished=false;
	if (mIkPositionSolved && mIkOrientationSolved)
	{
		mAnimationProgress+=mAnimationStep;
		if (mAnimationProgress>=1.0)
		{
			mAnimationProgress=1.0;
			mAnimationTimer.stop();
			animFinished=true;
		}
		mIkPositionSolved=false;
		mIkOrientationSolved=false;
		solveIkForPosition(mStartPosition*(1.0-mAnimationProgress) + mTargetPosition*mAnimationProgress);
		solveIkForOrientation(mStartOrientation*(1.0-mAnimationProgress) + mTargetOrientation*mAnimationProgress);
	}
	if (animFinished)
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

static double vectorDiffSq(const QVector3D &va, const QVector3D &vb)
{
	double diffX=va.x()-vb.x();
	double diffY=va.y()-vb.y();
	double diffZ=va.z()-vb.z();
	return (diffX*diffX + diffY*diffY + diffZ*diffZ);
}

void QRobot::solveIkForPosition(const QVector3D &position)
{
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	bool improved=true;
	double currentDiff=vectorDiffSq(position, getWristPosition());
	static uint32_t ikIterations=0;
	while (improved)
	{
		improved=false;
		for (uint32_t ikIteration=0; !improved && ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=0; j < 3; ++j)
			{
				const double oldAngle=mJointAngles[j];
				mJointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDiff=vectorDiffSq(position, getWristPosition());
				if (currentDiff > newDiff)
				{
					currentDiff=newDiff;
					improved=true;
				}
				else
				{
					mJointAngles[j] = oldAngle;
					recalculateLinkMatrices(j);
					ikStep[j] *= ikSlowdownCoefficient;
				}
			}
			ikIterations++;
		}
	}
	qDebug()<<"ikIterations:"<<ikIterations;
	emit configurationChanged();
	mIkPositionSolved=true;
}

static double quaternionDiffSq(const QQuaternion &qa, const QQuaternion &qb)
{
	double diffS=qa.scalar()-qb.scalar();
	double diffX=qa.x()-qb.x();
	double diffY=qa.y()-qb.y();
	double diffZ=qa.z()-qb.z();
	return (diffS*diffS + diffX*diffX + diffY*diffY + diffZ*diffZ);
}

void QRobot::solveIkForOrientation(const QQuaternion &orientation)
{
	QQuaternion NormalizedOrientation=orientation.normalized();
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	bool improved=true;
	double currentDiff=quaternionDiffSq(NormalizedOrientation, getWristOrientation());
	while (improved)
	{
		improved=false;
		for (uint32_t ikIteration=0; !improved && ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=3; j < 6; ++j)
			{
				const double oldAngle=mJointAngles[j];
				mJointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDiff=quaternionDiffSq(NormalizedOrientation, getWristOrientation());
				if (currentDiff > newDiff)
				{
					currentDiff=newDiff;
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
	mIkOrientationSolved=true;
}

void QRobot::startAnimation()
{
	mStartPosition=getWristPosition();
	mStartOrientation=getWristOrientation();
	mAnimationProgress=0.0;
	float length=(mTargetPosition-mStartPosition).length();
	if (length>0.0)
	{
		mAnimationStep=1.0/length;
	}
	else
	{
		mAnimationStep=1.0/16.0;
	}
	mAnimationTimer.start(10, this);
}

void QRobot::setJointLimits(uint32_t joint_index, double min_deg, double max_deg)
{
	mAnimationTimer.stop();
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return;
	}
	mJointLimitMin[joint_index]=qMin(min_deg, max_deg);
	mJointLimitMax[joint_index]=qMax(min_deg, max_deg);
}

void QRobot::setLinkLength(uint32_t link_index, double mm)
{
	mAnimationTimer.stop();
	if (link_index < 0 || link_index >= numOfJoints)
	{
		return;
	}
	if (mLinkLengths[link_index] != mm)
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

void QRobot::setJointAngle(uint32_t joint_index, double deg)
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

void QRobot::setTargetPosition(float x, float y, float z)
{
	QVector3D newTargetPosition(x, y, z);
	if (mTargetPosition!=newTargetPosition)
	{
		mTargetPosition=newTargetPosition;
		recalculateTargetMatrix();
		emit targetPositionChanged();
		if (mAnimationTimer.isActive())
		{
			startAnimation();
		}
	}
}

void QRobot::setTargetOrientation(float pitch, float yaw, float roll)
{
	QQuaternion newTargetOrientation=QQuaternion::fromEulerAngles(pitch, yaw, roll).normalized();
	if (mTargetOrientation!=newTargetOrientation)
	{
		mTargetOrientation=newTargetOrientation;
		recalculateTargetMatrix();
		emit targetPositionChanged();
		if (mAnimationTimer.isActive())
		{
			startAnimation();
		}
	}
}

double QRobot::getJointAngle(uint32_t joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return (0.0);
	}
	return (mJointAngles[joint_index]);
}

QPair<qreal, qreal> QRobot::getJointLimits(uint32_t joint_index) const
{
	if (joint_index < 0 || joint_index >= numOfJoints)
	{
		return{0.0, 0.0};
	}
	return{mJointLimitMin[joint_index], mJointLimitMax[joint_index]};
}

const QMatrix4x4 &QRobot::getLinkMatrix(int link_index) const
{
	return (mLinkMatrices[link_index]);
}

const QMatrix4x4 &QRobot::getTargetMatrix() const
{
	return (mTargetMatrix);
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
