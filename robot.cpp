#include "robot.hpp"
#include <cmath>

QRobot::QRobot(QObject *parent) : QObject(parent)
{
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
	for (uint32_t joint=0; joint<QRobot::numOfJoints; joint++)
	{
		mPose.jointAngles[joint] = 0.0;
	}
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
			mLinkMatrices[0].rotate(mPose.jointAngles[0], 0, 0, 1);
			mLinkMatrices[0].translate(0, 0, mLinkLengths[0]);
		}
		case 1:
		{
			mLinkMatrices[1]=mLinkMatrices[0];
			mLinkMatrices[1].rotate(mPose.jointAngles[1], 1, 0, 0);
			mLinkMatrices[1].translate(0, 0, mLinkLengths[1]);
		}
		case 2:
		{
			mLinkMatrices[2]=mLinkMatrices[1];
			mLinkMatrices[2].rotate(mPose.jointAngles[2], 1, 0, 0);
			mLinkMatrices[2].translate(0, 0, mLinkLengths[2]);
		}
		case 3:
		{
			mLinkMatrices[3]=mLinkMatrices[2];
			mLinkMatrices[3].rotate(mPose.jointAngles[3], 0, 0, 1);
		}
		case 4:
		{
			mLinkMatrices[4]=mLinkMatrices[3];
			mLinkMatrices[4].rotate(mPose.jointAngles[4], 1, 0, 0);
		}
		case 5:
		{
			mLinkMatrices[5]=mLinkMatrices[4];
			mLinkMatrices[5].rotate(mPose.jointAngles[5], 0, 0, 1);
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

void QRobot::setJointLimits(uint32_t joint_index, double min_deg, double max_deg)
{
	if (joint_index >= numOfJoints)
	{
		return;
	}
	mJointLimitMin[joint_index]=qMin(min_deg, max_deg);
	mJointLimitMax[joint_index]=qMax(min_deg, max_deg);
}

void QRobot::setLinkLength(uint32_t link_index, double mm)
{
	if (link_index >= numOfJoints)
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

static double vectorDiffSq(const QVector3D &va, const QVector3D &vb)
{
	double diffX=va.x()-vb.x();
	double diffY=va.y()-vb.y();
	double diffZ=va.z()-vb.z();
	return (diffX*diffX + diffY*diffY + diffZ*diffZ);
}

double QRobot::solveIkForPosition(const QVector3D &position)
{
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	double DiffSq=vectorDiffSq(position, getWristPosition());
	bool Improved=true;
	while (Improved)
	{
		Improved=false;
		for (uint32_t ikIteration=0; ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=0; j < 3; ++j)
			{
				const double oldAngle=mPose.jointAngles[j];
				mPose.jointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDiff=vectorDiffSq(position, getWristPosition());
				if (DiffSq > newDiff)
				{
					DiffSq=newDiff;
					Improved=true;
				}
				else
				{
					mPose.jointAngles[j]=oldAngle;
					recalculateLinkMatrices(j);
					ikStep[j] *= ikSlowdownCoefficient;
				}
			}
		}
	}
	return (DiffSq);
}

static double quaternionDiffSq(const QQuaternion &qa, const QQuaternion &qb)
{
	QQuaternion b;
	if (QQuaternion::dotProduct(qa, qb) > 0.0)
	{
		b=qb;
	}
	else
	{
		b=-qb;
	}
	double diffS=qa.scalar()-b.scalar();
	double diffX=qa.x()-b.x();
	double diffY=qa.y()-b.y();
	double diffZ=qa.z()-b.z();
	return (diffS*diffS + diffX*diffX + diffY*diffY + diffZ*diffZ);
}

double QRobot::solveIkForOrientation(const QQuaternion &orientation)
{
	QQuaternion NormalizedOrientation=orientation.normalized();
	QVector<double> ikStep(numOfJoints, ikInitialStep);
	double DiffSq=quaternionDiffSq(NormalizedOrientation, getWristOrientation());
	bool Improved=true;
	while (Improved)
	{
		Improved=false;
		for (uint32_t ikIteration=0; ikIteration<ikIterationsPerCycle; ikIteration++)
		{
			for (int j=3; j < 6; ++j)
			{
				const double oldAngle=mPose.jointAngles[j];
				mPose.jointAngles[j]=qBound(mJointLimitMin[j], oldAngle + ikStep[j], mJointLimitMax[j]);
				recalculateLinkMatrices(j);
				const double newDiff=quaternionDiffSq(NormalizedOrientation, getWristOrientation());
				if (DiffSq > newDiff)
				{
					DiffSq=newDiff;
					Improved=true;
				}
				else
				{
					mPose.jointAngles[j]=oldAngle;
					recalculateLinkMatrices(j);
					ikStep[j] *= ikSlowdownCoefficient;
				}
			}
		}
	}
	return (DiffSq);
}

void QRobot::setJointAngle(uint32_t joint_index, double deg)
{
	if (joint_index >= numOfJoints)
	{
		return;
	}
	deg=qBound(mJointLimitMin[joint_index], deg, mJointLimitMax[joint_index]);
	if (mPose.jointAngles[joint_index] != deg)
	{
		mPose.jointAngles[joint_index] = deg;
		recalculateLinkMatrices(joint_index);
		emit configurationChanged();
	}
}

void QRobot::setPose(const Pose &pose)
{
	bool poseChanged=false;
	for (uint32_t joint=0; joint<QRobot::numOfJoints; joint++)
	{
		double newJointAngle=qBound(mJointLimitMin[joint], pose.jointAngles[joint], mJointLimitMax[joint]);
		if (mPose.jointAngles[joint] != newJointAngle)
		{
			mPose.jointAngles[joint] = newJointAngle;
			poseChanged=true;
		}
	}
	if(poseChanged)
	{
		recalculateLinkMatrices(0);
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
	}
}

double QRobot::getJointAngle(uint32_t joint_index) const
{
	if (joint_index >= numOfJoints)
	{
		joint_index=0;
	}
	return (mPose.jointAngles[joint_index]);
}

const QRobot::Pose &QRobot::getPose() const
{
	return (mPose);
}

QPair<qreal, qreal> QRobot::getJointLimits(uint32_t joint_index) const
{
	if (joint_index >= numOfJoints)
	{
		joint_index=0;
	}
	return {mJointLimitMin[joint_index], mJointLimitMax[joint_index]};
}

const QMatrix4x4 &QRobot::getLinkMatrix(uint32_t link_index) const
{
	if (link_index >= numOfJoints)
	{
		link_index=0;
	}
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
