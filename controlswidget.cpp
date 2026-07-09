#include "controlswidget.hpp"
#include "ui_controlswidget.h"

ControlsWidget::ControlsWidget(QWidget *parent) : QWidget(parent)
{
	ui=new Ui::ControlsWidget;
	ui->setupUi(this);
	connect(ui->horizontalScrollBar_J0, &QScrollBar::valueChanged, this, &ControlsWidget::onJ0ControlValueChanged);
	connect(ui->horizontalScrollBar_J1, &QScrollBar::valueChanged, this, &ControlsWidget::onJ1ControlValueChanged);
	connect(ui->horizontalScrollBar_J2, &QScrollBar::valueChanged, this, &ControlsWidget::onJ2ControlValueChanged);
	connect(ui->horizontalScrollBar_J3, &QScrollBar::valueChanged, this, &ControlsWidget::onJ3ControlValueChanged);
	connect(ui->horizontalScrollBar_J4, &QScrollBar::valueChanged, this, &ControlsWidget::onJ4ControlValueChanged);
	connect(ui->horizontalScrollBar_J5, &QScrollBar::valueChanged, this, &ControlsWidget::onJ5ControlValueChanged);
}

ControlsWidget::~ControlsWidget()
{
	delete ui;
}

void ControlsWidget::attachRobot(QRobot *robot)
{
	if(nullptr==robot)
	{
		return;
	}
	if(nullptr!=this->mRobot)
	{
		QObject::disconnect(this->mRobot, nullptr, this, nullptr);
		QObject::disconnect(this, nullptr, this->mRobot, nullptr);
	}
	mRobot=robot;
	QObject::connect(this, &ControlsWidget::jointControlValueChanged, mRobot, &QRobot::setJointAngle);
	QObject::connect(this, &ControlsWidget::needToSetTargetPosition, mRobot, &QRobot::setTargetPosition);
	QObject::connect(this, &ControlsWidget::needToStartAnimation, mRobot, &QRobot::startAnimation);
	QObject::connect(mRobot, &QRobot::configurationChanged, this, &ControlsWidget::onRobotConfigurationChanged);
	ui->horizontalScrollBar_J0->setMinimum(mRobot->getJointLimits(0).first*controlsMultiplicator);
	ui->horizontalScrollBar_J0->setMaximum(mRobot->getJointLimits(0).second*controlsMultiplicator);
	ui->horizontalScrollBar_J1->setMinimum(mRobot->getJointLimits(1).first*controlsMultiplicator);
	ui->horizontalScrollBar_J1->setMaximum(mRobot->getJointLimits(1).second*controlsMultiplicator);
	ui->horizontalScrollBar_J2->setMinimum(mRobot->getJointLimits(2).first*controlsMultiplicator);
	ui->horizontalScrollBar_J2->setMaximum(mRobot->getJointLimits(2).second*controlsMultiplicator);
	ui->horizontalScrollBar_J3->setMinimum(mRobot->getJointLimits(3).first*controlsMultiplicator);
	ui->horizontalScrollBar_J3->setMaximum(mRobot->getJointLimits(3).second*controlsMultiplicator);
	ui->horizontalScrollBar_J4->setMinimum(mRobot->getJointLimits(4).first*controlsMultiplicator);
	ui->horizontalScrollBar_J4->setMaximum(mRobot->getJointLimits(4).second*controlsMultiplicator);
	ui->horizontalScrollBar_J5->setMinimum(mRobot->getJointLimits(5).first*controlsMultiplicator);
	ui->horizontalScrollBar_J5->setMaximum(mRobot->getJointLimits(5).second*controlsMultiplicator);
	onRobotConfigurationChanged();
}

void ControlsWidget::onRobotConfigurationChanged()
{
	mMuteControls=true;

	double jointAngle;

	QVector3D toolPosition=mRobot->getToolPosition();
	ui->lineEdit_current_x->setText(QString::number(toolPosition.x(), 'f', 3));
	ui->lineEdit_current_y->setText(QString::number(toolPosition.y(), 'f', 3));
	ui->lineEdit_current_z->setText(QString::number(toolPosition.z(), 'f', 3));

	jointAngle=mRobot->getJointAngle(0);
	ui->lineEdit_j0->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J0->setSliderPosition(jointAngle*controlsMultiplicator);

	jointAngle=mRobot->getJointAngle(1);
	ui->lineEdit_j1->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J1->setSliderPosition(jointAngle*controlsMultiplicator);

	jointAngle=mRobot->getJointAngle(2);
	ui->lineEdit_j2->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J2->setSliderPosition(jointAngle*controlsMultiplicator);

	jointAngle=mRobot->getJointAngle(3);
	ui->lineEdit_j3->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J3->setSliderPosition(jointAngle*controlsMultiplicator);

	jointAngle=mRobot->getJointAngle(4);
	ui->lineEdit_j4->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J4->setSliderPosition(jointAngle*controlsMultiplicator);

	jointAngle=mRobot->getJointAngle(5);
	ui->lineEdit_j5->setText(QString::number(jointAngle, 'f', 3));
	ui->horizontalScrollBar_J5->setSliderPosition(jointAngle*controlsMultiplicator);

	mMuteControls=false;
}

void ControlsWidget::onJ0ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(0, angle);
}

void ControlsWidget::onJ1ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(1, angle);
}

void ControlsWidget::onJ2ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(2, angle);
}

void ControlsWidget::onJ3ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(3, angle);
}

void ControlsWidget::onJ4ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(4, angle);
}

void ControlsWidget::onJ5ControlValueChanged(int value)
{
	if(mMuteControls)
	{
		return;
	}
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(5, angle);
}

void ControlsWidget::on_pushButton_start_animation_clicked()
{
	if(mMuteControls)
	{
		return;
	}
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
	emit needToStartAnimation();
}

void ControlsWidget::on_lineEdit_target_x_textChanged(const QString &arg1)
{
	if(mMuteControls)
	{
		return;
	}
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}

void ControlsWidget::on_lineEdit_target_y_textChanged(const QString &arg1)
{
	if(mMuteControls)
	{
		return;
	}
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}

void ControlsWidget::on_lineEdit_target_z_textChanged(const QString &arg1)
{
	if(mMuteControls)
	{
		return;
	}
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}
