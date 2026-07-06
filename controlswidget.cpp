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
	if(nullptr!=this->robot)
	{
		QObject::disconnect(this->robot, nullptr, this, nullptr);
		QObject::disconnect(this, nullptr, this->robot, nullptr);
	}
	this->robot=robot;

	QObject::connect(this, &ControlsWidget::jointControlValueChanged, robot, &QRobot::setJointAngle);
	QObject::connect(this, &ControlsWidget::needToSetTargetPosition, robot, &QRobot::setTargetPosition);
	QObject::connect(this, &ControlsWidget::needToStartAnimation, robot, &QRobot::startAnimation);
	QObject::connect(robot, &QRobot::configurationChanged, this, &ControlsWidget::onRobotConfigurationChanged);

	ui->horizontalScrollBar_J0->setMinimum(this->robot->getJointLimits(0).first*controlsMultiplicator);
	ui->horizontalScrollBar_J0->setMaximum(this->robot->getJointLimits(0).second*controlsMultiplicator);
	ui->horizontalScrollBar_J1->setMinimum(this->robot->getJointLimits(1).first*controlsMultiplicator);
	ui->horizontalScrollBar_J1->setMaximum(this->robot->getJointLimits(1).second*controlsMultiplicator);
	ui->horizontalScrollBar_J2->setMinimum(this->robot->getJointLimits(2).first*controlsMultiplicator);
	ui->horizontalScrollBar_J2->setMaximum(this->robot->getJointLimits(2).second*controlsMultiplicator);
	ui->horizontalScrollBar_J3->setMinimum(this->robot->getJointLimits(3).first*controlsMultiplicator);
	ui->horizontalScrollBar_J3->setMaximum(this->robot->getJointLimits(3).second*controlsMultiplicator);
	ui->horizontalScrollBar_J4->setMinimum(this->robot->getJointLimits(4).first*controlsMultiplicator);
	ui->horizontalScrollBar_J4->setMaximum(this->robot->getJointLimits(4).second*controlsMultiplicator);
	ui->horizontalScrollBar_J5->setMinimum(this->robot->getJointLimits(5).first*controlsMultiplicator);
	ui->horizontalScrollBar_J5->setMaximum(this->robot->getJointLimits(5).second*controlsMultiplicator);

	onRobotConfigurationChanged();
}

void ControlsWidget::onRobotConfigurationChanged()
{
	QVector3D toolPosition=robot->getToolPosition();
	ui->lineEdit_current_x->setText(QString::number(toolPosition.x(), 'f', 3));
	ui->lineEdit_current_y->setText(QString::number(toolPosition.y(), 'f', 3));
	ui->lineEdit_current_z->setText(QString::number(toolPosition.z(), 'f', 3));
	ui->lineEdit_j0->setText(QString::number(robot->getJointAngle(0), 'f', 3));
	ui->lineEdit_j1->setText(QString::number(robot->getJointAngle(1), 'f', 3));
	ui->lineEdit_j2->setText(QString::number(robot->getJointAngle(2), 'f', 3));
	ui->lineEdit_j3->setText(QString::number(robot->getJointAngle(3), 'f', 3));
	ui->lineEdit_j4->setText(QString::number(robot->getJointAngle(4), 'f', 3));
	ui->lineEdit_j5->setText(QString::number(robot->getJointAngle(5), 'f', 3));
}

void ControlsWidget::onJ0ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(0, angle);
}

void ControlsWidget::onJ1ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(1, angle);
}

void ControlsWidget::onJ2ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(2, angle);
}

void ControlsWidget::onJ3ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(3, angle);
}

void ControlsWidget::onJ4ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(4, angle);
}

void ControlsWidget::onJ5ControlValueChanged(int value)
{
	double angle=value/controlsMultiplicator;
	emit jointControlValueChanged(5, angle);
}

void ControlsWidget::on_pushButton_solve_ik_clicked()
{
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
	emit needToStartAnimation();
}

void ControlsWidget::on_lineEdit_target_x_textChanged(const QString &arg1)
{
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}

void ControlsWidget::on_lineEdit_target_y_textChanged(const QString &arg1)
{
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}

void ControlsWidget::on_lineEdit_target_z_textChanged(const QString &arg1)
{
	emit needToSetTargetPosition(QVector3D(
		ui->lineEdit_target_x->text().toFloat(),
		ui->lineEdit_target_y->text().toFloat(),
		ui->lineEdit_target_z->text().toFloat()));
}
