#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>

#include "robot.hpp"
#include "robotviewwidget.hpp"
#include "controlswidget.hpp"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	format.setSamples(8);
	QSurfaceFormat::setDefaultFormat(format);

	app.setApplicationName("robot");

	QRobot *robot=new QRobot;
	RobotViewWidget *rvWidget=new RobotViewWidget;
	rvWidget->resize(1024, 768);
	rvWidget->attachRobot(robot);
	QObject::connect(robot, &QRobot::configurationChanged, rvWidget, &RobotViewWidget::onRobotConfigurationChanged);

	ControlsWidget *cWidget=new ControlsWidget;
	QObject::connect(cWidget, &ControlsWidget::jointControlValueChanged, robot, &QRobot::setJointAngle);
	QObject::connect(cWidget, &ControlsWidget::jointControlValueChanged, robot, &QRobot::solveForwardKinematics);

	rvWidget->show();
	cWidget->show();

	return app.exec();
}
