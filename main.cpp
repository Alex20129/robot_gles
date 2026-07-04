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

	ControlsWidget *cWidget=new ControlsWidget;
	cWidget->attachRobot(robot);


	rvWidget->show();
	cWidget->show();

	return app.exec();
}
