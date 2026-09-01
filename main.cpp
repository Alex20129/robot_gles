#include <QApplication>
#include <QLabel>
#include <QSurfaceFormat>

#include "trajectoryplanner.hpp"
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
	QTrajectoryPlanner *trajectoryPlanner=new QTrajectoryPlanner;
	trajectoryPlanner->attachRobot(robot);

	RobotViewWidget *rvWidget=new RobotViewWidget;

	rvWidget->attachRobot(robot);
	rvWidget->attachTrajectoryPlanner(trajectoryPlanner);

	ControlsWidget *cWidget=new ControlsWidget;
	cWidget->attachRobot(robot);

	QObject::connect(cWidget, &ControlsWidget::needToSetAnimationSpeed, trajectoryPlanner, &QTrajectoryPlanner::setAnimationSpeed);
	QObject::connect(cWidget, &ControlsWidget::needToStartAnimation, trajectoryPlanner, &QTrajectoryPlanner::startAnimation);
	QObject::connect(cWidget, &ControlsWidget::needToStopAnimation, trajectoryPlanner, &QTrajectoryPlanner::stopAnimation);

	rvWidget->resize(1024, 768);
	rvWidget->show();
	cWidget->show();

	trajectoryPlanner->loadFromJsonFile("demo-path.json");
	trajectoryPlanner->rebuildPoses();

	return app.exec();
}
