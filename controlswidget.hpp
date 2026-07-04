#ifndef CONTROLSWIDGET_HPP
#define CONTROLSWIDGET_HPP

#include <QWidget>
#include <QVector3D>
#include "robot.hpp"

namespace Ui
{
	class ControlsWidget;
}

class ControlsWidget : public QWidget
{
	Q_OBJECT
	Ui::ControlsWidget *ui;
	QRobot *robot=nullptr;

private slots:
	void onRobotConfigurationChanged();
	void onJ0ControlValueChanged(int value);
	void onJ1ControlValueChanged(int value);
	void onJ2ControlValueChanged(int value);
	void onJ3ControlValueChanged(int value);
	void onJ4ControlValueChanged(int value);
	void onJ5ControlValueChanged(int value);
	void on_pushButton_solve_ik_clicked();
	void on_lineEdit_target_x_textChanged(const QString &arg1);
	void on_lineEdit_target_y_textChanged(const QString &arg1);
	void on_lineEdit_target_z_textChanged(const QString &arg1);

public:
	explicit ControlsWidget(QWidget *parent = nullptr);
	~ControlsWidget();
	void attachRobot(QRobot *robot);

signals:
	void jointControlValueChanged(int jointIndex, double deg);
	void needToSetTargetPosition(QVector3D target_position);
	void needToStartAnimation();
};

#endif // CONTROLSWIDGET_HPP
