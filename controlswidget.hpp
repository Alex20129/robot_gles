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
	QRobot *mRobot=nullptr;
	QVector3D mTarget;
	float mTargetPositionX=0.0;
	float mTargetPositionY=0.0;
	float mTargetPositionZ=0.0;
	float mTargetOrientationPitch=0.0;
	float mTargetOrientationYaw=0.0;
	float mTargetOrientationRoll=0.0;
	bool mMuteControls=false;
	static constexpr double controlsMultiplicator=8.0;

private slots:
	void onRobotConfigurationChanged();
	void onJ0ControlValueChanged(int value);
	void onJ1ControlValueChanged(int value);
	void onJ2ControlValueChanged(int value);
	void onJ3ControlValueChanged(int value);
	void onJ4ControlValueChanged(int value);
	void onJ5ControlValueChanged(int value);
	void on_pushButton_start_animation_clicked();
	void on_lineEdit_target_x_textChanged(const QString &arg1);
	void on_lineEdit_target_y_textChanged(const QString &arg1);
	void on_lineEdit_target_z_textChanged(const QString &arg1);
	void on_lineEdit_target_pitch_textEdited(const QString &arg1);
	void on_lineEdit_target_yaw_textEdited(const QString &arg1);
	void on_lineEdit_target_roll_textEdited(const QString &arg1);

public:
	explicit ControlsWidget(QWidget *parent = nullptr);
	~ControlsWidget();
	void attachRobot(QRobot *robot);

signals:
	void jointControlValueChanged(int jointIndex, double deg);
	void needToSetTargetPosition(float x, float y, float z);
	void needToSetTargetOrientation(float pitch, float yaw, float roll);
	void needToStartAnimation();
};

#endif // CONTROLSWIDGET_HPP
