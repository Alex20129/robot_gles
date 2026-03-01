#ifndef CONTROLSWIDGET_HPP
#define CONTROLSWIDGET_HPP

#include <QWidget>

namespace Ui
{
	class ControlsWidget;
}

class ControlsWidget : public QWidget
{
	Q_OBJECT
	Ui::ControlsWidget *ui;
private slots:
	void onJ0ControlValueChanged(int value);
	void onJ1ControlValueChanged(int value);
	void onJ2ControlValueChanged(int value);
	void onJ3ControlValueChanged(int value);
	void onJ4ControlValueChanged(int value);
	void onJ5ControlValueChanged(int value);
public:
	explicit ControlsWidget(QWidget *parent = nullptr);
	~ControlsWidget();
signals:
	void jointControlValueChanged(int jointIndex, double deg);
};

#endif // CONTROLSWIDGET_HPP
