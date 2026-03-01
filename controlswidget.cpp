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

void ControlsWidget::onJ0ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(0, angle);
}

void ControlsWidget::onJ1ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(1, angle);
}

void ControlsWidget::onJ2ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(2, angle);
}

void ControlsWidget::onJ3ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(3, angle);
}

void ControlsWidget::onJ4ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(4, angle);
}

void ControlsWidget::onJ5ControlValueChanged(int value)
{
	double angle=value/4.0;
	emit jointControlValueChanged(5, angle);
}
