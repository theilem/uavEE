////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#include "ground_station/Widgets/PID/PIDConfigPlot.h"
#include "ui_PIDConfigPlot.h"

PIDConfigPlot::PIDConfigPlot(QWidget *parent, int key, std::string name,
		const Control::PID::Parameters &param) :
		QWidget(parent), ui(new Ui::PIDConfigPlot), key_(key)
{
	ui->setupUi(this);
	//ui->customPlot->setTitle(QString::fromStdString(name));
	ui->customPlot->setTitle(name);
	//ui->customPlot->setTname
	title = QString::fromStdString(name);
	// Set title and PID params
	ui->kP->setText(QString::number(param.kp));
	ui->kP->setStyleSheet(QString("border: 1px solid gray; width: 60px; height:25px;"));
	ui->kI->setText(QString::number(param.ki));
	ui->kI->setStyleSheet(QString("border: 1px solid gray; width: 60px; height:25px;"));
	ui->kD->setText(QString::number(param.kd));
	ui->kD->setStyleSheet(QString("border: 1px solid gray; width: 60px; height:25px;"));
	ui->IMax->setText(QString::number(param.imax));
	ui->IMax->setStyleSheet(QString("border: 1px solid gray; width: 60px; height:25px;"));
	ui->FF->setText(QString::number(param.ff));
	ui->FF->setStyleSheet(QString("border: 1px solid gray; width: 60px; height:25px;"));
}

PIDConfigPlot::~PIDConfigPlot()
{
	delete ui;
}

void
PIDConfigPlot::setData(double current, double target)
{
	ui->customPlot->addData(current, target);
}

void
PIDConfigPlot::sendData()
{
	on_send_clicked();
}

void
PIDConfigPlot::connect(std::shared_ptr<IPIDConfigurator> configManager)
{
	cm_ = configManager;
}

double
PIDConfigPlot::getkP()
{
	return ui->kP->text().toDouble();
}

double
PIDConfigPlot::getkI()
{
	return ui->kI->text().toDouble();
}

double
PIDConfigPlot::getkD()
{
	return ui->kD->text().toDouble();;
}

double
PIDConfigPlot::getFF()
{
	return ui->FF->text().toDouble();
}

double
PIDConfigPlot::getIMax()
{
	return ui->IMax->text().toDouble();
}

void
PIDConfigPlot::resetGraph()
{
	ui->customPlot->resetGraph();
}

void
PIDConfigPlot::setkP(double kP)
{
	ui->kP->setText(QString::number(kP));
}

void
PIDConfigPlot::setkI(double kI)
{
	ui->kI->setText(QString::number(kI));
}

void
PIDConfigPlot::setkD(double kD)
{
	ui->kD->setText(QString::number(kD));
}

void
PIDConfigPlot::setFF(double ff)
{
	ui->FF->setText(QString::number(ff));
}

void
PIDConfigPlot::setIMax(double iMax)
{
	ui->IMax->setText(QString::number(iMax));
}

void
PIDConfigPlot::on_send_clicked()
{
	Control::PID::Parameters a;
	a.kp = ui->kP->text().toDouble();
	a.ki = ui->kI->text().toDouble();
	a.kd = ui->kD->text().toDouble();
	a.ff = ui->FF->text().toDouble();
	a.imax = ui->IMax->text().toDouble();
	PIDTuning tune;
	tune.pid = key_;
	tune.params = a;
	cm_->tunePID(tune);
}

