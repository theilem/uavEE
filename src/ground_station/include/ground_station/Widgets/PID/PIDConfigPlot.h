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
#ifndef PIDCONFIGPLOT_H
#define PIDCONFIGPLOT_H

#include <QWidget>
#include "ground_station/ConfigManager.h"
#include "uavAP/FlightControl/Controller/PIDController/PIDHandling.h"
#include "ground_station/IPIDConfigurator.h"

namespace Ui
{
class PIDConfigPlot;
}

class PIDConfigPlot: public QWidget
{
Q_OBJECT
	friend class WidgetCPGrid;
public:
	explicit
	PIDConfigPlot(QWidget *parent = 0);
	PIDConfigPlot(QWidget *parent, int key, std::string name,
			const Control::PIDParameters& param);
	void
	setData(double current, double target);
	void
	sendData();
	void
	connect(std::shared_ptr<IPIDConfigurator> configManager);
	double
	getkP();
	double
	getkI();
	double
	getkD();
	double
	getFF();
	double
	getIMax();
	void
	resetGraph();
	void
	setkP(double kP);
	void
	setkI(double kI);
	void
	setkD(double kD);
	void
	setFF(double ff);
	void
	setIMax(double iMax);
	~PIDConfigPlot();

private slots:
	void
	on_send_clicked();

private:
	Ui::PIDConfigPlot *ui;
	int key_;
	std::shared_ptr<IPIDConfigurator> cm_;
	QString title;
};

#endif // PIDCONFIGPLOT_H
