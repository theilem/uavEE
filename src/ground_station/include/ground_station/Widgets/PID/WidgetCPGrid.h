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
#ifndef WIDGETCPGRID_H
#define WIDGETCPGRID_H

#include <radio_comm/pidstati.h>
#include <QWidget>
#include "ground_station/ConfigManager.h"

namespace Ui
{
class WidgetCPGrid;
}

class PIDConfigPlot;
class IWidgetInterface;

class WidgetCPGrid: public QWidget
{
	Q_OBJECT

public:

	static constexpr auto widgetName = "cp_grid";

	explicit
	WidgetCPGrid(QWidget* parent = 0);
	~WidgetCPGrid();

	static QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent);



public slots:
	void
	on_sendAllParams_clicked();
	void
	onPIDStati(const radio_comm::pidstati& data);
	void
	on_saveGains_clicked();
	void
	on_loadGains_clicked();

private:
	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);
	QString title;
	Ui::WidgetCPGrid* ui;
	std::map<PIDs, std::shared_ptr<PIDConfigPlot>> plots;
};

#endif // WIDGETCPGRID_H
