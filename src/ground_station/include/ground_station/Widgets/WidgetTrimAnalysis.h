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

#ifndef WIDGETTRIMANALYSIS_H
#define WIDGETTRIMANALYSIS_H

#include <QWidget>

#include "ui_WidgetTrimAnalysis.h"
#include "ground_station/IWidgetInterface.h"
#include "ground_station/ConfigManager.h"

namespace Ui
{
class WidgetTrimAnalysis;
}

class ControllerOutput;

class WidgetTrimAnalysis: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "trim_analysis";

	explicit
	WidgetTrimAnalysis(QWidget *parent = 0);

	~WidgetTrimAnalysis();

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetTrimAnalysis(parent));
		widget->connectInterface(interface);
		return widget;
	}

public slots:

	void
	onControllerOutputTrim(const ControllerOutput& trim);

private slots:

	void
	on_sendOffset_clicked();

private:

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	ObjectHandle<IConfigManager> configManager_;
	Ui::WidgetTrimAnalysis *ui;
};

#endif /* WIDGETTRIMANALYSIS_H */
