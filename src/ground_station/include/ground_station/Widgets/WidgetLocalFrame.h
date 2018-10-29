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

#ifndef WIDGETLOCALFRAME_H
#define WIDGETLOCALFRAME_H

#include <mutex>
#include <QWidget>
#include <uavAP/Core/Frames/VehicleOneFrame.h>

#include "ui_WidgetLocalFrame.h"
#include "ground_station/IWidgetInterface.h"
#include "ground_station/ConfigManager.h"
#include "ground_station/MapLogic.h"

namespace Ui
{
class WidgetLocalFrame;
}

class IWidgetInterface;

class WidgetLocalFrame: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "local_frame";

	explicit
	WidgetLocalFrame(QWidget* parent = 0);

	~WidgetLocalFrame();

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetLocalFrame(parent));
		widget->connectInterface(interface);
		return widget;
	}

private slots:

	void
	on_reset_clicked();

	void
	on_copy_clicked();

	void
	on_send_clicked();

	void
	onLocalFrame(const VehicleOneFrame& frame);

private:

	void
	copyLocalFrame(const VehicleOneFrame& frame);

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	bool init_;
	VehicleOneFrame frame_;
	VehicleOneFrame frameOriginal_;
	mutable std::mutex frameMutex_;
	ObjectHandle<IConfigManager> configManager_;
	ObjectHandle<MapLogic> mapLogic_;
	Ui::WidgetLocalFrame* ui;
};

#endif // WIDGETLOCALFRAME_H
