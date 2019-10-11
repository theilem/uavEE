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

#ifndef WIDGETXPLANE_H
#define WIDGETXPLANE_H

#include <memory>
#include <mutex>
#include <QWidget>
#include <QLineEdit>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Frames/VehicleOneFrame.h>
#include <simulation_interface/sensor_data.h>

class IConfigManager;
class IWidgetInterface;

namespace Ui
{
class WidgetXPlane;
}

class WidgetXPlane: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "x_plane";

	explicit
	WidgetXPlane(QWidget* parent = 0);

	~WidgetXPlane();

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetXPlane(parent));
		widget->connectInterface(interface);
		return widget;
	}

public slots:

	void
	on_engineStartValue_clicked();

	void
	on_engineStopValue_clicked();

	void
	on_gpsFixValue_clicked();

	void
	on_autopilotValue_clicked();

	void
	on_edit_clicked();

	void
	on_cancel_clicked();

	void
	on_apply_clicked();

	void
	onFrameIndexChanged(int index);

	void
	onWindLayerIndexChanged(int index);

	void
	onXPlaneSensorData(const simulation_interface::sensor_data& sensorData);

	void
	onLocalFrame(const VehicleOneFrame& localFrame);

private:

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	void
	toInertialFrame(simulation_interface::sensor_data& sensorData);

	void
	readText(double& sensorData, const QLineEdit* lineEdit);

	void
	setText(const QString& text);

	void
	setStyle(const bool& edit);

	void
	setEdit(const bool& edit);

	void
	clear();

	Ui::WidgetXPlane* ui;

	ObjectHandle<IConfigManager> configManager_;

	QString disabledStyle_;
	QString enabledStyle_;
	QString enabledStyleButton_;

	unsigned gpsFix_;
	unsigned autopilotActive_;
	bool sensorDataActive_;

	bool lastGPSFix_;
	bool lastAutopilotActive_;
	std::mutex lastMutex_;

	bool edit_;
	std::mutex editMutex_;

	int frameIndex_;
	std::mutex frameIndexMutex_;

	int windLayerIndex_;
	std::mutex windLayerIndexMutex_;

	VehicleOneFrame localFrame_;
};

#endif /* WIDGETXPLANE_H */
