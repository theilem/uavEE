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
#ifndef WIDGETOVERHEADMAP_H
#define WIDGETOVERHEADMAP_H

#include <QWidget>

//#include "DataModel.h"
#include "ground_station/MapLocation.h"
#include "ground_station/Widgets/GraphicsMapView.h"
#include <uavAP/MissionControl/GlobalPlanner/PathSections/IPathSection.h>
namespace Ui
{
class WidgetOverheadMap;
}

class IWidgetInterface;

class WidgetOverheadMap: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "overhead_map";

	explicit
	WidgetOverheadMap(QWidget* parent = 0);
	~WidgetOverheadMap();

	static QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent);

	void
	setMode(GraphicsMapView::ViewMode mode);

	void
	setMapImage(const QPixmap& map);

	void
	setNWMapCorner(const MapLocation& loc);

	void
	setSEMapCorner(const MapLocation& loc);

	void
	setMapCenter(const MapLocation& loc);

	void
	setAirplaneLocation(const MapLocation& loc);

	void
	setAirplaneHeading(double heading);

	void
	requestGraphicsUpdate();

private slots:

	void
	on_fixedModeButton_toggled(bool checked);

	void
	on_displayWaypoints_toggled(bool checked);

	void
	on_displayTrajectory_toggled(bool checked);

	void
	on_displaySafetyRectangle_toggled(bool checked);

	void
	on_requestMission_clicked();

	void
	on_requestTrajectory_clicked();

	void
	on_airplaneScaleBox_valueChanged(double arg1);

	void
	on_zoomBox_valueChanged(double arg1);

	void
	on_updateCenter_clicked();

	void
	on_requestSafetyNet_clicked();

private:

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	void
	configure(const boost::property_tree::ptree& json);

	ObjectHandle<MapLogic> mapLogic_;
	Ui::WidgetOverheadMap* ui;
};

#endif // WIDGETOVERHEADMAP_H
