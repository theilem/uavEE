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
#ifndef GRAPHICSMAPVIEW_H
#define GRAPHICSMAPVIEW_H

#include <QGraphicsView>
#include <QImage>
#include <QPair>

#include "ground_station/DataManager.h"
#include "ground_station/MapLogic.h"
#include "ground_station/MapLocation.h"
#include <uavAP/MissionControl/MissionPlanner/Mission.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/Curve.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/Line.h>
#include <uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h>

class GraphicsMapView: public QGraphicsView
{
Q_OBJECT
	friend class WidgetOverheadMap;
public:

	enum class ViewMode
	{
		FIXED_MODE, FOLLOW_MODE
	};

	explicit
	GraphicsMapView(QWidget *parent = 0);

	Vector3
	mapPointToUTM(QPointF position, double down) const;
	QPointF
	UTMToMapPoint(double e, double n) const;
	void
	connect(std::shared_ptr<IDataSignals> dataSignals, std::shared_ptr<MapLogic> mapLogic);

protected:

    /**
     * @brief drawBackground
     * @param painter
     * @param rect
     */
	void
	drawBackground(QPainter * painter, const QRectF &rect) override;
	void
	drawForeground(QPainter * painter, const QRectF &) override;
	void
	mouseReleaseEvent(QMouseEvent *event) override;

private slots:

	void
	on_hasNewSensorData(const simulation_interface::sensor_data& sd);
	void
	on_hasNewMission(const Mission&);
	void
	on_hasNewTrajectory(const Trajectory&);
	void
	on_hasNewActivePath(int);

private:

	void
	drawMap(QPainter *painter, const QRectF &rect);
	void
	drawTrajectory(QPainter *painter);
	Vector3
	drawLine(QPainter *painter, Line &line, Vector3 *startPoint);
	Vector3
	drawCurve(QPainter *painter, Curve &curve, Vector3 *startPoint);
	Vector3
	drawCubicSpline(QPainter *painter, CubicSpline& spline, Vector3 *startPoint);
	void
	drawOrbit(QPainter *painter, const Orbit &orbit);
	void
	drawMission(QPainter *painter);
	void
	drawPathHistory(QPainter *painter);
	void
	drawControllerTarget(QPainter* painter);
	void
	highlightActivePath(QPainter *painter);
	void
	drawAircraft(QPainter *painter);
//	void
//	drawAntenna(QPainter *painter);
	void
	drawSafetyNet(QPainter *painter);

	MapLocation nwCorner;
	MapLocation seCorner;
	MapLocation center;
	MapLocation focus;
	int zoom;
	float aircraftScale;
	QPixmap mapImage;
	QPixmap aircraftImage;
	double aircraftHeading;
	double xPxM, yPxM;
	ViewMode viewMode_;
	ObjectHandle<MapLogic> mapLogic_;
	bool drawWaypoints;
	bool drawPaths;
	bool drawSafetyRectangle;
	float savedScale;
	QPixmap savedScaledMapImage;
	MapLocation aircraftLocation;
	IPathSection * currentSection;
};

#endif // GRAPHICSMAPVIEW_H
