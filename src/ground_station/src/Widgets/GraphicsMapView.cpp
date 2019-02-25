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

#include <string>
#include <math.h>
#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
#include <QtGui>
#include <QMenu>
#include <autopilot_interface/detail/uavAPConversions.h>
#include <uavAP/Core/LinearAlgebraProto.h>

#include "ground_station/Widgets/GraphicsMapView.h"

GraphicsMapView::GraphicsMapView(QWidget *parent) :
		QGraphicsView(parent), zoom_(16), drawWaypoints(0), drawPaths(0), drawSafetyRectangle(0)
{
	aircraftHeading = 45;
	aircraftScale = 1.0;
	aircraftLocation = center;
	viewMode_ = ViewMode::FIXED_MODE;
}

void
GraphicsMapView::connect(std::shared_ptr<IDataSignals> dataSignals,
		std::shared_ptr<MapLogic> mapLogic)
{
	auto sender = std::dynamic_pointer_cast < QObject > (dataSignals);
	if (sender)
	{
		QObject::connect(sender.get(),
				SIGNAL(onSensorData(const simulation_interface::sensor_data&)), this,
				SLOT(on_hasNewSensorData(const simulation_interface::sensor_data&)));
		QObject::connect(sender.get(), SIGNAL(onTrajectory(const Trajectory&)), this,
				SLOT(on_hasNewTrajectory(const Trajectory&)));
		QObject::connect(sender.get(), SIGNAL(onMission(const Mission&)), this,
				SLOT(on_hasNewMission(const Mission&)));
		QObject::connect(sender.get(), SIGNAL(onPathSectionChange(int)), this,
				SLOT(on_hasNewActivePath(int)));
		QObject::connect(sender.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)), this,
				SLOT(on_localFrame(const VehicleOneFrame&)));
	}
	else
		APLOG_ERROR << "GraphicsMapView: Couldn't dynamic cast signal sender!";

	mapLogic_.set(mapLogic);
}

void
GraphicsMapView::drawBackground(QPainter *painter, const QRectF &rect)
{
	//NOTE (0,0) in rect is center of widget, not top left corner
	//painter uses (0,0) as center of widget
	//NOTE (0,0) in viewport()->rect() is top left corner. Thus, viewport()->rect().right() is width and viewport()->rect().bottom() is height
	drawMap(painter, rect);
}

void
GraphicsMapView::drawForeground(QPainter *painter, const QRectF &)
{
	if (drawPaths)
	{
		drawTrajectory(painter);
		highlightActivePath(painter);
	}
	if (drawWaypoints)
	{
		drawMission(painter);
	}
	if (drawSafetyRectangle)
	{
		drawSafetyNet(painter);
	}
	drawControllerTarget(painter);
	drawPathHistory(painter);
	drawAircraft(painter);
}

void
GraphicsMapView::drawMap(QPainter *painter, const QRectF &rect)
{
	painter->setRenderHint(QPainter::HighQualityAntialiasing);
	if (viewMode_ == ViewMode::FOLLOW_MODE)
	{
		focus = aircraftLocation;    //FOLLOW DOES NOT WORK BECAUSE NEED TO CONVERT UTM TO LAT LON
	}
	else
	{
		focus = center;
	}
	// TILED MAP
	const int TILE_SIZE = 256;
	//const double d = TILE_SIZE*scale;
	const double d = TILE_SIZE;
	int x = (int) floor(focus.x(zoom_));
	int y = (int) floor(focus.y(zoom_));
	QPointF focusedTileDraw = -QPointF(d * (focus.x(zoom_) - x), d * (focus.y(zoom_) - y));
	QPointF mapDraw = focusedTileDraw;
	QPointF mapBR = focusedTileDraw;
	QPoint nwXY(x, y);
	QPoint seXY(x, y);
	while (mapDraw.x() > rect.left())
	{
		mapDraw.setX(mapDraw.x() - d);
		nwXY.setX(nwXY.x() - 1);
	}
	while (mapDraw.y() > rect.top())
	{
		mapDraw.setY(mapDraw.y() - d);
		nwXY.setY(nwXY.y() - 1);
	}
	while (mapBR.x() < rect.right())
	{
		mapBR.setX(mapBR.x() + d);
		seXY.setX(seXY.x() + 1);
	}
	while (mapBR.y() < rect.bottom())
	{
		mapBR.setY(mapBR.y() + d);
		seXY.setY(seXY.y() + 1);
	}
	QPixmap map(d * (seXY.x() - nwXY.x()), d * (seXY.y() - nwXY.y()));
	QPainter mapPainter(&map);
	mapPainter.fillRect(map.rect(), QColor(0, 0, 0));
	auto mapLogic = mapLogic_.get();
	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic unset!";
		return;
	}
	auto tileDir = mapLogic->getMapTileDirectory();
	for (int i = 0; i < seXY.x() - nwXY.x(); i++)
	{
		for (int j = 0; j < seXY.y() - nwXY.y(); j++)
		{
			QString path;
			path = path.sprintf("%s/%d/%d/%d.jpg", tileDir.c_str(), zoom_, nwXY.x() + i,
					nwXY.y() + j);
			APLOG_TRACE << "tile path is: " << path.toStdString();
			QPixmap tile; // load from tileDirectory or cache
			if (tile.load(path))
			{
				tile = tile.scaled((int) d, (int) d);
				mapPainter.drawPixmap(i * d, j * d, d, d, tile);
			}
			else
			{
				APLOG_WARN << "Missing tile for " << nwXY.x() + i << " " << nwXY.y() + j
						<< "path is: " << path.toStdString();
			}
		}
	}
	nwCorner = MapLocation::fromMapTileCoords(nwXY.x(), nwXY.y(), zoom_);
	seCorner = MapLocation::fromMapTileCoords(seXY.x(), seXY.y(), zoom_);
	xPxM = map.width() / (seCorner.easting() - nwCorner.easting());
	yPxM = map.height() / (nwCorner.northing() - seCorner.northing());
	QPixmap croppedMap = map;
	painter->drawPixmap(mapDraw, croppedMap);
}

void
GraphicsMapView::drawMission(QPainter *painter)
{
	auto mapLogic = mapLogic_.get();
	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic not set!";
		return;
	}
	painter->setPen(Qt::cyan);
	for (auto wp : mapLogic->getWaypoints())
	{
		QPointF wpLoc = LocalFrameToMapPoint(wp.location.x(), wp.location.y());
		QRectF r;
		r.setCoords(wpLoc.x() - 2, wpLoc.y() - 2, wpLoc.x() + 2, wpLoc.y() + 2);
		painter->fillRect(r, Qt::cyan);
	}
}

void
GraphicsMapView::drawTrajectory(QPainter *painter)
{
	QPen pen(QColor(255, 200, 0));
	pen.setWidth(2);
	painter->setPen(pen);
	auto mapLogic = mapLogic_.get();
	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic not set!";
		return;
	}

	LocalPlannerStatus status = mapLogic->getLocalPlannerStatus();
	Trajectory traj = mapLogic->getPath();

	if (traj.pathSections.empty())
		return;

	Vector3 lastPoint = traj.pathSections.back()->getEndPoint();

	if (traj.pathSections.size() == 0)
		return;


	if (status.has_linear_status())
	{
		if (status.linear_status().is_in_approach() && traj.approachSection)
		{
			drawPathSection(painter, traj.approachSection, lastPoint);
		}
	}

	for (auto ps : traj.pathSections)
	{
		drawPathSection(painter, ps, lastPoint);
	}
}

void
GraphicsMapView::drawPathHistory(QPainter *painter)
{
	QPointF aircraftCenter = LocalFrameToMapPoint(aircraftLocation.easting(), aircraftLocation.northing());
	QPointF first = aircraftCenter;
	QPointF second;
	QColor color = QColor(255, 255, 255);
	auto mapLogic = mapLogic_.get();
	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic not set!";
		return;
	}
	std::vector<MapLocation> flightPath = mapLogic->getPathHistory();
	for (unsigned int i = 1; i < flightPath.size(); i++)
	{
		float f = 1.0 - (float) (i - 1) / flightPath.size();
		color.setAlpha(255 * f);
		second = LocalFrameToMapPoint(flightPath[i].easting(), flightPath[i].northing());
		painter->setPen(QPen(QBrush(color), 2));
		painter->drawLine(first, second);
		first = second;
	}

}

void
GraphicsMapView::highlightActivePath(QPainter *painter) //highlights active path
{
	QPen pen(Qt::green);
	pen.setWidth(4);
	painter->setPen(pen);
	auto ml = mapLogic_.get();
	if (!ml)
	{
		APLOG_ERROR << "Map logic missing. Cannot highlight active path";
		return;
	}

	unsigned int pathIndex = 0;
	bool inApproach = false;


	LocalPlannerStatus lp = ml->getLocalPlannerStatus();
	if (lp.has_linear_status())
	{
		pathIndex =  lp.linear_status().current_path_section();
		inApproach = lp.linear_status().is_in_approach();
	}
	else if (lp.has_maneuver_status())
	{
		pathIndex =  lp.maneuver_status().current_path_section();
		inApproach = lp.maneuver_status().is_in_approach();
	}
	else
	{
		APLOG_ERROR << "Status cannot be displayed.";
		return;
	}



	Trajectory traj = ml->getPath();

	if (pathIndex >= traj.pathSections.size())
	{
		return;
	}

	std::shared_ptr<IPathSection> ps;
	if (inApproach)
		ps = traj.approachSection;
	else
		ps = traj.pathSections.at(pathIndex);

	if (ps)
	{
		Vector3 lastPoint(0,0,0);
		drawPathSection(painter, ps, lastPoint);
	}
}

void
GraphicsMapView::drawAircraft(QPainter* painter)
{
	// AIRPLANE
	QTransform aircraftTransform;
	aircraftTransform.translate(aircraftImage.size().width() / 2,
			aircraftImage.size().height() / 2);
	aircraftTransform.rotate(-(aircraftHeading - M_PI/2) * 180 / M_PI);
	aircraftTransform.scale(aircraftScale / 10.0, aircraftScale / 10.0);
	aircraftTransform.translate(aircraftImage.size().width() / -2,
			aircraftImage.size().height() / -2);
	QPixmap rotatedAircraftImage(aircraftImage.size());
	rotatedAircraftImage.fill(Qt::transparent);
	QPainter aircraftPainter;
	aircraftPainter.begin(&rotatedAircraftImage);
	aircraftPainter.setTransform(aircraftTransform);
	aircraftPainter.drawPixmap(0, 0, aircraftImage);
	aircraftPainter.end();
	QPointF aircraftCenter = LocalFrameToMapPoint(aircraftLocation.easting(), aircraftLocation.northing());
	QPointF halfAircraftSize = QPointF(rotatedAircraftImage.width() / 2,
			rotatedAircraftImage.height() / 2);
	QPointF aircraftDrawPoint = aircraftCenter - halfAircraftSize;
	painter->drawPixmap(aircraftDrawPoint, rotatedAircraftImage);
}

void
GraphicsMapView::drawPathSection(QPainter* painter, std::shared_ptr<IPathSection> ps,
		Vector3& lastPoint)
{
	if (auto curve = std::dynamic_pointer_cast < Curve > (ps))
	{
		drawCurve(painter, curve, lastPoint);
	}
	else if (auto orbit = std::dynamic_pointer_cast < Orbit > (ps))
	{
		drawOrbit(painter, orbit, lastPoint);
	}
	else if (auto spline = std::dynamic_pointer_cast < CubicSpline > (ps))
	{
		drawCubicSpline(painter, spline, lastPoint);
	}
	else if (auto line = std::dynamic_pointer_cast < Line > (ps))
	{
		drawLine(painter, line, lastPoint);
	}
	else
	{
		APLOG_WARN << "Unknown PathSection encountered in list during drawTrajectory";
	}
}

void
GraphicsMapView::drawLine(QPainter* painter, std::shared_ptr<Line> line, Vector3& lastPoint)
{
	QPointF first = LocalFrameToMapPoint(line->origin().x(), line->origin().y());
	QPointF second = LocalFrameToMapPoint(line->getEndPoint().x(), line->getEndPoint().y());

	painter->drawLine(first, second);
	lastPoint = line->getEndPoint();
}

void
GraphicsMapView::drawCurve(QPainter* painter, std::shared_ptr<Curve> curve, Vector3& lastPoint)
{
	QPointF center = LocalFrameToMapPoint(curve->getCenter().x(), curve->getCenter().y());
	QPointF toFindRadius = LocalFrameToMapPoint(curve->getCenter().x() + curve->getRadius(),
			curve->getCenter().y() + curve->getRadius());
	double rX = toFindRadius.x() - center.x();
	double rY = -toFindRadius.y() + center.y();
	QRectF rect(center.x() - rX, center.y() - rY, rX * 2, rY * 2);
	QPointF start = LocalFrameToMapPoint(lastPoint.x(), lastPoint.y());
	QPointF end = LocalFrameToMapPoint(curve->getEndPoint().x(), curve->getEndPoint().y());
	double startAngle = 16 * std::atan2(center.y() - start.y(), start.x() - center.x())
			* 180.0/ M_PI; //Qt gives all angles as a multiplier of 16
	double endAngle = 16 * std::atan2(center.y() - end.y(), end.x() - center.x()) * 180.0 / M_PI;
	double spanAngle = endAngle - startAngle;
	if (spanAngle < -16 * 180)
	{
		spanAngle += 16 * 360;
	}
	else if (spanAngle > 16 * 180)
	{
		spanAngle -= 16 * 360;
	}
	painter->drawArc(rect, startAngle, spanAngle);
	lastPoint = curve->getEndPoint();
}

void
GraphicsMapView::drawCubicSpline(QPainter* painter, std::shared_ptr<CubicSpline> spline,
		Vector3& lastPoint)
{
	const double stepSize = 0.05;
	QPointF first = LocalFrameToMapPoint(spline->c0_.x(), spline->c0_.y());

	Vector2 c0 = spline->c0_.head(2);
	Vector2 c1 = spline->c1_.head(2);
	Vector2 c2 = spline->c2_.head(2);
	Vector2 c3 = spline->c3_.head(2);

	for (double u = stepSize; u <= 1; u += stepSize)
	{
		auto p = c0 + c1 * u + c2 * pow(u, 2) + c3 * pow(u, 3);
		QPointF next = LocalFrameToMapPoint(p.x(), p.y());
		painter->drawLine(first, next);
		first = next;
	}
	lastPoint = spline->getEndPoint();
}

void
GraphicsMapView::drawOrbit(QPainter* painter, std::shared_ptr<Orbit> orbit, Vector3& lastPoint)
{
	QPointF center = LocalFrameToMapPoint(orbit->getCenter().x(), orbit->getCenter().y());
	QPointF toFindRadius = LocalFrameToMapPoint(orbit->getCenter().x() + orbit->getRadius(),
			orbit->getCenter().y() + orbit->getRadius());
	double rX = sqrt(pow(toFindRadius.x() - center.x(),2) + pow(toFindRadius.y() - center.y(),2)) / sqrt(2);
	painter->drawEllipse(center, rX, rX);
	lastPoint = orbit->getCenter();
}

void
GraphicsMapView::on_localFrame(const VehicleOneFrame& frame)
{
	localFrame_ = frame;
	int zone = center.getZone();
	char hemi = center.getHemi();
	center = MapLocation::fromVector3(localFrame_.toInertialFramePosition(Vector3(0,0,0)), zone, hemi);
}

void
GraphicsMapView::drawSafetyNet(QPainter *painter)
{
	auto mapLogic = mapLogic_.get();

	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic Missing.";
		return;
	}

	const auto& bounds = mapLogic->getSafetyBounds();

	if (!bounds.has_center())
	{
		APLOG_TRACE << "GraphicsMapView: Safety Bounds Center Missing.";
		return;
	}

	auto center = toVector(bounds.center());
	double rotation = bounds.major_side_orientation() * M_PI / 180.0;
	double majorOffset = bounds.major_side_length() / 2;
	double minorOffset = bounds.minor_side_length() / 2;

	Vector2 Q1 = rotate2Drad(Vector2(majorOffset, minorOffset), rotation) + center.head(2);
	Vector2 Q2 = rotate2Drad(Vector2(-majorOffset, minorOffset), rotation) + center.head(2);
	Vector2 Q3 = rotate2Drad(Vector2(-majorOffset, -minorOffset), rotation) + center.head(2);
	Vector2 Q4 = rotate2Drad(Vector2(majorOffset, -minorOffset), rotation) + center.head(2);

	QPointF M1 = LocalFrameToMapPoint(Q1.x(), Q1.y());
	QPointF M2 = LocalFrameToMapPoint(Q2.x(), Q2.y());
	QPointF M3 = LocalFrameToMapPoint(Q3.x(), Q3.y());
	QPointF M4 = LocalFrameToMapPoint(Q4.x(), Q4.y());

	QPen pen;
	pen.setColor(Qt::red);
	pen.setWidth(2);

	painter->setPen(pen);
	painter->drawLine(M1, M2);
	painter->drawLine(M2, M3);
	painter->drawLine(M3, M4);
	painter->drawLine(M1, M4);
}

void
GraphicsMapView::drawControllerTarget(QPainter* painter)
{
	auto mapLogic = mapLogic_.get();
	if (!mapLogic)
	{
		APLOG_ERROR << "GraphicsMapView: MapLogic not set!";
		return;
	}
	auto status = mapLogic->getLocalPlannerStatus();
	if (!status.has_linear_status())
		return;
	if (status.linear_status().has_airplane_status())
	{
		//drawing controller target

		double command = status.linear_status().airplane_status().heading_target();
		QPointF airplaneLoc = LocalFrameToMapPoint(aircraftLocation.easting(),
				aircraftLocation.northing()); //ENU
		QPointF dest = QPointF(airplaneLoc.x() + 50 * cos(-command + M_PI / 2),
				airplaneLoc.y() - 50 * sin(-command + M_PI / 2));
		QPen pen;
		pen.setColor(Qt::red);
		pen.setWidth(3);
		painter->setPen(pen);
		painter->drawLine(airplaneLoc, dest);

		//drawing position deviation
		auto pathIndex = status.linear_status().current_path_section();
		if (mapLogic->getPath().pathSections.size() <= pathIndex)
		{
			return;
		}

		std::shared_ptr<IPathSection> ps;
		if (status.linear_status().is_in_approach())
			ps = mapLogic->getPath().approachSection;
		else
			ps = mapLogic->getPath().pathSections.at(pathIndex);
		Vector3 currentPos = xyzTypeToVector3(mapLogic->getSensorData().position);
		ps->updatePosition(currentPos);

		Vector3 closestPoint = currentPos + ps->getPositionDeviation();
		QPointF mapPoint = LocalFrameToMapPoint(closestPoint[0], closestPoint[1]);
		pen.setColor(Qt::blue);
		pen.setWidth(2);
		painter->setPen(pen);
		painter->drawLine(airplaneLoc, mapPoint);
		pen.setWidth(5);
		painter->setPen(pen);
		painter->drawPoint(mapPoint);
	}
}

Vector3
GraphicsMapView::mapPointToUTM(QPointF position, double down) const
{
	QPointF diff = position - this->rect().center();
	double northing = focus.northing() - diff.y() / yPxM;
	double easting = focus.easting() + diff.x() / xPxM;
	return Vector3(northing, easting, down);
}

QPointF
GraphicsMapView::UTMToMapPoint(double e, double n) const
{
	QPointF mapCenter = QPointF((focus.easting() - nwCorner.easting()) * xPxM,
			(nwCorner.northing() - focus.northing()) * yPxM);
	float x = (e - nwCorner.easting()) * xPxM - mapCenter.x();
	float y = (nwCorner.northing() - n) * yPxM - mapCenter.y();
	return QPointF(x, y);
}

QPointF
GraphicsMapView::LocalFrameToMapPoint(double e, double n) const
{
	Vector3 local(e, n, 0);
	local = localFrame_.toInertialFramePosition(local);
	return UTMToMapPoint(local.x(), local.y());
}

void
GraphicsMapView::on_hasNewMission(const Mission&)
{
	if (drawWaypoints)
	{
		viewport()->update();
	}
}

void
GraphicsMapView::on_hasNewTrajectory(const Trajectory&)
{
	if (drawPaths)
	{
		viewport()->update();
	}
}

void
GraphicsMapView::on_hasNewActivePath(int)
{
	if (drawPaths)
	{
		viewport()->update();
	}
}

void
GraphicsMapView::on_hasNewSensorData(const simulation_interface::sensor_data &sd)
{
	aircraftLocation = MapLocation(sd.position.x, sd.position.y);
	aircraftHeading = localFrame_.toInertialFrameRotation(Vector3(0,0,sd.attitude.z)).z();
	viewport()->update();
}

void
GraphicsMapView::mouseMoveEvent(QMouseEvent* move)
{
	QPointF newCenter = lastCenterTileCoords_ - (move->localPos() - moveStart_)/256.0;
	center = MapLocation::fromMapTileCoords(newCenter.x(), newCenter.y(), zoom_);
	viewport()->update();
}

void
GraphicsMapView::mousePressEvent(QMouseEvent* event)
{
	moveStart_ = event->localPos();
	lastCenter_ = center;
	lastCenterTileCoords_ = QPointF(lastCenter_.x(zoom_), lastCenter_.y(zoom_));
}

void
GraphicsMapView::mouseReleaseEvent(QMouseEvent *event)
{
}
