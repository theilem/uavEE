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
#include <ground_station/IWidgetInterface.h>
#include "ground_station/Widgets/WidgetOverheadMap.h"
#include "ui_WidgetOverheadMap.h"
#include <QJsonDocument>
#include <QtOpenGL/QGLWidget>
#include "ground_station/ConfigManager.h"
#include "ground_station/Widgets/GraphicsMapView.h"
#include "ground_station/MapLogic.h"
//#include "QMouseEvent"
WidgetOverheadMap::WidgetOverheadMap(QWidget* parent) :
    QWidget(parent), ui(new Ui::WidgetOverheadMap)
{
    ui->setupUi(this);
    ui->airplaneScaleLabel->setStyleSheet("color:white;");
    ui->zoomLabel->setStyleSheet("color:white;");
    ui->mapView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    ui->mapView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    ui->mapView->setScene(new QGraphicsScene());

    //ui->mapView->antenna = MapLocation(); Maybe reimplement
    ui->mapView->center = MapLocation();
    ui->mapView->nwCorner = MapLocation();
    ui->mapView->seCorner = MapLocation();
}

WidgetOverheadMap::~WidgetOverheadMap()
{
    delete ui;
}

void
WidgetOverheadMap::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
    if (!interface)
    {
        APLOG_ERROR << "WidgetOverheadMap cannot connect to interface";
        return;
    }
    if (!interface->getIDataSignals().isSet())
    {
        APLOG_ERROR << "WidgetOverheadMap cannot get IDataSignals";
        return;
    }
    if (!interface->getMapLogic().isSet())
    {
        APLOG_ERROR << "WidgetOverheadMap cannot get MapLogic";
        return;
    }
    ui->mapView->connect(interface->getIDataSignals().get(), interface->getMapLogic().get());
    mapLogic_ = interface->getMapLogic();
    if (!interface->getConfigManager().isSet())
    {
        APLOG_ERROR << "WidgetOverheadMap cannot get ConfigManager";
        return;
    }
    configure(interface->getConfigManager().get()->getWidgetConfigByName(widgetName));
}

void
WidgetOverheadMap::configure(const boost::property_tree::ptree& json)
{
    /*if (json.contains("airplaneImagePath"))
     {
     QPixmap temp;
     temp.load(json["airplaneImagePath"].toString());
     ui->mapView->aircraftImage = temp;
     }*/
    auto mapLogic = mapLogic_.get();
    if (!mapLogic)
    {
        APLOG_ERROR << "MapLogic missing in overheadmap";
        return;
    }
    PropertyMapper pm(json);
    std::string aircraftImagePath;
    pm.add("aircraft_image", aircraftImagePath, true);
    QPixmap temp;
    aircraftImagePath = mapLogic->getIconPath() + aircraftImagePath;
    temp.load(QString::fromStdString(aircraftImagePath));
    ui->mapView->aircraftImage = temp;
    /*if (json.contains("mapCenter"))
     {
     ui->mapView->center = MapLocation::fromJson(json["mapCenter"].toObject());
     ui->CenterX->setText(QString::number(ui->mapView->center.latitude()));
     ui->CenterY->setText(QString::number(ui->mapView->center.longitude()));
     APLOG_TRACE << "new config center " << ui->mapView->center.latitude() << " "
     << ui->mapView->center.longitude();
     }*/
    boost::property_tree::ptree center;
    pm.add("map_center", center, true);
    ui->mapView->center = MapLocation::fromJson(center);
    ui->CenterX->setText(QString::number(ui->mapView->center.latitude()));
    ui->CenterY->setText(QString::number(ui->mapView->center.longitude()));
    /*if (json.contains("map_nw_corner"))
     {
     ui->mapView->nwCorner = MapLocation::fromJson(json["mapNWCorner"].toObject());
     APLOG_TRACE << "new NWCorner center " << ui->mapView->nwCorner.latitude() << " "
     << ui->mapView->nwCorner.longitude();
     }*/
    boost::property_tree::ptree nw_corner;
    pm.add("map_nw_corner", nw_corner, true);
    ui->mapView->nwCorner = MapLocation::fromJson(nw_corner);

    /*if (json.contains("map_se_corner"))
     {
     ui->mapView->seCorner = MapLocation::fromJson(json["mapSECorner"].toObject());
     APLOG_TRACE << "new SECorner center " << ui->mapView->seCorner.latitude() << " "
     << ui->mapView->seCorner.longitude();
     }*/

    boost::property_tree::ptree se_corner;
    pm.add("map_se_corner", se_corner, true);
    ui->mapView->seCorner = MapLocation::fromJson(se_corner);
}

void
WidgetOverheadMap::setMapImage(const QPixmap& map)
{
    ui->mapView->mapImage = map;
}

void
WidgetOverheadMap::setNWMapCorner(const MapLocation& loc)
{
    ui->mapView->nwCorner = loc;
}

void
WidgetOverheadMap::setSEMapCorner(const MapLocation& loc)
{
    ui->mapView->seCorner = loc;
}

void
WidgetOverheadMap::setMapCenter(const MapLocation& loc)
{
    ui->mapView->center = loc;
}

void
WidgetOverheadMap::setAirplaneHeading(double heading)
{
    ui->mapView->aircraftHeading = heading;
}

void
WidgetOverheadMap::setMode(GraphicsMapView::ViewMode mode)
{
    ui->mapView->viewMode_ = mode;
}

void
WidgetOverheadMap::requestGraphicsUpdate()
{
    this->update();
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_fixedModeButton_toggled(bool checked)
{
    ui->mapView->viewMode_ = (
                                 (checked) ?
                                 GraphicsMapView::ViewMode::FIXED_MODE : GraphicsMapView::ViewMode::FOLLOW_MODE);
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_displayWaypoints_toggled(bool checked)
{
    ui->mapView->drawWaypoints = checked;
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_displayTrajectory_toggled(bool checked)
{
    ui->mapView->drawPaths = checked;
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_displaySafetyNet_toggled(bool checked)
{
    ui->mapView->drawSafetyRectangle = checked;
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_requestMission_clicked()
{
    if (mapLogic_.isSet())
        mapLogic_.get()->askForMission();
}

void
WidgetOverheadMap::on_requestTrajectory_clicked()
{
    if (mapLogic_.isSet())
        mapLogic_.get()->askForTrajectory();
}

void
WidgetOverheadMap::on_airplaneScaleBox_valueChanged(double arg1)
{
    ui->mapView->aircraftScale = arg1;
    QPainter a(ui->mapView->viewport());
    ui->mapView->drawAircraft(&a);
}

void
WidgetOverheadMap::on_zoomBox_valueChanged(double arg1)
{
    ui->mapView->zoom = (int) arg1;
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_updateCenter_clicked()
{
    ui->mapView->center = MapLocation::fromLatLong(ui->CenterX->text().toDouble(),
                          ui->CenterY->text().toDouble());
    ui->mapView->viewport()->update();
}

void
WidgetOverheadMap::on_requestSafetyNet_clicked()
{
    if (mapLogic_.isSet())
        mapLogic_.get()->askForSafetyNet();
}
