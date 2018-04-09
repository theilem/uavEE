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
#ifndef WIDGETSENSORDATA_H
#define WIDGETSENSORDATA_H

#include <simulation_interface/sensor_data.h>
#include <memory>
#include <QWidget>

class IWidgetInterface;

namespace Ui
{
class WidgetSensorData;
}

class WidgetSensorData: public QWidget
{
    Q_OBJECT

public:

    static constexpr char* widgetName = "sensor_data";

    explicit
    WidgetSensorData(QWidget* parent = 0);

    ~WidgetSensorData();

    static inline QWidget*
    createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
    {
        auto widget(new WidgetSensorData(parent));
        widget->connectInterface(interface);
        return widget;
    }

public slots:

    void
    onSensorData(const simulation_interface::sensor_data& sd);

private:
    void
    connectInterface(std::shared_ptr<IWidgetInterface> interface);
    Ui::WidgetSensorData* ui;
};

#endif // WIDGETSENSORDATA_H
