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
#ifndef WIDGETPIDPLOTS_H
#define WIDGETPIDPLOTS_H


#include <QWidget>
#include "ground_station/IDataSignals.h"
#include "ground_station/IPIDConfigurator.h"

class PIDCustomPlot;
class IWidgetInterface;
namespace Ui
{
class WidgetPIDPlots;
}

class WidgetPIDPlots: public QWidget
{
    Q_OBJECT

public:

    static constexpr char* widgetName = "pid_plots";

    explicit
    WidgetPIDPlots(QWidget* parent = 0);
    ~WidgetPIDPlots();

    static inline QWidget*
    createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
    {
        auto widget(new WidgetPIDPlots(parent));
        widget->connectInterface(interface);
        return widget;
    }

private slots:

    void
    on_rowsOnly_pressed();

    void
    on_columnsOnly_pressed();

    void
    on_custom_pressed();

    void
    on_numCols_valueChanged(int arg1);

    void
    onPIDStati(const radio_comm::pidstati& data);

private:

    void
    connectInterface(std::shared_ptr<IWidgetInterface> interface);

    void
    clearGrid();

    Ui::WidgetPIDPlots* ui;
    std::map<int, std::shared_ptr<PIDCustomPlot>> plots;
};

#endif // WIDGETPIDPLOTS_H
