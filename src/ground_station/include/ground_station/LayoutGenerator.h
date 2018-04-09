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
#ifndef LAYOUTGENERATOR_H
#define LAYOUTGENERATOR_H

#include <QMainWindow>
#include <QObject>

#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <boost/property_tree/ptree.hpp>
#include <ground_station/GSWidgetFactory.h>
#include <memory>
#include "ground_station/IWidgetInterface.h"

class PanTiltHandler;
class WidgetLoader;
class DataManager;
class ConfigManager;
class MapLogic;

/**
 * @brief   The LayoutGenerator class handles all the layouts and widgets in the
 *          ground station
 */
class LayoutGenerator: public QObject,
    public IAggregatableObject,
    public IRunnableObject,
    public IWidgetInterface,
    public std::enable_shared_from_this<LayoutGenerator>
{
    Q_OBJECT
public:

    LayoutGenerator() = default;

    ~LayoutGenerator();

    /**
     * @brief   create creates a LayoutGenerator. It ignores the passed in json
     *          as it does not need a config
     * @return  a shared pointer to the newly created layout generator
     */
    static std::shared_ptr<LayoutGenerator>
    create(const boost::property_tree::ptree&);

    void
    notifyAggregationOnUpdate(Aggregator& agg) override;

    bool
    run(RunStage stage) override;

    ObjectHandle<IDataSignals>
    getIDataSignals() const override;

    ObjectHandle<MapLogic>
    getMapLogic() const override;

    ObjectHandle<IConfigManager>
    getConfigManager() const override;

    ObjectHandle<IPIDConfigurator>
    getPIDConfigurator() const override;

private:

    /**
     * @brief   addMenus adds top menu to the specified window
     * @param   win pointer to menu to add menu too
     */
    void
    addMenus(QMainWindow* win);

    /**
     * @brief changeLayout replaces a WidgetLoader with specified layout
     * @param wid pointer to WidgetLoader
     * @param widget layout name to load
     * @param rows number of rows or tabs
     * @param cols number of columns
     */
    void
    changeLayout(WidgetLoader* wid, const QString& type, int rows, int cols);

    /**
     * @brief   changeWidget replaces a WidgetLoader with specified widget
     * @param   wid pointer to WidgetLoader
     * @param   widget name of widget to load
     */
    void
    changeWidget(WidgetLoader* wid, const QString& widget);

    /**
     * @brief   createLayout recursively parses a json defining the format of the
     *          ground station. Returns a widget or format at each recursive call
     * @param   json is the boost property representation of the json configuration
     *          for the ground station
     * @param   parent is the parent widget for the returned widget to be associated
     *          with
     * @return  QWidget pointer that represents the entire passed in json
     */
    QWidget*
    createLayout(const boost::property_tree::ptree& json, QWidget* parent);

    /**
     * @brief   createWidget is a wrapper function for the GSWidgetFactory
     * @param   type name of the widget to create
     * @param   parent parent for the widget to be associated to
     * @return  pointer to created widget or blank widget if type invalid
     */
    QWidget*
    createWidget(const std::string& type, QWidget* parent);

    /**
     * @brief makeScrollableWin creates a QMainWindow and makes it scrollable
     * @param win pointer to QWidget to make center of window
     */
    void
    makeScrollableWin(QWidget* win);

    ///! vector of all windows currently displaying widgets
    std::vector<QMainWindow*> windows_;

    ///! factory used to create new widgets
    GSWidgetFactory widgetFactory_;

    ///! reference to ConfigManager
    ObjectHandle<IConfigManager> configManager_;

    ///! reference to ConfigManager
    ObjectHandle<IPIDConfigurator> pidConfigurator_;

    ///! reference to mapLogic
    ObjectHandle<MapLogic> mapLogic_;

    ///!reference to data signals
    ObjectHandle<IDataSignals> dataSignals_;

private slots:
    /**
     * @brief   handleQuit is called when user clicks Quit from drop down menu
     */
    void
    handleQuit();

    /**
     * @brief   addWin is called when user clicks Add Window from Config from drop
     *          down menu
     */
    void
    addWin();

    /**
     * @brief   addWidget is called when user clicks Add Widget from drop down menu
     */
    void
    addWidget();

    /**
     * @brief   addCustomWin is called when user clicks Create Custom Window from
     *          drop down menu
     */
    void
    addCustomWin();
};

#endif // LAYOUTGENERATOR_H
