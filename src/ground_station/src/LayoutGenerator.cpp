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
#include <boost/property_tree/json_parser.hpp>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include "ground_station/LayoutGenerator.h"
#include <QApplication>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMenuBar>
#include <QTabWidget>
#include <QFileDialog>
#include <QInputDialog>
#include "ground_station/Widgets/WidgetOverheadMap.h"
#include "ground_station/Widgets/WidgetSensorData.h"
#include "ground_station/Widgets/PID/WidgetPIDPlots.h"
#include "ground_station/Widgets/WidgetLoader.h"
#include "ground_station/Widgets/PID/WidgetCPGrid.h"
#include "ground_station/Widgets/QFlightInstruments/WidgetSix.h"
#include "ground_station/Widgets/QFlightInstruments/WidgetPFD.h"
#include "ground_station/Widgets/WidgetManeuverPlanner.h"
#include "ground_station/ConfigManager.h"
#include "ground_station/DataManager.h"
#include "ground_station/MapLogic.h"

LayoutGenerator::~LayoutGenerator()
{
	handleQuit();
}

std::shared_ptr<LayoutGenerator>
LayoutGenerator::create(const boost::property_tree::ptree&)
{
	return std::make_shared<LayoutGenerator>();
}

void
LayoutGenerator::notifyAggregationOnUpdate(const Aggregator& agg)
{
	configManager_.setFromAggregationIfNotSet(agg);
	pidConfigurator_.setFromAggregationIfNotSet(agg);
	mapLogic_.setFromAggregationIfNotSet(agg);
	dataSignals_.setFromAggregationIfNotSet(agg);
	dataHandling_.setFromAggregationIfNotSet(agg);
}

bool
LayoutGenerator::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!configManager_.isSet())
		{
			APLOG_ERROR << "LayoutGenerator: ConfigManager missing";
			return true;
		}
		if (!pidConfigurator_.isSet())
		{
			APLOG_ERROR << "LayoutGenerator: PIDConfigurator missing";
			return true;
		}
		if (!mapLogic_.isSet())
		{
			APLOG_ERROR << "LayoutGenerator: MapLogic missing";
			return true;
		}
		if (!dataSignals_.isSet())
		{
			APLOG_ERROR << "LayoutGenerator: DataSignals missing";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto configMan = configManager_.get();
		if (!configMan)
		{
			APLOG_ERROR << "Config manager missing. Cannot create main layout.";
			return true;
		}

		makeScrollableWin(createLayout(configMan->getGSConfig(), NULL));
		break;
	}
	case RunStage::FINAL:
	{
		for (auto it : windows_)
		{
			if (!it)
				continue;
			it->show();
		}
		break;
	}
	default:
		break;
	}
	return false;
}

ObjectHandle<IDataSignals>
LayoutGenerator::getIDataSignals() const
{
	return dataSignals_;
}

ObjectHandle<MapLogic>
LayoutGenerator::getMapLogic() const
{
	return mapLogic_;
}

ObjectHandle<IConfigManager>
LayoutGenerator::getConfigManager() const
{
	return configManager_;
}

ObjectHandle<IPIDConfigurator>
LayoutGenerator::getPIDConfigurator() const
{
	return pidConfigurator_;
}

ObjectHandle<DataHandling>
LayoutGenerator::getDataHandling() const
{
	return dataHandling_;
}

void
LayoutGenerator::addMenus(QMainWindow* win)
{
	//it might be a problem that we are creating the actions with null parents,
	//however, when I try linking them to win, it sigabrts on exit
	QMenu* fileMenu = win->menuBar()->addMenu("File");
	QAction* quitAct = new QAction("Quit", nullptr);
	quitAct->setShortcut(QKeySequence::Quit);
	QObject::connect(quitAct, SIGNAL(triggered()), this, SLOT(handleQuit())); //TODO std::bind
	QAction* addWindow = new QAction("Add Window from Config", nullptr);
	QObject::connect(addWindow, SIGNAL(triggered()), this, SLOT(addWin()));
	addWindow->setShortcut(QKeySequence::New);
	QAction* addCustomWindow = new QAction("Create Custom Window", nullptr);
	QObject::connect(addCustomWindow, SIGNAL(triggered()), this, SLOT(addCustomWin()));
	addCustomWindow->setShortcut(QKeySequence::AddTab);
	QAction* addWidget = new QAction("Add Widget", nullptr);
	addWidget->setShortcut(tr("Ctrl+Shift+N"));
	QObject::connect(addWidget, SIGNAL(triggered()), this, SLOT(addWidget()));
	fileMenu->addAction(addWidget);
	fileMenu->addAction(addWindow);
	fileMenu->addAction(addCustomWindow);
	fileMenu->addAction(quitAct);
}

void
LayoutGenerator::changeLayout(WidgetLoader *wid, const QString &type, int rows, int cols)
{
	QGridLayout* grid = wid->getLayoutHandle(); //TODO get rid of layout handle and look at parent instead
	if (type == "grid_layout")
	{
		QGridLayout* ngrid = new QGridLayout();
		for (int x = 0; x < rows; x++)
		{
			for (int y = 0; y < cols; y++)
			{
				WidgetLoader* wl = new WidgetLoader();
				wl->setLayoutHandle(ngrid);
				ngrid->addWidget(wl, x, y);
				wl->linkLayoutGenerator(
						std::bind(&LayoutGenerator::changeWidget, this, std::placeholders::_1,
								std::placeholders::_2),
						std::bind(&LayoutGenerator::changeLayout, this, std::placeholders::_1,
								std::placeholders::_2, std::placeholders::_3,
								std::placeholders::_4), widgetFactory_.getWidgetTypes());
			}
		}
		QWidget* wGrid = new QWidget();
		wGrid->setLayout(ngrid);
		grid->replaceWidget(wid, wGrid);
		wid->close();
	}
	else if (type == "tab_layout")
	{
		QTabWidget* tabs = new QTabWidget();
		for (int i = 0; i < rows; i++)
		{
			QString name = "Tab " + QString::number(i + 1);
			WidgetLoader* wl = new WidgetLoader();
			tabs->addTab(wl, name);
			wl->setTabIndex(i);
			wl->linkLayoutGenerator(
					std::bind(&LayoutGenerator::changeWidget, this, std::placeholders::_1,
							std::placeholders::_2),
					std::bind(&LayoutGenerator::changeLayout, this, std::placeholders::_1,
							std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
					widgetFactory_.getWidgetTypes());
			wl->setTabHandle(tabs);
			wl->setLayoutHandle(grid);
		}
		grid->replaceWidget(wid, tabs);
		wid->close();
	}
}

void
LayoutGenerator::changeWidget(WidgetLoader* wid, const QString& type)
{
	QGridLayout* grid = wid->getLayoutHandle(); //TODO get rid of layout handle and look at parent instead
	if (type == "blank")
	{
		wid->close();
	}
	else
	{
		QWidget* newWidget = createWidget(type.toStdString(), wid->parentWidget());
		if (wid->getTabHandle() == NULL)
			grid->replaceWidget(wid, newWidget);
		else
		{
			wid->getTabHandle()->addTab(newWidget, type);
			wid->getTabHandle()->removeTab(0);
		}
		wid->close();
	}
	delete wid;

}

QWidget*
LayoutGenerator::createLayout(const boost::property_tree::ptree& json, QWidget* parent)
{
	//const QString type = json["type"].toString();
	//QJsonObject config;
	//if (json.contains("config"))
	//		config = json["config"].toObject();
	PropertyMapper pm(json);
	std::string type;
	if (!pm.add("type", type, true))
	{
		APLOG_WARN << "json object did not have type";
		return new QWidget(parent);
	}

	boost::property_tree::ptree config;

	//
	// LAYOUTS
	//
	if (pm.add("config", config, false))
	{
		PropertyMapper config_pm(config);
		if (type == "horizontal_layout")
		{
			QWidget* wHBox = new QWidget(parent);
			//QJsonArray items = config["items"].toArray();
			std::vector<boost::property_tree::ptree> items;
			if (!config_pm.addVector("items", items, true))
			{
				APLOG_WARN << "horizontal layout has no items";
				return new QWidget(parent);
			}
			QHBoxLayout* hbox = new QHBoxLayout(wHBox);
			for (auto item : items)
			{
				//QJsonObject item = items.at(i).toObject();
				QWidget* wItem = createLayout(item, wHBox);
				hbox->addWidget(wItem);
			}
			wHBox->setLayout(hbox);
			return wHBox;
		}
		if (type == "vertical_layout")
		{
			QWidget* wVBox = new QWidget(parent);
			//QJsonArray items = config["items"].toArray();
			std::vector<boost::property_tree::ptree> items;
			if (!config_pm.addVector("items", items, true))
			{
				APLOG_WARN << "vertical layout has no items";
				return new QWidget(parent);
			}
			QVBoxLayout* vbox = new QVBoxLayout(wVBox);
			for (auto item : items)
			{
				//QJsonObject item = items.at(i).toObject();
				QWidget* wItem = createLayout(item, wVBox);
				vbox->addWidget(wItem);
			}

			wVBox->setLayout(vbox);
			return wVBox;
		}
		if (type == "quad_layout")
		{
			boost::property_tree::ptree nw, ne, sw, se;
			QWidget* wGrid = new QWidget(parent);
			QGridLayout* grid(new QGridLayout(wGrid));
			QWidget* wNW, *wNE, *wSW, *wSE;
			if (config_pm.add("nw", nw, true))
			{
				wNW = createLayout(nw, wGrid);
			}
			else
			{
				APLOG_ERROR << "No nw element specified in quad layout";
				wNW = new QWidget(parent);
			}

			if (config_pm.add("ne", ne, true))
			{
				wNE = createLayout(ne, wGrid);
			}
			else
			{
				APLOG_ERROR << "No ne element specified in quad layout";
				wNE = new QWidget(parent);
			}

			if (config_pm.add("sw", sw, true))
			{
				wSW = createLayout(sw, wGrid);
			}
			else
			{
				APLOG_ERROR << "No sw element specified in quad layout";
				wSW = new QWidget(parent);
			}

			if (config_pm.add("se", se, true))
			{
				wSE = createLayout(se, wGrid);
			}
			else
			{
				APLOG_ERROR << "No se element specified in quad layout";
				wSE = new QWidget(parent);
			}
			grid->addWidget(wNW, 0, 0);
			grid->addWidget(wNE, 0, 1);
			grid->addWidget(wSW, 1, 0);
			grid->addWidget(wSE, 1, 1);

			wGrid->setLayout(grid);

			return wGrid;
		}
		if (type == "tab_layout")
		{
			//QJsonArray tabs = config["tabs"].toArray();
			std::vector<boost::property_tree::ptree> items;
			if (!config_pm.addVector("items", items, true))
			{
				APLOG_WARN << "tab layout has no items";
				return new QWidget(parent);
			}
			QTabWidget* wTabs = new QTabWidget();
			for (int x = 0; x < items.size(); x++)
			{
				boost::property_tree::ptree tab = items[x];
				//QString name = tab["name"].toString();
				PropertyMapper pm(tab);
				std::string name = "tab " + (x + 1);
				pm.add("name", name, false);
				QWidget* wTab = createLayout(tab, parent);
				wTabs->addTab(wTab, QString::fromStdString(name));
			}
			return wTabs;
		}
	}

	return createWidget(type, parent);
}

QWidget*
LayoutGenerator::createWidget(const std::string& type, QWidget* parent)
{
	QWidget* wid = widgetFactory_.createWidget(type, shared_from_this(), parent);
	return wid != nullptr ? wid : new QWidget(parent);
}

void
LayoutGenerator::makeScrollableWin(QWidget* win)
{
	QMainWindow* mainWin(new QMainWindow);
	QScrollArea* scrollArea(new QScrollArea(mainWin));
	scrollArea->setWidgetResizable(true);
	win->setParent(scrollArea);
	scrollArea->setWidget(win);
	addMenus(mainWin);
	mainWin->setCentralWidget(scrollArea);
	mainWin->show();
	windows_.push_back(mainWin);
}

void
LayoutGenerator::handleQuit()
{
	for (auto win : windows_)
	{
		if (!win)
			continue;
		win->close();
		delete win;
	}
}

void
LayoutGenerator::addWin()
{
	APLOG_TRACE << "Prompting for config file.";
	QFileDialog dialog;
	dialog.setWindowTitle("Open Configuration");
	dialog.setFileMode(QFileDialog::ExistingFile);
	dialog.setAcceptMode(QFileDialog::AcceptOpen);
	QString confPath;
	if (dialog.exec())
	{
		confPath = dialog.selectedFiles().front();
	}
	else
	{
		APLOG_TRACE << "Cancelled config loading.";
		return;
	}
	boost::property_tree::ptree config;
	boost::property_tree::read_json(confPath.toStdString(), config);
	makeScrollableWin(createLayout(config, NULL));
}

void
LayoutGenerator::addWidget()
{
	QGridLayout* grid(new QGridLayout());
	QMainWindow* win(new QMainWindow);
	WidgetLoader* wl(new WidgetLoader());
	wl->linkLayoutGenerator(
			std::bind(&LayoutGenerator::changeWidget, this, std::placeholders::_1,
					std::placeholders::_2),
			std::bind(&LayoutGenerator::changeLayout, this, std::placeholders::_1,
					std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
			widgetFactory_.getWidgetTypes());
	wl->setLayoutHandle(grid);
	QWidget* wGrid = new QWidget();
	wGrid->setLayout(grid);
	grid->addWidget(wl);
	win->setCentralWidget(wGrid);
	addMenus(win);
	win->show();
	win->resize(350, 100);
	windows_.push_back(std::move(win));
}

void
LayoutGenerator::addCustomWin()
{
	QGridLayout* grid = new QGridLayout();
	QWidget* wGrid = new QWidget();
	for (int x = 0; x < 2; x++)
	{
		for (int y = 0; y < 2; y++)
		{
			WidgetLoader* wl = new WidgetLoader(wGrid);
			wl->setLayoutHandle(grid);
			grid->addWidget(wl, x, y);
			wl->linkLayoutGenerator(
					std::bind(&LayoutGenerator::changeWidget, this, std::placeholders::_1,
							std::placeholders::_2),
					std::bind(&LayoutGenerator::changeLayout, this, std::placeholders::_1,
							std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
					widgetFactory_.getWidgetTypes());
		}
	}

	wGrid->setLayout(grid);
	makeScrollableWin(wGrid);
}
