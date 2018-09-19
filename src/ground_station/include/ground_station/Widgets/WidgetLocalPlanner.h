/*
 * WidgetLocalPlanner.h
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETLOCALPLANNER_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETLOCALPLANNER_H_
#include <google/protobuf/message.h>
#include <ground_station/IWidgetInterface.h>
#include <QWidget>
#include <uavAP/Core/DataPresentation/Content.h>

namespace Ui
{
class WidgetLocalPlanner;
}

class IConfigManager;

class WidgetLocalPlanner: public QWidget
{
Q_OBJECT

public:
	static constexpr auto widgetName = "local_planner";

	explicit
	WidgetLocalPlanner(QWidget* parent = 0);

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetLocalPlanner(parent));
		widget->connectInterface(interface);
		return widget;
	}

public slots:

	void
	on_pushButton_clicked();

	void
	on_comboBox_currentIndexChanged(const QString& name);

private:

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	google::protobuf::Message* params_;
	Tuning currentTuning_;

	ObjectHandle<IConfigManager> configManager_;

	Ui::WidgetLocalPlanner* ui;
};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETLOCALPLANNER_H_ */
