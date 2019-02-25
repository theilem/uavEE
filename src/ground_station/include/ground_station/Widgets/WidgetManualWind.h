/*
 * WidgetManualWind.h
 *
 *  Created on: Feb 22, 2019
 *      Author: mirco
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETMANUALWIND_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETMANUALWIND_H_
#include <ground_station/IWidgetInterface.h>
#include <QWidget>

#include <memory>

namespace Ui
{
class WidgetManualWind;
}

class WidgetManualWind: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "manual_wind";

	WidgetManualWind(QWidget* parent);

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetManualWind(parent));
		widget->connectInterface(interface);
		return widget;
	}

private slots:
	void
	on_send_clicked();

private:
	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	ObjectHandle<DataHandling> dataHandling_;

	Ui::WidgetManualWind* ui;
};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_WIDGETMANUALWIND_H_ */
