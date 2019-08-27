
/*
 * WidgetManualWind.cpp
 *
 *  Created on: Feb 22, 2019
 *      Author: mirco
 */
#include <ground_station/Widgets/WidgetManualWind.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/FlightAnalysis/WindEstimation/WindInfo.h>
#include <uavAP/Core/DataHandling/DataHandling.h>
#include "ui_WidgetManualWind.h"

WidgetManualWind::WidgetManualWind(QWidget* parent):QWidget(parent)
,ui(new Ui::WidgetManualWind)
{
	ui->setupUi(this);
}

void
WidgetManualWind::on_send_clicked()
{
	auto dh = dataHandling_.get();
	if (!dh)
	{
		APLOG_ERROR << "General DataHandling not supported. Cannot send Wind.";
		return;
	}
	Vector3 w;
	w[0] = ui->x->text().toDouble();
	w[1] = ui->y->text().toDouble();
	w[2] = ui->z->text().toDouble();

	WindInfo wind;
	wind.direction = w.normalized();
	wind.speed = w.norm();

	dh->sendData(wind, Content::WIND_INFO, Target::FLIGHT_CONTROL);
}

void
WidgetManualWind::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	dataHandling_.set(interface->getDataHandling().get());
}
