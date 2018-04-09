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
#include "ground_station/Widgets/FlightView/WidgetRadioView.h"
#include <uavAP/Core/Logging/APLogger.h>
#include <QJsonObject>

WidgetRadioView::WidgetRadioView(QWidget *parent) :
		QWidget(parent)
{
	stickL.setParent(this);
	stickR.setParent(this);

	sliderBLL.setParent(this);
	sliderBL.setParent(this);
	sliderBR.setParent(this);
	sliderBRR.setParent(this);
	sliderTL.setParent(this);
	sliderTC.setParent(this);
	sliderTR.setParent(this);

	switchLLLLL.setParent(this);
	switchLLLL.setParent(this);
	switchLLL.setParent(this);
	switchLL.setParent(this);
	switchL.setParent(this);
	switchR.setParent(this);
	switchRR.setParent(this);
	switchRRR.setParent(this);
	switchRRRR.setParent(this);
	switchRRRRR.setParent(this);

	QPalette pal(palette());
	pal.setColor(QPalette::Background, Qt::black);
	this->setAutoFillBackground(true);
	this->setPalette(pal);
}

void
WidgetRadioView::configure(const QJsonObject &json)
{
	if (json.contains("stickL"))
	{
		stickL.configure(json["stickL"].toObject());
	}
	if (json.contains("stickR"))
	{
		stickR.configure(json["stickR"].toObject());
	}
	if (json.contains("sliderBLL"))
	{
		sliderBLL.configure(json["sliderBLL"].toObject());
	}
	if (json.contains("sliderBL"))
	{
		sliderBL.configure(json["sliderBL"].toObject());
	}
	if (json.contains("sliderBR"))
	{
		sliderBR.configure(json["sliderBR"].toObject());
	}
	if (json.contains("sliderBRR"))
	{
		sliderBRR.configure(json["sliderBRR"].toObject());
	}
	if (json.contains("sliderTL"))
	{
		sliderTL.configure(json["sliderTL"].toObject());
	}
	if (json.contains("sliderTC"))
	{
		sliderTC.configure(json["sliderTC"].toObject());
	}
	if (json.contains("sliderTR"))
	{
		sliderTR.configure(json["sliderTR"].toObject());
	}
	if (json.contains("switchLLLLL"))
	{
		switchLLLLL.configure(json["switchLLLLL"].toObject());
	}
	if (json.contains("switchLLLL"))
	{
		switchLLLL.configure(json["switchLLLL"].toObject());
	}
	if (json.contains("switchLLL"))
	{
		switchLLL.configure(json["switchLLL"].toObject());
	}
	if (json.contains("switchLL"))
	{
		switchLL.configure(json["switchLL"].toObject());
	}
	if (json.contains("switchL"))
	{
		switchL.configure(json["switchL"].toObject());
	}
	if (json.contains("switchR"))
	{
		switchR.configure(json["switchR"].toObject());
	}
	if (json.contains("switchRR"))
	{
		switchRR.configure(json["switchRR"].toObject());
	}
	if (json.contains("switchRRR"))
	{
		switchRRR.configure(json["switchRRR"].toObject());
	}
	if (json.contains("switchRRRR"))
	{
		switchRRRR.configure(json["switchRRRR"].toObject());
	}
	if (json.contains("switchRRRRR"))
	{
		switchRRRRR.configure(json["switchRRRRR"].toObject());
	}
}

void
WidgetRadioView::connect(std::shared_ptr<IDataSignals> dataSignals, QJsonObject config)
{
	auto sender = std::dynamic_pointer_cast<QObject>(dataSignals);
	if (sender)
		QObject::connect(sender.get(), SIGNAL(on_ActuationData(const ActuationData&)), this,
				SLOT(on_hasNewSample(const ActuationData&)));
	else
		APLOG_ERROR << "WidgetRadioView: Couldn't dynamic cast signal sender!";

	configure(config);
}

void
WidgetRadioView::resizeEvent(QResizeEvent *)
{
	QPointF origin(0, 0);
	float U = this->size().width() / 64.0;
	if (this->size().height() != 36.0 * U)
	{
		float rat = this->size().width() / this->size().height();
		if (rat > 16.0 / 9.0)
		{
			U = this->size().height() / 36.0;
			origin.setX((this->size().width() - 64 * U) / 2);
		}
		else
		{
			origin.setY((this->size().height() - 36 * U) / 2);
		}
	}

	QSizeF topSize(3 * U, 11 * U);
	QSizeF bottomSize(3 * U, 21 * U);
	QSizeF stickSize(21 * U, 21 * U);

	switchLLLLL.setGeometry(QRectF(origin + QPointF(U, U), topSize).toRect());
	switchLLLL.setGeometry(QRectF(origin + QPointF(5 * U, U), topSize).toRect());
	switchLLL.setGeometry(QRectF(origin + QPointF(9 * U, U), topSize).toRect());
	switchLL.setGeometry(QRectF(origin + QPointF(13 * U, U), topSize).toRect());
	switchL.setGeometry(QRectF(origin + QPointF(17 * U, U), topSize).toRect());

	sliderTL.setGeometry(QRectF(origin + QPointF(26 * U, U), topSize).toRect());
	sliderTC.setGeometry(QRectF(origin + QPointF(30.5 * U, U), topSize).toRect());
	sliderTR.setGeometry(QRectF(origin + QPointF(35 * U, U), topSize).toRect());

	switchR.setGeometry(QRectF(origin + QPointF(44 * U, U), topSize).toRect());
	switchRR.setGeometry(QRectF(origin + QPointF(48 * U, U), topSize).toRect());
	switchRRR.setGeometry(QRectF(origin + QPointF(52 * U, U), topSize).toRect());
	switchRRRR.setGeometry(QRectF(origin + QPointF(56 * U, U), topSize).toRect());
	switchRRRRR.setGeometry(QRectF(origin + QPointF(60 * U, U), topSize).toRect());

	sliderBLL.setGeometry(QRectF(origin + QPointF(U, 14 * U), bottomSize).toRect());
	sliderBL.setGeometry(QRectF(origin + QPointF(5 * U, 14 * U), bottomSize).toRect());

	stickL.setGeometry(QRectF(origin + QPointF(10 * U, 14 * U), stickSize).toRect());
	stickR.setGeometry(QRectF(origin + QPointF(33 * U, 14 * U), stickSize).toRect());

	sliderBR.setGeometry(QRectF(origin + QPointF(56 * U, 14 * U), bottomSize).toRect());
	sliderBRR.setGeometry(QRectF(origin + QPointF(60 * U, 14 * U), bottomSize).toRect());
}

void
WidgetRadioView::on_hasNewSample(const simulation_interface::actuation &a)
{
	/*stickR.xCurrent = a.aileronL;
	 stickR.yCurrent = a.elevatorL;
	 stickL.yCurrent = a.flapL;
	 stickL.xCurrent = a.rudder;
	 sliderBRR.current = a.throttle;*/ //TODO fix
	this->update();
}
