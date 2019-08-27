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
#include "ground_station/Widgets/FlightView/WidgetAirplaneView.h"
#include <QtGui>
#include <uavAP/Core/Logging/APLogger.h>
#include "ground_station/ConfigManager.h"

WidgetAirplaneView::WidgetAirplaneView(QWidget *parent) :
		QWidget(parent)
{
	controlSurfaceView = new WidgetControlSurface(this);
	meterLLL = new WidgetLevelMeter(this);
	meterLL = new WidgetLevelMeter(this);
	meterL = new WidgetLevelMeter(this);
	meterR = new WidgetLevelMeter(this);
	meterRR = new WidgetLevelMeter(this);
	meterRRR = new WidgetLevelMeter(this);
	rpmT = new WidgetSlider(this);
	rpmB = new WidgetSlider(this);
	rpmT->setOrientation(WidgetSlider::RIGHT);
	rpmB->setOrientation(WidgetSlider::RIGHT);

	/*// NOTE hid these widgets per Or's request
	 meterRRR->setVisible(false);
	 battRRR.setVisible(false);
	 maxRRR.setVisible(false);
	 currRRR.setVisible(false);
	 rpmB->setVisible(false);
	 nameRRR.setVisible(false);
	 */
	battLLL.setParent(this);
	battLL.setParent(this);
	battL.setParent(this);
	battRRR.setParent(this);
	battRR.setParent(this);
	battR.setParent(this);
	currLLL.setParent(this);
	maxLLL.setParent(this);
	nameLLL.setParent(this);
	currLL.setParent(this);
	maxLL.setParent(this);
	nameLL.setParent(this);
	currL.setParent(this);
	maxL.setParent(this);
	nameL.setParent(this);
	currRRR.setParent(this);
	maxRRR.setParent(this);
	nameRRR.setParent(this);
	currRR.setParent(this);
	maxRR.setParent(this);
	nameRR.setParent(this);
	currR.setParent(this);
	maxR.setParent(this);
	nameR.setParent(this);

	battLLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	battLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	battL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	battRRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	battRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	battR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currLLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxLLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameLLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameLL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameL.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currRRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxRRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameRRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameRR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	currR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	maxR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	nameR.setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

	battLLL.setStyleSheet("font: 7pt;");
	battLL.setStyleSheet("font: 7pt;");
	battL.setStyleSheet("font: 7pt;");
	battRRR.setStyleSheet("font: 7pt;");
	battRR.setStyleSheet("font: 7pt;");
	battR.setStyleSheet("font: 7pt;");
	currLLL.setStyleSheet("font: 7pt;");
	maxLLL.setStyleSheet("font: 7pt;");
	nameLLL.setStyleSheet("font: 7pt;");
	currLL.setStyleSheet("font: 7pt;");
	maxLL.setStyleSheet("font: 7pt;");
	nameLL.setStyleSheet("font: 7pt;");
	currL.setStyleSheet("font: 7pt;");
	maxL.setStyleSheet("font: 7pt;");
	nameL.setStyleSheet("font: 7pt;");
	currRRR.setStyleSheet("font: 7pt;");
	maxRRR.setStyleSheet("font: 7pt;");
	nameRRR.setStyleSheet("font: 7pt;");
	currRR.setStyleSheet("font: 7pt;");
	maxRR.setStyleSheet("font: 7pt;");
	nameRR.setStyleSheet("font: 7pt;");
	currR.setStyleSheet("font: 7pt;");
	maxR.setStyleSheet("font: 7pt;");
	nameR.setStyleSheet("font: 7pt;");

	battLLL.setText("Battery");
	battLL.setText("Battery");
	battL.setText("Battery");
	battRRR.setText("Battery");
	battRR.setText("Battery");
	battR.setText("Battery");

	currLLL.setText("00.00 V");
	maxLLL.setText("00.00 V");
	nameLLL.setText("Temp");
	currLL.setText("00.00 V");
	maxLL.setText("00.00 V");
	nameLL.setText("Temp");
	currL.setText("00.00 V");
	maxL.setText("00.00 V");
	nameL.setText("Temp");
	currRRR.setText("00.00 V");
	maxRRR.setText("00.00 V");
	nameRRR.setText("Temp");
	currRR.setText("00.00 V");
	maxRR.setText("00.00 V");
	nameRR.setText("Temp");
	currR.setText("00.00 V");
	maxR.setText("00.00 V");
	nameR.setText("Temp");
}

WidgetAirplaneView::~WidgetAirplaneView()
{
	delete controlSurfaceView;
	delete meterLLL;
	delete meterLL;
	delete meterL;
	delete meterR;
	delete meterRR;
	delete meterRRR;
	delete rpmT;
	delete rpmB;
}

void
WidgetAirplaneView::configure(const Configuration &json)
{
	if (json.contains("cs"))
	{
		controlSurfaceView->configure(json["cs"].toObject());
	}
	else
	{
		controlSurfaceView->setVisible(false);
	}
	if (json.contains("levelL3"))
	{
		QJsonObject j = json["levelL3"].toObject();
		if (j.contains("name"))
		{
			nameLLL.setText(j["name"].toString());
		}
	}
	if (json.contains("levelL2"))
	{
		QJsonObject j = json["levelL2"].toObject();
		if (j.contains("name"))
		{
			nameLL.setText(j["name"].toString());
		}
	}
	if (json.contains("levelL1"))
	{
		QJsonObject j = json["levelL1"].toObject();
		if (j.contains("name"))
		{
			nameL.setText(j["name"].toString());
		}
	}
	if (json.contains("levelR1"))
	{
		QJsonObject j = json["levelR1"].toObject();
		if (j.contains("name"))
		{
			nameR.setText(j["name"].toString());
		}
	}
	if (json.contains("levelR2"))
	{
		QJsonObject j = json["levelR2"].toObject();
		if (j.contains("name"))
		{
			nameRR.setText(j["name"].toString());
		}
	}
	if (json.contains("levelR3"))
	{
		QJsonObject j = json["levelR3"].toObject();
		if (j.contains("name"))
		{
			nameRRR.setText(j["name"].toString());
		}
	}
	if (json.contains("rpmB"))
	{
		rpmB->configure(json["rpmB"].toObject());
	}
	else
	{
		rpmB->setVisible(false);
	}
	if (json.contains("rpmT"))
	{
		rpmT->configure(json["rpmT"].toObject());
	}
	else
	{
		rpmT->setVisible(false);
	}
}

void
WidgetAirplaneView::connectAggregator(const std::shared_ptr<Aggregator> agg)
{
	//connect(std::shared_ptr<IDataSignals> dataSignals, const QJsonObject &config)
	controlSurfaceView->connect(agg);
	//dataSignals.subscribeOnNewActuationData(boost::bind(&WidgetAirplaneView::on_hasNewSample, this, _1));
	auto sender = std::dynamic_pointer_cast<QObject>(agg->getOne<IDataSignals>());
	if (sender)
		QObject::connect(sender.get(), SIGNAL(onActuationData(const ActuationData&)), this,
				SLOT(on_hasNewSample(const ActuationData&)));
	else
		APLOG_ERROR << "WidgetAirplaneView: Couldn't dynamic cast signal sender!";
	configure(agg->getOne<ConfigManager>()->getWidgetConfigByName("airplane_view"));
}

/*
 void WidgetAirplaneView::paintEvent(QPaintEvent *)
 {
 const QColor BLACK(0,0,0);
 QPainter painter(this);
 painter.fillRect(0,0,this->width(),this->height(), BLACK);

 controlSurfaceView->repaint();
 meterLLL->repaint();
 meterLL->repaint();
 meterL->repaint();
 meterR->repaint();
 meterRR->repaint();
 meterRRR->repaint();
 rpmT->repaint();
 rpmB->repaint();

 currLLL.repaint();
 maxLLL.repaint();
 nameLLL.repaint();controlSurfaceView
 currLL.repaint();
 maxLL.repaint();
 nameLL.repaint();
 currL.repaint();
 maxL.repaint();
 nameL.repaint();
 currRRR.repaint();
 maxRRR.repaint();
 nameRRR.repaint();
 currRR.repaint();
 maxRR.repaint();
 nameRR.repaint();
 currR.repaint();
 maxR.repaint();
 nameR.repaint();
 }*/

void
WidgetAirplaneView::resizeEvent(QResizeEvent *)
{
	QPoint origin(0, 0);
	float U = this->size().width() / 160.0;
	if (this->size().height() != 90.0 * U)
	{
		float rat = this->size().width() / this->size().height();
		if (rat > 160.0 / 90.0)
		{
			U = this->size().height() / 90.0;
			origin.setX((this->size().width() - 160.0 * U) / 2);
		}
		else
		{
			origin.setY((this->size().height() - 90.0 * U) / 2);
		}
	}
	QPoint controlSurfaceViewOrigin = QPoint(30 * U, 0) + origin;
	QSize controlSurfaceViewSize = QSize(100 * U, 70 * U);

	controlSurfaceView->setGeometry(QRect(controlSurfaceViewOrigin, controlSurfaceViewSize));
	QPoint meterLLLOrigin = QPoint(3 * U, 6 * U) + origin;
	QSize meterSize = QSize(6 * U, 60 * U);
	meterLLL->setGeometry(QRect(meterLLLOrigin, meterSize));

	QPoint meterLLOrigin = QPoint(12 * U, 6 * U) + origin;
	meterLL->setGeometry(QRect(meterLLOrigin, meterSize));

	QPoint meterLOrigin = QPoint(21 * U, 6 * U) + origin;
	meterL->setGeometry(QRect(meterLOrigin, meterSize));

	QPoint meterROrigin = QPoint(133 * U, 6 * U) + origin;
	meterR->setGeometry(QRect(meterROrigin, meterSize));

	QPoint meterRROrigin = QPoint(142 * U, 6 * U) + origin;
	meterRR->setGeometry(QRect(meterRROrigin, meterSize));

	QPoint meterRRROrigin = QPoint(151 * U, 6 * U) + origin;
	meterRRR->setGeometry(QRect(meterRRROrigin, meterSize));

	QPoint rpmTOrigin = QPoint(50 * U, 73 * U) + origin;
	QSize rpmSize = QSize(60 * U, 6 * U);
	rpmT->setGeometry(QRect(rpmTOrigin, rpmSize));

	QPoint rpmBOrigin = QPoint(50 * U, 82 * U) + origin;
	rpmB->setGeometry(QRect(rpmBOrigin, rpmSize));

	QSize labelSize = QSize(8 * U, 4 * U);
	QPoint currLLLOrigin = QPoint(2 * U, 68 * U) + origin;
	currLLL.setGeometry(QRect(currLLLOrigin, labelSize));

	QPoint maxLLLOrigin = QPoint(2 * U, 73 * U) + origin;
	maxLLL.setGeometry(QRect(maxLLLOrigin, labelSize));

	QPoint nameLLLOrigin = QPoint(2 * U, 79 * U) + origin;
	nameLLL.setGeometry(QRect(nameLLLOrigin, labelSize));

	QPoint battLLLOrigin = QPoint(2 * U, 83 * U) + origin;
	battLLL.setGeometry(QRect(battLLLOrigin, labelSize));

	QPoint currLLOrigin = QPoint(11 * U, 68 * U) + origin;
	currLL.setGeometry(QRect(currLLOrigin, labelSize));

	QPoint maxLLOrigin = QPoint(11 * U, 73 * U) + origin;
	maxLL.setGeometry(QRect(maxLLOrigin, labelSize));

	QPoint nameLLOrigin = QPoint(11 * U, 79 * U) + origin;
	nameLL.setGeometry(QRect(nameLLOrigin, labelSize));

	QPoint battLLOrigin = QPoint(11 * U, 83 * U) + origin;
	battLL.setGeometry(QRect(battLLOrigin, labelSize));

	QPoint currLOrigin = QPoint(20 * U, 68 * U) + origin;
	currL.setGeometry(QRect(currLOrigin, labelSize));

	QPoint maxLOrigin = QPoint(20 * U, 73 * U) + origin;
	maxL.setGeometry(QRect(maxLOrigin, labelSize));

	QPoint nameLOrigin = QPoint(20 * U, 79 * U) + origin;
	nameL.setGeometry(QRect(nameLOrigin, labelSize));

	QPoint battLOrigin = QPoint(20 * U, 83 * U) + origin;
	battL.setGeometry(QRect(battLOrigin, labelSize));

	QPoint currRRROrigin = QPoint(150 * U, 68 * U) + origin;
	currRRR.setGeometry(QRect(currRRROrigin, labelSize));

	QPoint maxRRROrigin = QPoint(150 * U, 73 * U) + origin;
	maxRRR.setGeometry(QRect(maxRRROrigin, labelSize));

	QPoint nameRRROrigin = QPoint(150 * U, 79 * U) + origin;
	nameRRR.setGeometry(QRect(nameRRROrigin, labelSize));

	QPoint battRRROrigin = QPoint(150 * U, 83 * U) + origin;
	battRRR.setGeometry(QRect(battRRROrigin, labelSize));

	QPoint currRROrigin = QPoint(141 * U, 68 * U) + origin;
	currRR.setGeometry(QRect(currRROrigin, labelSize));

	QPoint maxRROrigin = QPoint(141 * U, 73 * U) + origin;
	maxRR.setGeometry(QRect(maxRROrigin, labelSize));

	QPoint nameRROrigin = QPoint(141 * U, 79 * U) + origin;
	nameRR.setGeometry(QRect(nameRROrigin, labelSize));

	QPoint battRROrigin = QPoint(141 * U, 83 * U) + origin;
	battRR.setGeometry(QRect(battRROrigin, labelSize));

	QPoint currROrigin = QPoint(132 * U, 68 * U) + origin;
	currR.setGeometry(QRect(currROrigin, labelSize));

	QPoint maxROrigin = QPoint(132 * U, 73 * U) + origin;
	maxR.setGeometry(QRect(maxROrigin, labelSize));

	QPoint nameROrigin = QPoint(132 * U, 79 * U) + origin;
	nameR.setGeometry(QRect(nameROrigin, labelSize));

	QPoint battROrigin = QPoint(132 * U, 83 * U) + origin;
	battR.setGeometry(QRect(battROrigin, labelSize));
}

void
WidgetAirplaneView::on_hasNewSample(const simulation_interface::actuation& ad)
{
	//TODO adapt with new actuation data struct
	/*rpmT->current = 0; //sensor data does not provide airplane rpm
	 rpmB->current = 900;

	 double dtxMinV = 9.0, dtxMaxV = 12.6;
	 double dtxMinD = 1855, dtxMaxD = 2600;
	 QString dtxMaxS;
	 dtxMaxS = dtxMaxS.sprintf("%5.2f V", dtxMaxV);
	 QString dtxCurrS;

	 double vtxMinV = 9.0, vtxMaxV = 12.6;
	 double vtxMinD = 1855, vtxMaxD = 2600;
	 QString vtxMaxS;
	 vtxMaxS = vtxMaxS.sprintf("%5.2f V", vtxMaxV);
	 QString vtxCurrS;

	 double rxMinV = 6.0, rxMaxV = 8.4;
	 double rxMinD = 1975, rxMaxD = 2765;
	 QString rxMaxS;
	 rxMaxS = rxMaxS.sprintf("%5.2f V", rxMaxV);
	 QString rxCurrS;

	 double motMinV = 12.0, motMaxV = 16.8;
	 double motMinD = 1795, motMaxD = 2510;
	 QString motMaxS;
	 motMaxS = motMaxS.sprintf("%5.2f V", motMaxV);
	 QString motCurrS;

	 double aviMinV = 6.0, aviMaxV = 8.4;
	 QString aviMaxS;
	 aviMaxS = aviMaxS.sprintf("%5.2f V", aviMaxV);
	 QString aviCurrS;

	 double dtxP = (ad.elevatorR - dtxMinD)/(dtxMaxD - dtxMinD);
	 double dtxV = dtxP*(dtxMaxV - dtxMinV) + dtxMinV;
	 dtxCurrS = dtxCurrS.sprintf("%5.2f V", dtxV);
	 meterR->level = dtxP;
	 maxR.setText(dtxMaxS);
	 currR.setText(dtxCurrS);

	 double vtxP = (ad.flapL - vtxMinD)/(vtxMaxD - vtxMinD);
	 double vtxV = vtxP*(vtxMaxV - vtxMinV) + vtxMinV;
	 vtxCurrS = vtxCurrS.sprintf("%5.2f V", vtxV);
	 meterRR->level = vtxP;
	 maxRR.setText(vtxMaxS);
	 currRR.setText(vtxCurrS);

	 double rxP = (ad.aileronR - rxMinD)/(rxMaxD - rxMinD);
	 double rxV = rxP*(rxMaxV - rxMinV) + rxMinV;
	 rxCurrS = rxCurrS.sprintf("%5.2f V", rxV);
	 meterLL->level = rxP;
	 maxLL.setText(rxMaxS);
	 currLL.setText(rxCurrS);

	 double motP = (ad.aileronL - motMinD)/(motMaxD - motMinD);
	 double motV = motP*(motMaxV - motMinV) + motMinV;
	 motCurrS = motCurrS.sprintf("%5.2f V", motV);
	 meterLLL->level = motP;
	 maxLLL.setText(motMaxS);
	 currLLL.setText(motCurrS);

	 double aviP = 0.8*dtxP;
	 double aviV = aviP*(aviMaxV - aviMinV) + aviMinV;
	 aviCurrS = aviCurrS.sprintf("%5.2f V", aviV);
	 meterL->level = aviP;
	 maxL.setText(aviMaxS);
	 currL.setText(aviCurrS);*/

	this->update();
}
