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
#include <ground_station/Widgets/FlightView/WidgetControlSurface.h>
#include <QtGui>
#include <QPainterPath>

#include <uavAP/Core/Logging/APLogger.h>

WidgetControlSurface::WidgetControlSurface(QWidget *parent) :
		QWidget(parent), adcPrimary(false), actuationPrimary(true), adcSecondary(false), actuationSecondary(
				false) //, ui(new Ui::WidgetControlSurface)
{
	for (int i = 0; i < NUM_CONTROL_SURFACES; i++)
	{
		valuesact[i] = 2048;
		mids[i] = 2048;
		mins[i] = 0;
		maxs[i] = 4096;
		reverses[i] = false;
	}
}

void
WidgetControlSurface::configure(const QJsonObject &json)
{
	QStringList surfaces;
	surfaces << "elevatorL" << "elevatorR" << "rudder" << "wingLO" << "wingRO" << "wingLI"
			<< "wingRI" << "displaySettings";
	APLOG_DEBUG << "WidgetControlSurface config json count: " << json.count();
	for (int i = 0; i < 8; i++)
	{
		if (json.contains(surfaces[i]))
		{
			QJsonObject surface = json[surfaces[i]].toObject();
			if (surface.contains("value"))
			{
				valuesact[i] = surface["value"].toInt();
			}
			if (surface.contains("center"))
			{
				mids[i] = surface["center"].toInt();
			}
			if (surface.contains("max"))
			{
				maxs[i] = surface["max"].toInt();
			}
			if (surface.contains("min"))
			{
				mins[i] = surface["min"].toInt();
			}
			if (surface.contains("reverse"))
			{
				reverses[i] = surface["reverse"].toBool();
			}
			if (surface.contains("primary"))
			{
				if (surface["primary"] == "adc")
				{
					qDebug() << "Changing ADC to primary view in Control Surface";
					adcPrimary = true;
					actuationPrimary = false;
				}
				/*if(surface["primary"]=="pwm"){
				 qDebug() << "Changing PWM to primary view in Control Surface";
				 pwmPrimary = true;
				 actuationPrimary = false;
				 }*/
			}
			if (surface.contains("secondary"))
			{
				if (surface["secondary"] == "adc")
				{
					qDebug() << "Setting ADC to secondary view in Control Surface";
					adcSecondary = true;
				}
				/*else if(surface["secondary"]=="pwm"){
				 qDebug() << "Setting PWM to secondary view in Control Surface";
				 pwmSecondary = true;
				 }*/
				else
				{
					qDebug() << "Setting Actuation to secondary view in Control Surface";
					actuationSecondary = true;
				}
			}
		}
	}

	/*if(json.contains(primary)){
	 if(primary.contains("adc")){
	 adcPrimary = true;
	 actuationPrimary = false;
	 }
	 }
	 if(json.contains(secondary)){
	 if(secondary.contains("adc"))
	 adcSecondary = true;
	 else
	 actuationSecondary = true;
	 }*/
}

void
WidgetControlSurface::connect(std::shared_ptr<IDataSignals> dataSignals, const QJsonObject &json)
{
	auto sender = std::dynamic_pointer_cast<QObject>(dataSignals);
	if (sender)
		QObject::connect(sender.get(), SIGNAL(onActuationData(const ActuationData&)), this,
				SLOT(on_hasNewSample(const ActuationData&)));
	else
		APLOG_ERROR << "WidgetControlSurface: Couldn't dynamic cast signal sender!";
	configure(json);
}

void
WidgetControlSurface::paintEvent(QPaintEvent *)
{
	const QColor BLACK(0, 0, 0);
	const QColor WHITE(255, 255, 255);
	const QColor BLUE(0, 0, 255);
	const QColor GREEN(0, 255, 0);
	//const QColor RED(255,0,0);

	QPoint origin(0, 0);
	int U = this->size().width() / 20;
	if (this->size().height() != 9 * U)
	{
		float rat = this->size().width() / this->size().height();
		if (rat > 20.0 / 9.0)
		{
			U = this->size().height() / 9;
			origin.setX((this->size().width() - 20 * U) / 2);
		}
		else
		{
			origin.setY((this->size().height() - 9 * U) / 2);
		}
	}

	const QPoint fuselageCenter(10 * U + origin.x(), 7 * U + origin.y());
	const int fuselageRadius = U;
	const int PADDING = (U / 40 > 1) ? U / 40 : 1;

	QPoint pts[NUM_CONTROL_SURFACES], ptsSecondary[NUM_CONTROL_SURFACES];
	pts[CS_ELEVATOR_LEFT] = QPoint(4 * U, 2 * U) + origin;
	pts[CS_ELEVATOR_RIGHT] = QPoint(12 * U, 2 * U) + origin;
	pts[CS_RUDDER] = QPoint(10 * U, 2 * U) + origin;
	pts[CS_WING_LEFT_OUTBOARD] = QPoint(0, 7 * U) + origin;
	pts[CS_WING_LEFT_INBOARD] = QPoint(4 * U, 7 * U) + origin;
	pts[CS_WING_RIGHT_INBOARD] = QPoint(12 * U, 7 * U) + origin;
	pts[CS_WING_RIGHT_OUTBOARD] = QPoint(16 * U, 7 * U) + origin;

	ptsSecondary[CS_ELEVATOR_LEFT] = QPoint(5.5 * U, 2 * U) + origin;
	ptsSecondary[CS_ELEVATOR_RIGHT] = QPoint(13.5 * U, 2 * U) + origin;
	ptsSecondary[CS_RUDDER] = QPoint(10 * U, 3.5 * U) + origin;
	ptsSecondary[CS_WING_LEFT_OUTBOARD] = QPoint(1.5 * U, 7 * U) + origin;
	ptsSecondary[CS_WING_LEFT_INBOARD] = QPoint(5.5 * U, 7 * U) + origin;
	ptsSecondary[CS_WING_RIGHT_INBOARD] = QPoint(13.5 * U, 7 * U) + origin;
	ptsSecondary[CS_WING_RIGHT_OUTBOARD] = QPoint(17.5 * U, 7 * U) + origin;

	QPainter painter(this);
	painter.fillRect(origin.x(), origin.y(), 20 * U, 9 * U, BLACK);
	QPainterPath path;
	path.addEllipse(fuselageCenter, fuselageRadius, fuselageRadius);

	QColor planeColor = controlMode == 0 ? WHITE : GREEN;
	painter.fillPath(path, QBrush(planeColor));
	for (int i = 0; i < NUM_CONTROL_SURFACES; i++)
	{
		float d = mids[i] - valuesact[i];
		float dADC = mids[i] - valuesadc[i];
		//float dPWM = mids[i] - valuespwm[i];
		if (i == CS_RUDDER)
		{
			d *= -1;
			dADC *= -1;
			//dPWM *=-1;
		}
		if (reverses[i])
		{
			d *= -1;
			dADC *= -1;
			//dPWM *=-1;
		}
		if (valuesact[i] < mids[i])
		{
			d *= (float) (2 * U) / (mids[i] - mins[i]);
			dADC *= (float) (2 * U) / (mids[i] - mins[i]);
			//dPWM *= (float)(2*U)/(mids[i]-mins[i]);
		}
		else
		{
			d *= (float) (2 * U) / (maxs[i] - mids[i]);
			dADC *= (float) (2 * U) / (maxs[i] - mids[i]);
			//dPWM *= (float)(2*U)/(maxs[i]-mids[i]);
		}
		int width = 4 * U - 2 * PADDING;

		if (actuationPrimary)
		{
			if (i == CS_RUDDER)
				painter.fillRect(pts[i].x(), pts[i].y() + PADDING, d, width, BLUE);
			else
				painter.fillRect(pts[i].x() + PADDING, pts[i].y(), width, d, BLUE);
		}
		if (adcPrimary)
		{
			if (i == CS_RUDDER)
				painter.fillRect(pts[i].x(), pts[i].y() + PADDING, dADC, width, BLUE);
			else
				painter.fillRect(pts[i].x() + PADDING, pts[i].y(), width, dADC, BLUE);
		}
		/*if(pwmPrimary){
		 if(i == CS_RUDDER)
		 painter.fillRect(pts[i].x(), pts[i].y() + PADDING, dPWM, width, BLUE);
		 else
		 painter.fillRect(pts[i].x() + PADDING, pts[i].y(), width, dPWM, BLUE);
		 }*/

		if (adcSecondary)
		{
			if (i == CS_RUDDER)
				painter.fillRect(ptsSecondary[i].x(), ptsSecondary[i].y() + PADDING, dADC,
						width / 4, GREEN);
			else
				painter.fillRect(ptsSecondary[i].x() + PADDING, ptsSecondary[i].y(), width / 4,
						dADC, GREEN);
		}
		if (actuationSecondary)
		{
			if (i == CS_RUDDER)
				painter.fillRect(ptsSecondary[i].x(), ptsSecondary[i].y() + PADDING, d, width / 4,
						GREEN);
			else
				painter.fillRect(ptsSecondary[i].x() + PADDING, ptsSecondary[i].y(), width / 4, d,
						GREEN);
		}
		/*if(pwmSecondary){
		 if(i == CS_RUDDER)
		 painter.fillRect(ptsSecondary[i].x(), ptsSecondary[i].y() + PADDING, dPWM, width/4, GREEN);
		 else
		 painter.fillRect(ptsSecondary[i].x() + PADDING, ptsSecondary[i].y(), width/4, dPWM, GREEN);
		 }*/
		//painter.fillRect(ptsADC[i].x(), pts[i].y(), 5, 5, RED);
	}

	painter.fillRect(pts[CS_ELEVATOR_LEFT].x(), pts[CS_ELEVATOR_LEFT].y() - PADDING, 12 * U,
			2 * PADDING, WHITE);
	painter.fillRect(fuselageCenter.x() - PADDING, pts[CS_ELEVATOR_LEFT].y(), 2 * PADDING, 4 * U,
			WHITE);
	painter.fillRect(pts[CS_WING_LEFT_OUTBOARD].x(), pts[CS_WING_LEFT_OUTBOARD].y() - PADDING,
			20 * U, 2 * PADDING, WHITE);
}

void
WidgetControlSurface::setMiddle(control_surface_t surface, quint16 middle)
{
	this->mids[surface] = middle;
}

void
WidgetControlSurface::setValueAct(control_surface_t surface, quint16 value)
{
	this->valuesact[surface] = value;
}

void
WidgetControlSurface::setValueADC(control_surface_t surface, quint16 value)
{
	this->valuesadc[surface] = value;
}

/*void WidgetControlSurface::setValuePWM(control_surface_t surface, quint16 value)
 {
 this->valuespwm[surface] = value;
 }*/

void
WidgetControlSurface::setMaximum(control_surface_t surface, quint16 maximum)
{
	this->maxs[surface] = maximum;
}

void
WidgetControlSurface::setMinimum(control_surface_t surface, quint16 minimum)
{
	this->mins[surface] = minimum;
}

void
WidgetControlSurface::setReverse(control_surface_t surface, bool r)
{
	this->reverses[surface] = r;
}

void
WidgetControlSurface::on_hasNewSample(const simulation_interface::actuation &ad)
{
	/*if(m) {onData(6));
	 this->setValueADC(CS_WING_RIGHT_OUTBOARD, m->adc(8
	 this->setValue(CS_WING_RIGHT_OUTBOARD, m->adc(8));
	 this->setValue(CS_WING_RIGHT_INBOARD, m->adc(9));
	 this->setValue(CS_WING_LEFT_INBOARD, m->adc(16));
	 this->setValue(CS_WING_LEFT_OUTBOARD, m->adc(17));
	 this->setValue(CS_ELEVATOR_LEFT, m->adc(6));
	 this->setValue(CS_ELEVATOR_RIGHT, m->adc(6));
	 this->setValue(CS_RUDDER, m->adc(7));
	 }*/
	//TODO fix
	/*this->setValueAct(CS_WING_LEFT_OUTBOARD, 6000 + 2400 * ad.aileronL);
	 this->setValueAct(CS_WING_RIGHT_OUTBOARD, 6000 + 2400 * ad.aileronR);
	 this->setValueAct(CS_ELEVATOR_LEFT, 6000 + 2400 * ad.elevatorL);
	 this->setValueAct(CS_ELEVATOR_RIGHT, 6000 + 2400 * ad.elevatorR);
	 this->setValueAct(CS_WING_LEFT_INBOARD, 6000 + 2400 * ad.flapL);
	 this->setValueAct(CS_WING_RIGHT_INBOARD, 6000 + 2400 * ad.flapR);
	 this->setValueAct(CS_RUDDER, 6000 + 2400 * ad.rudder);*/

	/*this->setValueADC(CS_WING_RIGHT_OUTBOARD, m->adc(8)); No ADC data from sensors
	 this->setValueADC(CS_WING_RIGHT_INBOARD, m->adc(9));
	 this->setValueADC(CS_WING_LEFT_INBOARD, m->adc(16));
	 this->setValueADC(CS_WING_LEFT_OUTBOARD, m->adc(17));
	 this->setValueADC(CS_ELEVATOR_LEFT, m->adc(6));
	 this->setValueADC(CS_ELEVATOR_RIGHT, m->adc(6));
	 this->setValueADC(CS_RUDDER, m->adc(7));*/
	/*this->setValuePWM(CS_WING_RIGHT_OUTBOARD, m->pwm(8));
	 this->setValuePWM(CS_WING_RIGHT_INBOARD, m->pwm(9));
	 this->setValuePWM(CS_WING_LEFT_INBOARD, m->pwm(16));
	 this->setValuePWM(CS_WING_LEFT_OUTBOARD, m->pwm(17));
	 this->setValuePWM(CS_ELEVATOR_LEFT, m->pwm(6));
	 this->setValuePWM(CS_ELEVATOR_RIGHT, m->pwm(6));
	 this->setValuePWM(CS_RUDDER, m->pwm(7));*/
	//controlMode = currentSample.mode;
	this->update();
}
