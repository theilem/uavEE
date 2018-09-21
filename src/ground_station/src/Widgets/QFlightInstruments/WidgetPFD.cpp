/***************************************************************************//**
 * @file example/WidgetPFD.cpp
 * @author  Marek M. Cel <marekcel@mscsim.org>
 *
 * @section LICENSE
 *
 * Copyright (C) 2013 Marek M. Cel
 *
 * This file is part of QFlightInstruments. You can redistribute and modify it
 * under the terms of GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Further information about the GNU General Public License can also be found
 * on the world wide web at http://www.gnu.org.
 *
 * ---
 *
 * Copyright (C) 2013 Marek M. Cel
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/
#ifndef WIDGETPFD_CPP
#define WIDGETPFD_CPP
#endif

#include "ground_station/Widgets/QFlightInstruments/WidgetPFD.h"
#include "ui_WidgetPFD.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IWidgetInterface.h"
#include "ground_station/ConfigManager.h"
#include <cmath>
#include <uavAP/Core/Logging/APLogger.h>
////////////////////////////////////////////////////////////////////////////////

WidgetPFD::WidgetPFD(QWidget* parent) :
		QWidget(parent), m_ui(new Ui::WidgetPFD), m_pfd(0), m_layoutSq(0)
{
	m_ui->setupUi(this);

	setupUi();

	m_pfd = m_ui->graphicsPFD;
}

////////////////////////////////////////////////////////////////////////////////

WidgetPFD::~WidgetPFD()
{
	if (m_layoutSq)
	{
		delete m_layoutSq;
		m_layoutSq = 0;
	}

	if (m_ui)
	{
		delete m_ui;
		m_ui = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

/*auto sender = std::dynamic_pointer_cast<QObject>(dataSignals);
 if (sender)
 QObject::connect(sender.get(), SIGNAL(onSensorData(const SensorData&)), this, SLOT(on_hasNewSample(const SensorData&)));
 else
 APLOG_ERROR << "WidgetPFD: Couldn't dynamic cast signal sender!";*/

void
WidgetPFD::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetPFD received null interface";
	}
	if (auto ds = interface->getIDataSignals().get())
		QObject::connect(ds.get(), SIGNAL(onSensorData(const simulation_interface::sensor_data&)),
				this, SLOT(on_hasNewSample(const simulation_interface::sensor_data&)));
	else
		APLOG_ERROR << "Cannot connect WidgetPFD. IDataSignals missing.";
}

void
WidgetPFD::on_hasNewSample(const simulation_interface::sensor_data& s)
{
	setAltitude(s.position.z);
	setGroundspeed(s.ground_speed);
	setAirspeed(s.air_speed);
	setRoll(s.attitude.x * 180. / M_PI);
	setPitch(s.attitude.y * 180. / M_PI);
	setHeading(s.attitude.z * 180. / M_PI);
	update();
}

void
WidgetPFD::setupUi()
{
	m_layoutSq = new LayoutSquare(this);

	m_layoutSq->setContentsMargins(0, 0, 0, 0);
	m_layoutSq->addWidget(m_ui->framePFD);

	this->setLayout(m_layoutSq);
}
