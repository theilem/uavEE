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
/*
 * IGSWidget.h
 *
 *  Created on: Jan 8, 2018
 *      Author: seedship
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_IGSWIDGET_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_IGSWIDGET_H_

#include <memory>
#include <QWidget>

class IWidgetInterface;

class IGSWidget: public QWidget
{
public:
	virtual
	~IGSWidget()
	{
	}

	//virtual static IGSWidget*
	//createGSWidget(std::shared_ptr<IWidgetInterface>, QWidget*) = 0;

private:

};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_IGSWIDGET_H_ */
