////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
﻿#ifndef WIDGETANTENNADATAMANIPULATOR_H
#define WIDGETANTENNADATAMANIPULATOR_H

#include <QWidget>
#include <memory>
#include <pan_tilt_handler/PanTiltHandler.h>

namespace Ui
{
class WidgetAntennaDataManipulator;
}

class WidgetAntennaDataManipulator: public QWidget
{
Q_OBJECT

public:
	explicit
	WidgetAntennaDataManipulator(QWidget* parent = 0);
	void
	connect(std::shared_ptr<PanTiltHandler> ah);
	~WidgetAntennaDataManipulator();

private slots:
	void
	on_apply_clicked();

private:
	Ui::WidgetAntennaDataManipulator* ui;
	std::shared_ptr<PanTiltHandler> panTiltHandler_;
};

#endif // WIDGETANTENNADATAMANIPULATOR_H
