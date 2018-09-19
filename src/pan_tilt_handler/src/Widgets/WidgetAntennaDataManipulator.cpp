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
	/*if(ah->isIdle()){
	 APLOG_WARN<<"PanTilt is idle, cannot create WidgetAntenna";
	 return;
	 }*/
	panTiltHandler_ = ah;
}

WidgetAntennaDataManipulator::~WidgetAntennaDataManipulator()
{
	delete ui;
}

void
WidgetAntennaDataManipulator::on_apply_clicked()
{
	std::lock_guard<std::mutex> lock(panTiltHandler_->panTiltIMUDataMutex_);
	if (ui->GPSOverride->isChecked())
	{
		panTiltHandler_->overrideGPS_ = true;
		if (ui->latlon->isChecked())
		{
			double utmN, utmE;
			int zone;
			//Vector2 loc = MapLocation::fromLatLong(ui->pn->text().toDouble(), ui->pe->text().toDouble());
			LLtoUTM(eWGS84, ui->pn->text().toDouble(), ui->pe->text().toDouble(), utmN, utmE, zone);
			panTiltHandler_->GPSOverride_Position_[0] = utmE;
			panTiltHandler_->GPSOverride_Position_[1] = utmN;
		}
		else
		{
			panTiltHandler_->GPSOverride_Position_[0] = ui->pe->text().toDouble();
			panTiltHandler_->GPSOverride_Position_[1] = ui->pn->text().toDouble();
		}
		panTiltHandler_->GPSOverride_Position_[2] = ui->pd->text().toDouble();
		panTiltHandler_->panTiltIMUData_.gpsData.position = panTiltHandler_->GPSOverride_Position_;
		panTiltHandler_->panTiltIMUData_.gpsData.velocity = Vector3();
	}
	else
	{
		panTiltHandler_->overrideGPS_ = false;
	}
	if (ui->targetOverride->isChecked())
	{
		panTiltHandler_->overrideTarget_ = true;
		panTiltHandler_->overrideHeading_ = ui->target_heading->text().toDouble() * M_PI / 180;
		panTiltHandler_->overridePitch_ = ui->target_pitch->text().toDouble() * M_PI / 180;
		panTiltHandler_->target.heading = panTiltHandler_->overrideHeading_;
		panTiltHandler_->target.pitch = panTiltHandler_->overridePitch_;
		panTiltHandler_->panTiltIMUData_.imuData.attitudeRate = Vector3();
	}
	else
	{
		panTiltHandler_->overrideTarget_ = false;
	}
	panTiltHandler_->target.headingoffset = ui->headingOffet->text().toDouble() * M_PI / 180;
	panTiltHandler_->target.pitchoffset = ui->pitchOffset->text().toDouble() * M_PI / 180;
	panTiltHandler_->sendOutput_ = ui->activateServos->isChecked();
}
