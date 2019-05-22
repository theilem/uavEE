#include <autopilot_interface/detail/UTMToLatLong.h>
#include <uavAP/Core/Logging/APLogger.h>
#include "pan_tilt_handler/Widgets/WidgetAntennaDataManipulator.h"
#include "ui_WidgetAntennaDataManipulator.h"

WidgetAntennaDataManipulator::WidgetAntennaDataManipulator(QWidget* parent) :
    QWidget(parent),
    ui(new Ui::WidgetAntennaDataManipulator)
{
    ui->setupUi(this);
    ui->latlon->setChecked(1);
    ui->GPSOverride->setChecked(1);
    ui->activateServos->setChecked(1);
    ui->pn->setText("40.0594");
    ui->pe->setText("-88.5514");
    ui->pd->setText("-170");
    ui->headingOffet->setText("90");
    ui->pitchOffset->setText("180");
}

void
WidgetAntennaDataManipulator::connect(std::shared_ptr<PanTiltHandler> ah)
{
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
    if(ui->GPSOverride->isChecked())
    {
        panTiltHandler_->overrideGPS_ = true;
        if(ui->latlon->isChecked())
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
    if(ui->targetOverride->isChecked())
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
