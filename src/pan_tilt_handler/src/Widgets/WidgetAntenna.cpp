#include "pan_tilt_handler/Widgets/WidgetAntenna.h"
#include "pan_tilt_handler/Widgets/WidgetAntennaDataManipulator.h"
#include "ui_WidgetAntenna.h"
#include <uavAP/Core/Logging/APLogger.h>

#include <ground_station/Widgets/WidgetSensorData.h>
#include <ground_station/Widgets/PID/WidgetCPGrid.h>

WidgetAntenna::WidgetAntenna(QWidget* parent) :
    QWidget(parent),
    ui(new Ui::WidgetAntenna)
{
    ui->setupUi(this);
}

void
WidgetAntenna::connect(std::shared_ptr<PanTiltHandler> interface)
{
    if(interface->isIdle())
    {
        APLOG_WARN << "PanTilt is idle, cannot create WidgetAntenna";
        return;
    }
    auto gsInterface = interface->getInterface();
    ui->cpGrid_Layout->addWidget(dynamic_cast<WidgetCPGrid*>(WidgetCPGrid::createGSWidget(gsInterface, this)));
    WidgetAntennaDataManipulator* dataManipulator = new WidgetAntennaDataManipulator(this);
    dataManipulator->connect(interface);
    ui->verticalLayout->addWidget(dataManipulator);
    ui->verticalLayout->addWidget(dynamic_cast<WidgetSensorData*>(WidgetSensorData::createGSWidget(gsInterface, this)));
    //ui->cpgrid = dynamic_cast<WidgetCPGrid*>(WidgetCPGrid::createGSWidget(gsInterface, this));
    //ui->datamanipulator->connect(interface);
    //ui->sensordata = dynamic_cast<WidgetSensorData*>(WidgetSensorData::createGSWidget(gsInterface, this));
}

WidgetAntenna::~WidgetAntenna()
{
    delete ui;
}
