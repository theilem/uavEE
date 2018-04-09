#ifndef PANTILTHANDLER_H
#define PANTILTHANDLER_H
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/FlightControl/Controller/ControlElements/EvaluableControlElements.h>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include "PanTiltPIDCascade.h"
#include <ground_station/IDataSignals.h>
#include <ground_station/IPIDConfigurator.h>
#include <ground_station/IWidgetInterface.h>
#include <mutex>
#include <uavAP/Core/IDC/Serial/SerialIDC.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/SensorData.h>
#include "pan_tilt_handler/PanTiltHandlerWidgetInterface.h"

class PanTiltHandler: public IDataSignals,
    public IAggregatableObject,
    public IRunnableObject,
    public IPIDConfigurator
{
    Q_OBJECT
public:
    PanTiltHandler();

    static std::shared_ptr<PanTiltHandler>
    create(const boost::property_tree::ptree& config);

    bool
    configure(const boost::property_tree::ptree& config);

    void
    notifyAggregationOnUpdate(Aggregator& agg) override;

    bool
    run(RunStage stage) override;

    Vector2
    getAntennaLocation() const;

    double
    getAntennaTargetHeading() const;

    double
    getAntennaCurrentHeading() const;

    const PIDParametersMap&
    getPIDMap() const override;

    bool
    tunePID(const PIDTuning& tunePID) override;

    bool
    isIdle() const;

    std::shared_ptr<IWidgetInterface>
    getInterface() const;

signals:
    void
    onActuationData(const simulation_interface::actuation&) override;
    //void
    //onConfirmation(int) override;
    void
    onSensorData(const simulation_interface::sensor_data&) override;
    void
    onMission(const Mission&) override;
    void
    onTrajectory(const Trajectory&) override;
    void
    onPathSectionChange(int) override;
    void
    onLocalPlannerStatus(const LocalPlannerStatus&) override;
    void
    onPIDStati(const radio_comm::pidstati&) override;

public slots:
    void
    processAircraftSD(const simulation_interface::sensor_data& sd);

private:
    friend class WidgetAntennaDataManipulator;
    friend class IMUReader;
    bool idlemode_;
    bool overrideGPS_;
    bool overrideTarget_;
    double overrideHeading_;
    double overridePitch_;
    Vector3 GPSOverride_Position_;
    PanTiltData panTiltIMUData_;
    std::mutex panTiltIMUDataMutex_;

    simulation_interface::sensor_data aircraftSensorData_;
    std::shared_ptr<PanTiltPIDCascade> pidCascade_;

    std::string arduinoPath_;

    Sender sender_;
    ObjectHandle<IInterDeviceComm> idc_;
    ObjectHandle<IScheduler> scheduler_;
    std::shared_ptr<PanTiltHandlerWidgetInterface> widgetInterface_;
    PanTiltTarget target;
    PanTiltOutput output;
    bool sendOutput_;
    //void
    //updateSensorData();
    void
    calculateControl();
};

#endif // PANTILTHANDLER_H
