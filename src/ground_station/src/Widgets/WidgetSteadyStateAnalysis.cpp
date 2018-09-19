#include <iostream>

#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/FlightAnalysis/StateAnalysis/Metrics.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDHandling.h>
#include <uavAP/MissionControl/ManeuverPlanner/Override.h>
#include <uavAP/Core/DataPresentation/BinarySerialization.hpp>

#include "ground_station/Widgets/WidgetSteadyStateAnalysis.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IConfigManager.h"

WidgetSteadyStateAnalysis::WidgetSteadyStateAnalysis(QWidget *parent) :
		QWidget(parent), ui(new Ui::WidgetSteadyStateAnalysis), applied_(false), reset_(false)
{
	ui->setupUi(this);
	ui->inspectingMetricsValue->addItem(QString::fromStdString("-select-"));

	customPlot_ = std::make_shared<PIDCustomPlot>(this, "");
	customPlot_->setMinimumHeight(200);
	ui->customPlot->addWidget(customPlot_.get(), 0);
}

WidgetSteadyStateAnalysis::~WidgetSteadyStateAnalysis()
{
	delete ui;
}

void
WidgetSteadyStateAnalysis::onInspectingMetrics(const SteadyStateMetrics& inspectingMetrics)
{
	if (!applied_)
	{
		return;
	}

	const double& target = inspectingMetrics.target;
	const double& value = inspectingMetrics.value;
	const double& tolerance = inspectingMetrics.tolerance;
	const double& overshoot = inspectingMetrics.overshoot;
	const double& riseTime = inspectingMetrics.riseTime;
	const double& settlingTime = inspectingMetrics.settlingTime;
	const bool& inTolerance = inspectingMetrics.inTolerance;
	const bool& inSteadyState = inspectingMetrics.inSteadyState;
	const bool& isReset = inspectingMetrics.isReset;
	const bool& crossedTarget = inspectingMetrics.crossedTarget;
	const bool& foundLastTarget = inspectingMetrics.foundLastTarget;
	const bool& foundRiseTime = inspectingMetrics.foundRiseTime;
	const bool& foundSettlingTime = inspectingMetrics.foundSettlingTime;

	if (isReset)
	{
		resetMetrics();
	}
	else
	{
		if (!reset_)
		{
			customPlot_->addData(value, target);
		}
		else
		{
			customPlot_->resetGraph();
			customPlot_->replot();

			reset_ = false;
		}

		ui->toleranceValue->setText(QString::number(tolerance));

		if (!foundLastTarget)
		{
			ui->overshootValue->setText("N/A");
		}
		else if (foundLastTarget && !crossedTarget)
		{
			ui->overshootValue->setText(QString::number(0) + " %");
		}
		else if (foundLastTarget && crossedTarget)
		{
			ui->overshootValue->setText(QString::number(overshoot) + " %");
		}

		if (!foundRiseTime)
		{
			ui->riseTimeValue->setText("Rising");
		}
		else
		{
			ui->riseTimeValue->setText(QString::number(riseTime) + " ms");
		}

		if (!foundRiseTime)
		{
			ui->settlingTimeValue->setText("Rising");
		}
		else if (foundRiseTime && !foundSettlingTime)
		{
			ui->settlingTimeValue->setText("Settling");
		}
		else if (foundSettlingTime)
		{
			ui->settlingTimeValue->setText(QString::number(settlingTime) + " ms");
		}

		if (inTolerance)
		{
			ui->inToleranceValue->setText("True");
		}
		else
		{
			ui->inToleranceValue->setText("False");
		}

		if (inSteadyState)
		{
			ui->inSteadyStateValue->setText("True");
		}
		else
		{
			ui->inSteadyStateValue->setText("False");
		}
	}

	update();
}

void
WidgetSteadyStateAnalysis::onOverride(const Override& override)
{
	std::string groupString;

	groupString = EnumMap<OverrideGroup>::convert(OverrideGroup::LOCAL_PLANNER);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::POSITION_X);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::POSITION_Y);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::POSITION_Z);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::VELOCITY);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::HEADING);
	updateInspectingMetrics<LocalPlannerTargets>(groupString, override.localPlanner,
			LocalPlannerTargets::CLIMB_RATE);

	groupString = EnumMap<OverrideGroup>::convert(OverrideGroup::CONTROLLER_TARGETS);
	updateInspectingMetrics<ControllerTargets>(groupString, override.controllerTarget,
			ControllerTargets::VELOCITY);
	updateInspectingMetrics<ControllerTargets>(groupString, override.controllerTarget,
			ControllerTargets::YAW_RATE);
	updateInspectingMetrics<ControllerTargets>(groupString, override.controllerTarget,
			ControllerTargets::CLIMB_ANGLE);

	groupString = EnumMap<OverrideGroup>::convert(OverrideGroup::PIDS);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::VELOCITY);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::VELOCITY_X);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::VELOCITY_Y);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::RUDDER);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::CLIMB_ANGLE);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::CLIMB_RATE);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::PITCH);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::PITCH_RATE);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::YAW_RATE);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::ROLL);
	updateInspectingMetrics<PIDs>(groupString, override.pid, PIDs::ROLL_RATE);
}

void
WidgetSteadyStateAnalysis::onPIDStati(const radio_comm::pidstati& stati)
{
	if (!applied_)
	{
		return;
	}
}

void
WidgetSteadyStateAnalysis::on_apply_clicked()
{
	if (ui->inspectingMetricsValue->currentIndex() == 0)
	{
		APLOG_WARN << "WidgetSteadyStateAnalysis: Inspecting Metrics Not Selected.";
		return;
	}

	auto configManager = configManagerHandle_.get();

	if (!configManager)
	{
		APLOG_ERROR << "WidgetSteadyStateAnalysis: Config Manager Missing.";
		return;
	}

	auto lastInspectingMetricsString = inspectingMetricsString_;
	inspectingMetricsString_ = ui->inspectingMetricsValue->currentText().toStdString();

	customPlot_->setTitle(inspectingMetricsString_);

	if (inspectingMetricsString_ != lastInspectingMetricsString)
	{
		reset_ = true;
	}

	InspectingMetricsPair pair;
	std::string metricsGroup;
	std::string metricsMember;
	std::istringstream ss(inspectingMetricsString_);

	if (!std::getline(ss, metricsGroup, '/') || !std::getline(ss, metricsMember, '/'))
	{
		APLOG_WARN << "WidgetSteadyStateAnalysis: Invalid Inspecting Metrics: "
				<< inspectingMetricsString_;
	}

	auto metricsGroupEnum = EnumMap<MetricsGroup>::convert(metricsGroup);

	switch (metricsGroupEnum)
	{
	case MetricsGroup::LOCAL_PLANNER:
	{
		auto metricsMemberEnum = EnumMap<LocalPlannerTargets>::convert(metricsMember);
		pair = std::make_pair<int, int>(static_cast<int>(metricsGroupEnum),
				static_cast<int>(metricsMemberEnum));
		break;
	}
	case MetricsGroup::CONTROLLER_TARGETS:
	{
		auto metricsMemberEnum = EnumMap<ControllerTargets>::convert(metricsMember);
		pair = std::make_pair<int, int>(static_cast<int>(metricsGroupEnum),
				static_cast<int>(metricsMemberEnum));
		break;
	}
	case MetricsGroup::PIDS:
	{
		auto metricsMemberEnum = EnumMap<PIDs>::convert(metricsMember);
		pair = std::make_pair<int, int>(static_cast<int>(metricsGroupEnum),
				static_cast<int>(metricsMemberEnum));
		break;
	}
	case MetricsGroup::INVALID:
	{
		APLOG_WARN << "WidgetSteadyStateAnalysis: Invalid Metrics Group: " << metricsGroup;
		break;
	}
	default:
	{
		APLOG_WARN << "WidgetSteadyStateAnalysis: Unknown Metrics Group: " << metricsGroup;
		break;
	}
	}

	configManager->sendInspectingMetrics(pair);

	applied_ = true;
}

void
WidgetSteadyStateAnalysis::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetSteadyStateAnalysis: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetSteadyStateAnalysis: Cannot Get Config Manager from Interface.";
		return;
	}

	configManagerHandle_.set(interface->getConfigManager().get());

	if (!configManagerHandle_.isSet())
	{
		APLOG_ERROR << "WidgetSteadyStateAnalysis: Cannot Get Config Manager.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onInspectingMetrics(const SteadyStateMetrics&)), this,
				SLOT(onInspectingMetrics(const SteadyStateMetrics&)));

		QObject::connect(ds.get(), SIGNAL(onOverride(const Override&)), this,
				SLOT(onOverride(const Override&)));

		QObject::connect(ds.get(), SIGNAL(onPIDStati(const radio_comm::pidstati&)), this,
				SLOT(onPIDStati(const radio_comm::pidstati&)));
	}
	else
	{
		APLOG_ERROR << "WidgetSteadyStateAnalysis: IDataSignals Missing.";
	}

	ui->inspectingMetricsValue->setCurrentIndex(0);
}

void
WidgetSteadyStateAnalysis::resetMetrics()
{
	ui->toleranceValue->setText("N/A");
	ui->overshootValue->setText("N/A");
	ui->riseTimeValue->setText("N/A");
	ui->settlingTimeValue->setText("N/A");
	ui->inToleranceValue->setText("N/A");
	ui->inSteadyStateValue->setText("N/A");
}
