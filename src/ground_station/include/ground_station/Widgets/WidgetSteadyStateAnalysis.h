#ifndef WIDGETSTEADYSTATEANALYSIS_H
#define WIDGETSTEADYSTATEANALYSIS_H

#include <string>
#include <QWidget>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/FlightAnalysis/StateAnalysis/Metrics.h>
#include <uavAP/FlightControl/Controller/PIDController/PIDMapping.h>
#include <radio_comm/pidstati.h>
#include <radio_comm/serialized_object.h>

#include "ui_WidgetSteadyStateAnalysis.h"
#include "ground_station/IWidgetInterface.h"
#include "ground_station/ConfigManager.h"
#include "ground_station/Widgets/PID/PIDCustomPlot.h"

namespace Ui
{
class WidgetSteadyStateAnalysis;
}

class WidgetSteadyStateAnalysis: public QWidget
{
Q_OBJECT

public:

	static constexpr auto widgetName = "steady_state_analysis";

	explicit
	WidgetSteadyStateAnalysis(QWidget *parent = 0);

	~WidgetSteadyStateAnalysis();

	static inline QWidget*
	createGSWidget(std::shared_ptr<IWidgetInterface> interface, QWidget* parent)
	{
		auto widget(new WidgetSteadyStateAnalysis(parent));
		widget->connectInterface(interface);
		return widget;
	}

public slots:

	void
	onInspectingMetrics(const SteadyStateMetrics& inspectingMetrics);

	void
	onOverride(const Override& override);

	void
	onPIDStati(const radio_comm::pidstati& stati);

private slots:

	void
	on_apply_clicked();

private:

	void
	connectInterface(std::shared_ptr<IWidgetInterface> interface);

	template<typename Enum>
	void
	updateInspectingMetrics(const std::string& groupString, const std::map<Enum, double>& override,
			const Enum& member);

	void
	resetMetrics();

	ObjectHandle<IConfigManager> configManagerHandle_;
	Ui::WidgetSteadyStateAnalysis *ui;
	std::shared_ptr<PIDCustomPlot> customPlot_;
	std::string inspectingMetricsString_;
	bool applied_;
	bool reset_;
};

template<typename Enum>
inline void
WidgetSteadyStateAnalysis::updateInspectingMetrics(const std::string& groupString,
		const std::map<Enum, double>& override, const Enum& member)
{
	std::string memberString = EnumMap<Enum>::convert(member);
	std::string groupMemberString = groupString + "/" + memberString;
	int index = ui->inspectingMetricsValue->findText(QString::fromStdString(groupMemberString));

	if (auto pair = findInMap(override, member))
	{
		if (index == -1)
		{
			ui->inspectingMetricsValue->addItem(QString::fromStdString(groupMemberString));
			applied_ = false;

			customPlot_->setTitle("");
			customPlot_->resetGraph();
			customPlot_->replot();

			resetMetrics();
		}
	}
	else
	{
		if (index != -1)
		{
			ui->inspectingMetricsValue->removeItem(index);
			applied_ = false;

			customPlot_->setTitle("");
			customPlot_->resetGraph();
			customPlot_->replot();

			resetMetrics();
		}
	}
}

#endif /* WIDGETSTEADYSTATEANALYSIS_H */
