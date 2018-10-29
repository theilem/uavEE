/*
 * WidgetLocalFrame.cpp
 *
 *  Created on: Jul 24, 2018
 *      Author: sim
 */

#include "ground_station/Widgets/WidgetLocalFrame.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IConfigManager.h"

WidgetLocalFrame::WidgetLocalFrame(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetLocalFrame)
{
	ui->setupUi(this);
	init_ = false;
}

WidgetLocalFrame::~WidgetLocalFrame()
{
}

void
WidgetLocalFrame::on_reset_clicked()
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	frame_ = frameOriginal_;
	copyLocalFrame(frame_);
	lock.unlock();
}

void
WidgetLocalFrame::on_copy_clicked()
{
	std::unique_lock<std::mutex> lock(frameMutex_);
	copyLocalFrame(frame_);
	lock.unlock();
}

void
WidgetLocalFrame::on_send_clicked()
{
	double originX;
	double originY;
	double originZ;
	double orientation;

	std::unique_lock<std::mutex> lock(frameMutex_);
	if (ui->originXValue->text().isEmpty())
	{
		originX = frame_.getOrigin().x();
	}
	else
	{
		originX = ui->originXValue->text().toDouble();
	}

	if (ui->originYValue->text().isEmpty())
	{
		originY = frame_.getOrigin().y();
	}
	else
	{
		originY = ui->originYValue->text().toDouble();
	}

	if (ui->originZValue->text().isEmpty())
	{
		originZ = frame_.getOrigin().z();
	}
	else
	{
		originZ = ui->originZValue->text().toDouble();
	}

	if (ui->orientationValue->text().isEmpty())
	{
		orientation = frame_.getYaw();
	}
	else
	{
		orientation = ui->orientationValue->text().toDouble() * M_PI / 180;
	}

	VehicleOneFrame frame(orientation, Vector3(originX, originY, originZ));

	configManager_.get()->sendLocalFrame(frame);
	frame_ = frame;
	lock.unlock();

	mapLogic_.get()->askForLocalFrame();
}

void
WidgetLocalFrame::onLocalFrame(const VehicleOneFrame& frame)
{
	QString string;

	std::unique_lock<std::mutex> lock(frameMutex_);
	string.sprintf("%10.5f", frame.getOrigin().x());
	string = string.simplified();
	string.replace(" ", "");
	ui->originXDisplay->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().y());
	string = string.simplified();
	string.replace(" ", "");
	ui->originYDisplay->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().z());
	string = string.simplified();
	string.replace(" ", "");
	ui->originZDisplay->setText(string);

	string.sprintf("%10.5f", frame.getYaw() * 180 / M_PI);
	string = string.simplified();
	string.replace(" ", "");
	ui->orientationDisplay->setText(string);

	frame_ = frame;

	if (!init_)
	{
		frameOriginal_ = frame;
		init_ = true;
	}
	lock.unlock();
}

void
WidgetLocalFrame::copyLocalFrame(const VehicleOneFrame& frame)
{
	QString string;

	string.sprintf("%10.5f", frame.getOrigin().x());
	string = string.simplified();
	string.replace(" ", "");
	ui->originXValue->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().y());
	string = string.simplified();
	string.replace(" ", "");
	ui->originYValue->setText(string);

	string.sprintf("%10.5f", frame.getOrigin().z());
	string = string.simplified();
	string.replace(" ", "");
	ui->originZValue->setText(string);

	string.sprintf("%10.5f", frame.getYaw() * 180 / M_PI);
	string = string.simplified();
	string.replace(" ", "");
	ui->orientationValue->setText(string);
}

void
WidgetLocalFrame::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Connect to Interface.";
		return;
	}

	if (!interface->getConfigManager().isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: ConfigManager Missing.";
		return;
	}

	configManager_.set(interface->getConfigManager().get());

	if (!configManager_.isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Get Config Manager.";
		return;
	}

	mapLogic_.set(interface->getMapLogic().get());

	if (!mapLogic_.isSet())
	{
		APLOG_ERROR << "WidgetLocalFrame: Cannot Get Map Logic.";
		return;
	}

	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)), this,
				SLOT(onLocalFrame(const VehicleOneFrame&)));
	}
	else
	{
		APLOG_ERROR << "WidgetLocalFrame: IDataSignals Missing.";
	}
}
