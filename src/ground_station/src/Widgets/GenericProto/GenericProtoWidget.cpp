/*
 * GenericProtoWidget.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */
#include <ground_station/Widgets/GenericProto/GenericProtoWidget.h>
#include <ground_station/Widgets/GenericProto/NamedCheckbox.h>
#include <ground_station/Widgets/GenericProto/NamedLineEdit.h>
#include <uavAP/Core/Logging/APLogger.h>

GenericProtoWidget::GenericProtoWidget(QWidget* parent) :
		QGroupBox(parent), isSet_(false)
{
	setAutoFillBackground(true);
	setIsSet(false);
}

GenericProtoWidget::GenericProtoWidget(const std::string& name, QWidget* parent) :
		QGroupBox(parent), isSet_(false)
{
	setTitle(QString::fromStdString(name));
	setAutoFillBackground(true);
	setIsSet(false);
}

GenericProtoWidget::~GenericProtoWidget()
{
	clearLayout();
}

void
GenericProtoWidget::setLayoutAndPopulate(const google::protobuf::Message& message)
{
	setProtoLayout(message.GetDescriptor());
	fillLayout(message);
}

void
GenericProtoWidget::setProtoLayout(const google::protobuf::Descriptor* descriptor)
{
	clearLayout();
	delete layout();
	QVBoxLayout* layout = new QVBoxLayout;
	for (int i = 1; i <= descriptor->field_count(); ++i)
	{
		const auto* fDescriptor = descriptor->FindFieldByNumber(i);

		switch (fDescriptor->cpp_type())
		{
		case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
			layout->addWidget(
					addMessage(fDescriptor->lowercase_name(), fDescriptor->message_type()));
			break;
		case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
			layout->addWidget(addCheckbox(fDescriptor->lowercase_name()));
			break;
		default:
			layout->addWidget(addLineEdit(fDescriptor->lowercase_name()));
			break;
		}
	}

	setLayout(layout);
}

void
GenericProtoWidget::fillLayout(const google::protobuf::Message& message)
{
	const auto* descriptor = message.GetDescriptor();
	const auto* reflection = message.GetReflection();
	for (int i = 1; i <= descriptor->field_count(); ++i)
	{
		const auto* fDescriptor = descriptor->FindFieldByNumber(i);

		switch (fDescriptor->cpp_type())
		{
		case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
		{
			auto it = protoWidgets_.find(fDescriptor->lowercase_name());
			if (it == protoWidgets_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			if (reflection->HasField(message, fDescriptor))
				it->second->fillLayout(reflection->GetMessage(message, fDescriptor));
			break;
		}
		case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
		{
			auto it = checkBoxes_.find(fDescriptor->lowercase_name());
			if (it == checkBoxes_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			it->second->set(reflection->GetBool(message, fDescriptor));
			break;
		}
		default:
		{
			auto it = lineEdits_.find(fDescriptor->lowercase_name());
			if (it == lineEdits_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			switch (fDescriptor->cpp_type())
			{
			case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
				it->second->set(reflection->GetDouble(message, fDescriptor));
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
				it->second->set(reflection->GetFloat(message, fDescriptor));
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
				it->second->set(reflection->GetInt32(message, fDescriptor));
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
				it->second->set(static_cast<int>(reflection->GetInt64(message, fDescriptor)));
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
				it->second->set(reflection->GetString(message, fDescriptor));
				break;
			default:
				break;
			}
			break;
		}
		}
	}
}

void
GenericProtoWidget::populateMessage(google::protobuf::Message& message)
{
	setIsSet(true);
	const auto* descriptor = message.GetDescriptor();
	const auto* reflection = message.GetReflection();
	for (int i = 1; i <= descriptor->field_count(); ++i)
	{
		const auto* fDescriptor = descriptor->FindFieldByNumber(i);

		switch (fDescriptor->cpp_type())
		{
		case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
		{
			auto it = protoWidgets_.find(fDescriptor->lowercase_name());
			if (it == protoWidgets_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			if (it->second->isSet())
			{
				auto m = reflection->MutableMessage(&message, fDescriptor);
				it->second->populateMessage(*m);
			}
			break;
		}
		case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
		{
			auto it = checkBoxes_.find(fDescriptor->lowercase_name());
			if (it == checkBoxes_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			reflection->SetBool(&message, fDescriptor, it->second->get());
			break;
		}
		default:
		{
			auto it = lineEdits_.find(fDescriptor->lowercase_name());
			if (it == lineEdits_.end())
			{
				APLOG_ERROR << "Proto widget not properly initialized. Missing "
						<< fDescriptor->lowercase_name();
				break;
			}
			switch (fDescriptor->cpp_type())
			{
			case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
				reflection->SetDouble(&message, fDescriptor, it->second->getDouble());
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
				reflection->SetFloat(&message, fDescriptor, it->second->getFloat());
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
				reflection->SetInt32(&message, fDescriptor, it->second->getInt());
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
				reflection->SetInt64(&message, fDescriptor, it->second->getInt());
				break;
			case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
				reflection->SetString(&message, fDescriptor, it->second->getString());
				break;
			default:
				break;
			}
			break;
		}
		}
	}
}

QWidget*
GenericProtoWidget::addMessage(const std::string& name,
		const google::protobuf::Descriptor* descriptor)
{
	GenericProtoWidget* widget = new GenericProtoWidget(name, this);
	widget->setProtoLayout(descriptor);
	protoWidgets_.insert(std::make_pair(name, widget));
	return widget;
}

QWidget*
GenericProtoWidget::addCheckbox(const std::string& name)
{
	NamedCheckbox* widget = new NamedCheckbox(name, this);
	checkBoxes_.insert(std::make_pair(name, widget));
	return widget;
}

QWidget*
GenericProtoWidget::addLineEdit(const std::string& name)
{
	NamedLineEdit* widget = new NamedLineEdit(name, this);
	lineEdits_.insert(std::make_pair(name, widget));
	return widget;
}

bool
GenericProtoWidget::isSet()
{
	return isSet_;
}

void
GenericProtoWidget::mouseReleaseEvent(QMouseEvent* event)
{
	setIsSet(!isSet_);
}

void
GenericProtoWidget::setIsSet(bool isSet)
{
	isSet_ = isSet;
	QPalette p = palette();
	if (isSet)
		setStyleSheet("background-color: dark-grey");
	else
		setStyleSheet("background-color: grey");
}

void
GenericProtoWidget::clearLayout()
{
	for (auto& it : lineEdits_)
	{
		delete it.second;
	}

	for (auto& it : checkBoxes_)
	{
		delete it.second;
	}

	for (auto& it : protoWidgets_)
	{
		delete it.second;
	}

	lineEdits_.clear();
	checkBoxes_.clear();
	protoWidgets_.clear();
}
