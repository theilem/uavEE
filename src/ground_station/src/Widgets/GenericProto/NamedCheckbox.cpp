/*
 * NamedCheckbox.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */
#include <ground_station/Widgets/GenericProto/NamedCheckbox.h>

NamedCheckbox::NamedCheckbox(const std::string& name, QWidget* parent) :
		QWidget(parent), layout_(this)
{
	label_ = new QLabel(QString::fromStdString(name), parent);
	checkbox_ = new QCheckBox(parent);

	layout_.addWidget(label_);
	layout_.addWidget(checkbox_);
}

NamedCheckbox::~NamedCheckbox()
{
	delete label_;
	delete checkbox_;
}

bool
NamedCheckbox::get()
{
	return checkbox_->isChecked();
}

void
NamedCheckbox::set(bool val)
{
	checkbox_->setChecked(val);
}
