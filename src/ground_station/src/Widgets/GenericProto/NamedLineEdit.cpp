/*
 * LineEdit.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */
#include <ground_station/Widgets/GenericProto/NamedLineEdit.h>

NamedLineEdit::NamedLineEdit(const std::string& name, QWidget* parent):
QWidget(parent), layout_(this)
{
	label_ = new QLabel(QString::fromStdString(name), parent);
	edit_ = new QLineEdit(parent);

	if (parent)
	{
		label_->setFont(parent->font());
		edit_->setFont(parent->font());
	}

	layout_.addWidget(label_);
	layout_.addWidget(edit_);
	layout_.setMargin(2);
}

NamedLineEdit::~NamedLineEdit()
{
	delete edit_;
	delete label_;
}

double
NamedLineEdit::getDouble()
{
	return edit_->text().toDouble();
}

float
NamedLineEdit::getFloat()
{
	return edit_->text().toFloat();
}

int
NamedLineEdit::getInt()
{
	return edit_->text().toInt();
}

std::string
NamedLineEdit::getString()
{
	return edit_->text().toStdString();
}

void
NamedLineEdit::set(double val)
{
	QString string;
	string.setNum(val);
	edit_->setText(string);
}

void
NamedLineEdit::set(float val)
{
	QString string;
	string.setNum(val);
	edit_->setText(string);
}

void
NamedLineEdit::set(int val)
{
	QString string;
	string.setNum(val);
	edit_->setText(string);
}
void
NamedLineEdit::set(const std::string& val)
{
	edit_->text() = QString::fromStdString(val);
}

bool
NamedLineEdit::isEmpty()
{
	return edit_->text().isEmpty();
}
