/*
 * GenericProtoWidget.h
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTOWIDGET_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTOWIDGET_H_

#include <map>
#include <string>

#include <google/protobuf/message.h>

#include <QLineEdit>
#include <QtWidgets>
#include <QWidget>
#include <QVBoxLayout>

class NamedLineEdit;
class NamedCheckbox;

class GenericProtoWidget: public QGroupBox
{
Q_OBJECT
public:

	explicit
	GenericProtoWidget(QWidget* parent = 0);

	GenericProtoWidget(const std::string& name, QWidget* parent = 0);

	~GenericProtoWidget();

	void
	setLayoutAndPopulate(const google::protobuf::Message& message);

	void
	setProtoLayout(const google::protobuf::Descriptor* descriptor);

	void
	fillLayout(const google::protobuf::Message& message);

	void
	populateMessage(google::protobuf::Message& message);

	bool
	isSet();

protected:

	void
	mouseReleaseEvent(QMouseEvent* event);

private:

	void
	setIsSet(bool isSet);

	void
	clearLayout();

	QWidget*
	addMessage(const std::string& name, const google::protobuf::Descriptor* descriptor);

	QWidget*
	addCheckbox(const std::string& name);

	QWidget*
	addLineEdit(const std::string& name);

	bool isSet_;

	std::map<std::string, NamedLineEdit*> lineEdits_;
	std::map<std::string, NamedCheckbox*> checkBoxes_;
	std::map<std::string, GenericProtoWidget*> protoWidgets_;

};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTOWIDGET_H_ */
