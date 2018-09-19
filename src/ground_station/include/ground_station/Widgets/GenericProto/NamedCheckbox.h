/*
 * NamedCheckbox.h
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDCHECKBOX_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDCHECKBOX_H_
#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QWidget>

class NamedCheckbox: public QWidget
{
	Q_OBJECT
public:

	explicit
	NamedCheckbox(const std::string& name, QWidget* parent = 0);

	~NamedCheckbox();

	bool
	get();

	void
	set(bool val);

private:

	QHBoxLayout layout_;

	QLabel* label_;
	QCheckBox* checkbox_;

};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDCHECKBOX_H_ */
