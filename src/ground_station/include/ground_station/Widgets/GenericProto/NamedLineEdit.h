/*
 * LineEdit.h
 *
 *  Created on: Jul 31, 2018
 *      Author: mircot
 */

#ifndef GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDLINEEDIT_H_
#define GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDLINEEDIT_H_

#include <string>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QWidget>

class NamedLineEdit: public QWidget
{
Q_OBJECT
public:

	explicit
	NamedLineEdit(const std::string& name, QWidget* parent = 0);

	~NamedLineEdit();

	double
	getDouble();

	float
	getFloat();

	int
	getInt();

	std::string
	getString();

	bool
	isEmpty();

	void
	set(double val);

	void
	set(float val);

	void
	set(int val);

	void
	set(const std::string& val);

private:

	QHBoxLayout layout_;

	QLabel* label_;
	QLineEdit* edit_;

};

#endif /* GROUND_STATION_INCLUDE_GROUND_STATION_WIDGETS_GENERICPROTO_NAMEDLINEEDIT_H_ */
