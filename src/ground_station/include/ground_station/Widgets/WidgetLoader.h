////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#ifndef WIDGETLOADER_H
#define WIDGETLOADER_H

#include <QWidget>
#include <QGridLayout>
#include <QTabWidget>
#include <functional>

namespace Ui
{
class WidgetLoader;
}

class LayoutGenerator;

class WidgetLoader: public QWidget
{
Q_OBJECT

public:

	using ChangeWidget = std::function<void (WidgetLoader*,const QString&)>;

    using ChangeLayout = std::function<void (WidgetLoader*,const QString&, int, int)>;

	explicit
	WidgetLoader(QWidget *parent = 0);
	void
    linkLayoutGenerator(const ChangeWidget& changeWidget, const ChangeLayout &changeLayout, const std::vector<std::string> &widgets);
	void
	setLayoutHandle(QGridLayout * inLayout);
	QGridLayout*
	getLayoutHandle();
	void
	setTabHandle(QTabWidget * inTab);
	QTabWidget*
	getTabHandle();
	void
	setTabIndex(int index);
	int
	getTabIndex();
	~WidgetLoader();

public slots:
	void
	on_widgetButton_clicked();
	void
	on_layoutButton_clicked();

private:
	void
	widgetSelected(WidgetLoader *wid, const QString &widget);
    ChangeWidget changeWidget_;
    ChangeLayout changeLayout_;
	Ui::WidgetLoader *ui;
	QGridLayout * layout;
	QTabWidget *tab;
	int index;
};

#endif // WIDGETLOADER_H
