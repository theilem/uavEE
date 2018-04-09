#ifndef WIDGETANTENNA_H
#define WIDGETANTENNA_H

#include <QWidget>
#include <memory>
#include "pan_tilt_handler/PanTiltHandler.h"
#include "pan_tilt_handler/Widgets/WidgetAntenna.h"
namespace Ui
{
class WidgetAntenna;
}

class WidgetAntenna : public QWidget
{
    Q_OBJECT

public:
    explicit
    WidgetAntenna(QWidget* parent = 0);
    void
    connect(std::shared_ptr<PanTiltHandler> interface);
    ~WidgetAntenna();

private:
    Ui::WidgetAntenna* ui;
};

#endif // WIDGETANTENNA_H
