#ifndef WIDGETANTENNADATAMANIPULATOR_H
#define WIDGETANTENNADATAMANIPULATOR_H

#include <QWidget>
#include <memory>
#include <pan_tilt_handler/PanTiltHandler.h>

namespace Ui
{
class WidgetAntennaDataManipulator;
}

class WidgetAntennaDataManipulator : public QWidget
{
    Q_OBJECT

public:
    explicit
    WidgetAntennaDataManipulator(QWidget* parent = 0);
    void
    connect(std::shared_ptr<PanTiltHandler> ah);
    ~WidgetAntennaDataManipulator();

private slots:
    void
    on_apply_clicked();

private:
    Ui::WidgetAntennaDataManipulator* ui;
    std::shared_ptr<PanTiltHandler> panTiltHandler_;
};

#endif // WIDGETANTENNADATAMANIPULATOR_H
