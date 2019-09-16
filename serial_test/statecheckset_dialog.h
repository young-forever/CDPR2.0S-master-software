#ifndef STATECHECKSET_DIALOG_H
#define STATECHECKSET_DIALOG_H

#include <QDialog>

namespace Ui {
class statecheckset_dialog;
}

class statecheckset_dialog : public QDialog
{
    Q_OBJECT
    bool DriverOnlineCheckSwitch;
    bool ErrorOverCheckSwitch;
    bool CableOverTightCheckSwitch;
    bool CableOverLoseCheckSwitch;
    bool VelocityOverCheckSwitch;
    bool CurrentOverCheckSwitch;

public:
    explicit statecheckset_dialog(QWidget *parent = 0);
    ~statecheckset_dialog();

    void set_DriverOnlineCheckSwitch_from_ini(bool flag);
    void set_ErrorOverCheckSwitch_from_ini(bool flag);
    void set_CableOverTightCheckSwitch_from_ini(bool flag);
    void set_CableOverLoseCheckSwitch_from_ini(bool flag);
    void set_VelocityOverCheckSwitch_from_ini(bool flag);
    void set_CurrentOverCheckSwitch_from_ini(bool flag);

    bool get_DriverOnlineCheckSwitch(void);
    bool get_ErrorOverCheckSwitch(void);
    bool get_CableOverTightCheckSwitch(void);
    bool get_CableOverLoseCheckSwitch(void);
    bool get_VelocityOverCheckSwitch(void);
    bool get_CurrentOverCheckSwitch(void);

    void set_state_check_from_dialog(void);
    void set_dialog_state(void);

private slots:
    void on_StateCheckApply_pushButton_clicked();

private:
    Ui::statecheckset_dialog *ui;
};

#endif // STATECHECKSET_DIALOG_H
