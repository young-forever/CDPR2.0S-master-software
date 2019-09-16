#include "statecheckset_dialog.h"
#include "ui_statecheckset_dialog.h"
#include "mainwindow.h"


statecheckset_dialog::statecheckset_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::statecheckset_dialog)
{
    ui->setupUi(this);
}

statecheckset_dialog::~statecheckset_dialog()
{
    delete ui;
}

void statecheckset_dialog::set_DriverOnlineCheckSwitch_from_ini(bool flag)
{
    DriverOnlineCheckSwitch = flag;
}

void statecheckset_dialog::set_ErrorOverCheckSwitch_from_ini(bool flag)
{
    ErrorOverCheckSwitch = flag;
}

void statecheckset_dialog::set_CableOverTightCheckSwitch_from_ini(bool flag)
{
    CableOverTightCheckSwitch = flag;
}

void statecheckset_dialog::set_CableOverLoseCheckSwitch_from_ini(bool flag)
{
    CableOverLoseCheckSwitch = flag;
}

void statecheckset_dialog::set_VelocityOverCheckSwitch_from_ini(bool flag)
{
    VelocityOverCheckSwitch = flag;
}

void statecheckset_dialog::set_CurrentOverCheckSwitch_from_ini(bool flag)
{
    CurrentOverCheckSwitch = flag;
}

bool statecheckset_dialog::get_DriverOnlineCheckSwitch(void)
{
    return DriverOnlineCheckSwitch;
}

bool statecheckset_dialog::get_ErrorOverCheckSwitch(void)
{
    return ErrorOverCheckSwitch;
}

bool statecheckset_dialog::get_CableOverTightCheckSwitch(void)
{
    return CableOverTightCheckSwitch;
}

bool statecheckset_dialog::get_CableOverLoseCheckSwitch(void)
{
    return CableOverLoseCheckSwitch;
}

bool statecheckset_dialog::get_VelocityOverCheckSwitch(void)
{
    return VelocityOverCheckSwitch;
}

bool statecheckset_dialog::get_CurrentOverCheckSwitch(void)
{
    return CurrentOverCheckSwitch;
}

void statecheckset_dialog::set_state_check_from_dialog(void)
{
    if(ui->DriverOnlieCheck_checkBox->isChecked())
    {
        DriverOnlineCheckSwitch = true;
    }
    else
    {
        DriverOnlineCheckSwitch = false;
    }

    if(ui->ErrorOverCheck_checkBox->isChecked())
    {
        ErrorOverCheckSwitch = true;
    }
    else
    {
        ErrorOverCheckSwitch = false;
    }

    if(ui->CableOverTightCheck_checkBox->isChecked())
    {
        CableOverTightCheckSwitch = true;
    }
    else
    {
        CableOverTightCheckSwitch = false;
    }

    if(ui->CableOverLoseCheck_checkBox->isChecked())
    {
        CableOverLoseCheckSwitch = true;
    }
    else
    {
        CableOverLoseCheckSwitch = false;
    }

    if(ui->VelocityOverCheck_checkBox->isChecked())
    {
        VelocityOverCheckSwitch = true;
    }
    else
    {
        VelocityOverCheckSwitch = false;
    }

    if(ui->CurrentOverCheck_checkBox->isChecked())
    {
        CurrentOverCheckSwitch = true;
    }
    else
    {
        CurrentOverCheckSwitch = false;
    }

    StateCheckBuff = (get_DriverOnlineCheckSwitch()) |\
                    (get_ErrorOverCheckSwitch() << 1) | \
                    (get_CableOverTightCheckSwitch() << 2) |\
                    (get_CableOverLoseCheckSwitch() << 3) |\
                    (get_VelocityOverCheckSwitch() << 4) |\
                    (get_CurrentOverCheckSwitch() << 5);
}

void statecheckset_dialog::set_dialog_state(void)
{
    if(DriverOnlineCheckSwitch == true)
    {
        ui->DriverOnlieCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->DriverOnlieCheck_checkBox->setCheckState(Qt::Unchecked);
    }
    if(ErrorOverCheckSwitch == true)
    {
        ui->ErrorOverCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->ErrorOverCheck_checkBox->setCheckState(Qt::Unchecked);
    }
    if(CableOverTightCheckSwitch == true)
    {
        ui->CableOverTightCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->CableOverTightCheck_checkBox->setCheckState(Qt::Unchecked);
    }
    if(CableOverLoseCheckSwitch == true)
    {
        ui->CableOverLoseCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->CableOverLoseCheck_checkBox->setCheckState(Qt::Unchecked);
    }
    if(VelocityOverCheckSwitch == true)
    {
        ui->VelocityOverCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->VelocityOverCheck_checkBox->setCheckState(Qt::Unchecked);
    }
    if(CurrentOverCheckSwitch == true)
    {
        ui->CurrentOverCheck_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->CurrentOverCheck_checkBox->setCheckState(Qt::Unchecked);
    }
}

void statecheckset_dialog::on_StateCheckApply_pushButton_clicked()
{
    set_state_check_from_dialog();
}
