#include "position_mode_set_dialog.h"
#include "ui_position_mode_set_dialog.h"

position_mode_set_dialog::position_mode_set_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::position_mode_set_dialog)
{
    ui->setupUi(this);

    for(short i=1;i<=8;i++)
    {
        Joint_mode_choose[i-1] = 0;
    }
    constant_force = 0.0;
    Line_MaxV = 0.04;            //m/s
    Line_MaxA = 0.02;

}

position_mode_set_dialog::~position_mode_set_dialog()
{
    delete ui;
}

void position_mode_set_dialog::set_joint_mode_from_ini(bool* joint_mode )
{
    for(short i=0;i<=8;i++)
    {
        Joint_mode_choose[i-1] = joint_mode[i-1];
    }

}

void position_mode_set_dialog::set_joint_mode_from_dialog(void)
{
    if(ui->Joint1_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[0] = 1;
    }
    else
    {
        Joint_mode_choose[0] = 0;
    }
    if(ui->Joint2_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[1] = 1;
    }
    else
    {
        Joint_mode_choose[1] = 0;
    }
    if(ui->Joint3_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[2] = 1;
    }
    else
    {
        Joint_mode_choose[2] = 0;
    }
    if(ui->Joint4_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[3] = 1;
    }
    else
    {
        Joint_mode_choose[3] = 0;
    }
    if(ui->Joint5_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[4] = 1;
    }
    else
    {
        Joint_mode_choose[4] = 0;
    }
    if(ui->Joint6_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[5] = 1;
    }
    else
    {
        Joint_mode_choose[5] = 0;
    }
    if(ui->Joint7_mode_choose_checkBox->isChecked())
    {
        Joint_mode_choose[6] = 1;
    }
    else
    {
        Joint_mode_choose[6] = 0;
    }
    if(ui->Joint8_mode_choose_checkBox->isChecked())
    {
         Joint_mode_choose[7] = 1;
    }
    else
    {
        Joint_mode_choose[7] = 0;
    }

}

void position_mode_set_dialog::get_joint_mode(bool *joint_mode)
{
    for(short i=1;i<=8;i++)
    {
        joint_mode[i-1] = Joint_mode_choose[i-1];
    }
}

void position_mode_set_dialog::set_constant_fore_from_dialog(void)
{
    constant_force = ui->Constant_force_set_lineEdit->text().toFloat();
}

void position_mode_set_dialog::set_constant_fore_from_ini(float force)
{
    constant_force = force;
}

float position_mode_set_dialog::get_constant_force(void)
{
    return constant_force;
}

void position_mode_set_dialog::set_dialog_state(void)
{
    if(Joint_mode_choose[0] == 1)
    {
        ui->Joint1_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint1_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[1] == 1)
    {
        ui->Joint2_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint2_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[2] == 1)
    {
        ui->Joint3_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint3_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[3] == 1)
    {
        ui->Joint4_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint4_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[4] == 1)
    {
        ui->Joint5_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint5_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[5] == 1)
    {
        ui->Joint6_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint6_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[6] == 1)
    {
        ui->Joint7_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint7_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }
    if(Joint_mode_choose[7] == 1)
    {
        ui->Joint8_mode_choose_checkBox->setCheckState(Qt::Checked);
    }
    else
    {
        ui->Joint8_mode_choose_checkBox->setCheckState(Qt::Unchecked);
    }

    ui->Constant_force_set_lineEdit->setText(QString::number(constant_force,'f',2));
    ui->a_max_set_lineEdit->setText(QString::number(Line_MaxA*1000.0,'f',2));
    ui->v_max_set_lineEdit->setText(QString::number(Line_MaxV*1000.0,'f',2));
}

void position_mode_set_dialog::setLine_MaxV(const float v_max)
{
    Line_MaxV = v_max;
}


float position_mode_set_dialog::getLine_MaxV(void)
{
    return Line_MaxV;
}

void position_mode_set_dialog::setLine_MaxA(const float a_max)
{
    Line_MaxA = a_max;
}

float position_mode_set_dialog::getLine_MaxA(void)
{
    return Line_MaxA;
}

void position_mode_set_dialog::on_position_mode_apply_pushButton_clicked()
{
    set_joint_mode_from_dialog();  

    constant_force = ui->Constant_force_set_lineEdit->text().toFloat();
    Line_MaxA = ui->a_max_set_lineEdit->text().toFloat()/1000.0;
    Line_MaxV = ui->v_max_set_lineEdit->text().toFloat()/1000.0;

}
