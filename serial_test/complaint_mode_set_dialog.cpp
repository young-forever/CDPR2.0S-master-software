#include "complaint_mode_set_dialog.h"
#include "ui_complaint_mode_set_dialog.h"
#include "mainwindow.h"

complaint_mode_set_dialog::complaint_mode_set_dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::complaint_mode_set_dialog)
{
    ui->setupUi(this);
}

complaint_mode_set_dialog::~complaint_mode_set_dialog()
{
    delete ui;
}

void complaint_mode_set_dialog::set_complaint_stiffness_from_ini(float *stiffness)
{
    Ks[0][0] = stiffness[0];
    Ks[1][1] = stiffness[1];
    Ks[2][2] = stiffness[2];
    Ks[3][3] = stiffness[3];
    Ks[4][4] = stiffness[4];
    Ks[5][5] = stiffness[5];
}

void complaint_mode_set_dialog::set_complaint_damp_from_ini(float *damp)
{
    Bs[0][0] = damp[0];
    Bs[1][1] = damp[1];
    Bs[2][2] = damp[2];
    Bs[3][3] = damp[3];
    Bs[4][4] = damp[4];
    Bs[5][5] = damp[5];
}

void complaint_mode_set_dialog::set_complaint_stiffness_from_dialog(void)
{
    Ks[0][0] = ui->stiffness_X_lineEdit->text().toFloat()*1000.0;
    Ks[1][1]  = ui->stiffness_Y_lineEdit->text().toFloat()*1000.0;
    Ks[2][2]  = ui->stiffness_Z_lineEdit->text().toFloat()*1000.0;
    Ks[3][3] = ui->stiffness_Rx_lineEdit->text().toFloat()/PI*360.0;
    Ks[4][4]  = ui->stiffness_Ry_lineEdit->text().toFloat()/PI*360.0;
    Ks[5][5]  = ui->stiffness_Rz_lineEdit->text().toFloat()/PI*360.0;
}


void complaint_mode_set_dialog::set_complaint_damp_from_dialog(void)
{
    Bs[0][0] = ui->damp_X_lineEdit->text().toFloat()*1000.0;
    Bs[1][1]  = ui->damp_Y_lineEdit->text().toFloat()*1000.0;
    Bs[2][2]  = ui->damp_Z_lineEdit->text().toFloat()*1000.0;
    Bs[3][3] = ui->damp_Rx_lineEdit->text().toFloat()/PI*360.0;
    Bs[4][4]  = ui->damp_Ry_lineEdit->text().toFloat()/PI*360.0;
    Bs[5][5]  = ui->damp_Rz_lineEdit->text().toFloat()/PI*360.0;
}

void complaint_mode_set_dialog::get_complaint_stiffness_from_dialog(float *stiffness)
{
    stiffness[0] = Ks[0][0];
    stiffness[1] = Ks[1][1];
    stiffness[2] = Ks[2][2];
    stiffness[3] = Ks[3][3];
    stiffness[4] = Ks[4][4];
    stiffness[5] = Ks[5][5];
}

void complaint_mode_set_dialog::get_complaint_damp_from_dialog(float *damp)
{
    damp[0] = Bs[0][0];
    damp[1] = Bs[1][1];
    damp[2] = Bs[2][2];
    damp[3] = Bs[3][3];
    damp[4] = Bs[4][4];
    damp[5] = Bs[5][5];
}


void complaint_mode_set_dialog::set_dialog_state(void)
{
    ui->stiffness_X_lineEdit->setText(QString::number( Ks[0][0]/1000.0,'f',3));
    ui->stiffness_Y_lineEdit->setText(QString::number( Ks[1][1]/1000.0,'f',3));
    ui->stiffness_Z_lineEdit->setText(QString::number( Ks[2][2]/1000.0,'f',3));
    ui->stiffness_Rx_lineEdit->setText(QString::number( Ks[3][3]*PI/360.0,'f',5));
    ui->stiffness_Ry_lineEdit->setText(QString::number( Ks[4][4]*PI/360.0,'f',5));
    ui->stiffness_Rz_lineEdit->setText(QString::number( Ks[5][5]*PI/360.0,'f',5));

    ui->damp_X_lineEdit->setText(QString::number( Bs[0][0]/1000.0,'f',3));
    ui->damp_Y_lineEdit->setText(QString::number( Bs[1][1]/1000.0,'f',3));
    ui->damp_Z_lineEdit->setText(QString::number( Bs[2][2]/1000.0,'f',3));
    ui->damp_Rx_lineEdit->setText(QString::number( Bs[3][3]*PI/360.0,'f',5));
    ui->damp_Ry_lineEdit->setText(QString::number( Bs[4][4]*PI/360.0,'f',5));
    ui->damp_Rz_lineEdit->setText(QString::number( Bs[5][5]*PI/360.0,'f',5));

}


void complaint_mode_set_dialog::on_stiffness_set_pushButton_clicked()
{
    set_complaint_stiffness_from_dialog();
    set_complaint_damp_from_dialog();
}
