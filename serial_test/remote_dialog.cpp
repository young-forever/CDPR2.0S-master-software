#include "remote_dialog.h"
#include "ui_remote_dialog.h"

Remote_Dialog::Remote_Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Remote_Dialog)
{
    ui->setupUi(this);
    X_UP_flag=0;
    X_DOWN_flag=0;
    Y_UP_flag=0;
    Y_DOWN_flag=0;
    Z_UP_flag=0;
    Z_DOWN_flag=0;
    remote_step=0;

    RemoteTrickInit();
    remote_step = ui->remote_step_Slider->value()/100.0;
}

Remote_Dialog::~Remote_Dialog()
{
    delete ui;
}

void Remote_Dialog:: RemoteTrickInit()       //slider initial
{
    int nMin = 0;
    int nMax = 300;        //qc
    int nSingleStep = 10;
    // 微调框
//    QSpinBox *pSpinBox = new QSpinBox(this);
    ui->remote_step_angle_spinBox->setMinimum(nMin);  // 最小值
    ui->remote_step_angle_spinBox->setMaximum(nMax);  // 最大值
    ui->remote_step_angle_spinBox->setSingleStep(nSingleStep);  // 步长
    // 滑动条
//    QSlider *pSlider = new QSlider(this);
    ui->remote_step_Slider->setOrientation(Qt::Horizontal);  // 水平方向
    ui->remote_step_Slider->setMinimum(nMin);  // 最小值
    ui->remote_step_Slider->setMaximum(nMax);  // 最大值
    ui->remote_step_Slider->setSingleStep(nSingleStep);  // 步长
    // 连接信号槽（相互改变）
    connect(ui->remote_step_angle_spinBox, SIGNAL(valueChanged(int)), ui->remote_step_Slider, SLOT(setValue(int)));
    connect(ui->remote_step_Slider, SIGNAL(valueChanged(int)), ui->remote_step_angle_spinBox, SLOT(setValue(int)));
    ui->remote_step_angle_spinBox->setValue(150);
}


void Remote_Dialog::on_remote_step_Slider_valueChanged(int value)
{
    remote_step = ui->remote_step_Slider->value()/100.0;//unit :mm
}

void Remote_Dialog::on_Y_UP_pushButton_pressed()
{
    Y_UP_flag=1;
}

void Remote_Dialog::on_Y_UP_pushButton_released()
{
    Y_UP_flag=0;
}

void Remote_Dialog::on_Y_DOWN_pushButton_pressed()
{
    Y_DOWN_flag=1;
}

void Remote_Dialog::on_Y_DOWN_pushButton_released()
{
    Y_DOWN_flag=0;
}

void Remote_Dialog::on_X_DOWN_pushButton_pressed()
{
    X_DOWN_flag=1;
}

void Remote_Dialog::on_X_DOWN_pushButton_released()
{
    X_DOWN_flag=0;
}

void Remote_Dialog::on_X_UP_pushButton_pressed()
{
    X_UP_flag=1;
}

void Remote_Dialog::on_X_UP_pushButton_released()
{
    X_UP_flag=0;
}

void Remote_Dialog::on_Z_UP_pushButton_pressed()
{
    Z_UP_flag=1;
}

void Remote_Dialog::on_Z_UP_pushButton_released()
{
    Z_UP_flag=0;
}

void Remote_Dialog::on_Z_DOWN_pushButton_pressed()
{
    Z_DOWN_flag=1;
}

void Remote_Dialog::on_Z_DOWN_pushButton_released()
{
    Z_DOWN_flag=0;
}
