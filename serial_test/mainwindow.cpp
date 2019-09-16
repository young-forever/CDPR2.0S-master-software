#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mycommand.h"
#include "DKofCDPR.h"     //Include direct kinematic function of CDPR.
#include "ParaofCDPR.h"   //Include para of CDPR.
#include "invsknmtc_cdpr.h"
#include"controlalgorithm.h"

#include<QMessageBox>
#include<QFileDialog>
#include<QDebug>
#include <QTimer>
#include<QSettings>
#include<QThread>

#include <iostream>

MovingMode_e MovingMode = StopMode_e;

using namespace std;

bool ThreadStartFlag=0;
bool AUTOBACKflag = 0;
bool TrjOKFlag = 0;

bool ForcePrintFlag = 0;
short ForcePrintID = 0;
char StateCheckBuff = 0X00;
double bp[4][8]={0.0};
double ep[4][8]={0.0};
double dia[8]={0.0};
double pulleyRad[8]={0.0};
float TtoV[8]={0.0};
float Dm[8] = {0.0};//diameter of measure wheel.
float L0[8] = {0.0};
float Dc = 0.0;
float E = 0.0;
float CDPR_Force_PID[8][3] = {0.0};
float Ks[6][6] = {0.0};
float Bs[6][6] = {{ 10.0,    0.0,    0.0,    0.0,    0.0,    0.0},
                  { 0.0,    10.0,    0.0,    0.0,    0.0,    0.0},
                  { 0.0,    0.0,    10.0,    0.0,    0.0,    0.0},
                  { 0.0,    0.0,    0.0,    10.0,    0.0,    0.0},
                  { 0.0,    0.0,    0.0,    0.0,    10.0,    0.0},
                  { 0.0,    0.0,    0.0,    0.0,    0.0,    10.0}};

float MinCalbleForce = 5;//unit N;

//display message flag.
bool ConnectOKDisplayFlag = 0;
bool PIDParaConfDisplayFlag = 1;
bool SerialConSucDisFlag = 1;
bool TtoVParaConfDisplayFlag = 1;
bool TtoV_zeroParaConfDisplayFlag = 1;
bool DiaParaConfDisplayFlag = 1;
bool DmParaConfDisplayFlag = 1;
bool DcParaConfDisplayFlag = 1;
bool L0ParaConfDisplayFlag = 1;
bool EParaConfDisplayFlag = 1;
bool ErrorDisplayFlag = 0;

bool ExpectAngleUpdateFlag = 1;
bool RealAngleUpdateFlag = 1;
bool ExpectPoseUpdateFlag = 1;
bool RealPoseUpdateFlag = 1;

bool TrjCompleteDisplayFlag = 1;
bool TeachingMovingFlag = false;
bool ComplainceMode = 0;
bool SglPoseComplaintMode = 0;

//when window was creat, do this:
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    serial_command_(),
    trjplan(),
    remote_dialog()
{
    UiInit();
    SerialInit();
    ParaReadInit();
    ZeroingInit();
    HandleTimeOutInit();
    LineNum = 1;

    TrjFileChooseFlag = 0;//already choose txt file.
    FileOpenFlag = 0;//txt file open flag.

    TrjPoseBuff = MatrixXf::Zero(6,20);
    m_thread = new ThreadFromQThread(this);
}

//when window was destroy, do this:
MainWindow::~MainWindow()
{
    //store end pose;
    QSettings *ParaConfigIniWrite = new QSettings("para.ini",QSettings::IniFormat);

    QString str;
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle1",str.number(ExpectAngle[0]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle2",str.number(ExpectAngle[1]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle3",str.number(ExpectAngle[2]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle4",str.number(ExpectAngle[3]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle5",str.number(ExpectAngle[4]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle6",str.number(ExpectAngle[5]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle7",str.number(ExpectAngle[6]));
    ParaConfigIniWrite->setValue("/para_ip/ExpectAngle8",str.number(ExpectAngle[7]));

    //    VectorXf ZeroAngle(8);
    //    ZeroAngle = trjplan.getAngleNodeZero();
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero1",str.number(ZeroAngle[0]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero2",str.number(ZeroAngle[1]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero3",str.number(ZeroAngle[2]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero4",str.number(ZeroAngle[3]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero5",str.number(ZeroAngle[4]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero6",str.number(ZeroAngle[5]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero7",str.number(ZeroAngle[6]));
    ParaConfigIniWrite->setValue("/para_ip/AngleNodeZero8",str.number(ZeroAngle[7]));


    bool joint_mode[8];
    my_position_mode_set_dialog.get_joint_mode(joint_mode);

    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_1",str.number(joint_mode[0]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_2",str.number(joint_mode[1]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_3",str.number(joint_mode[2]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_4",str.number(joint_mode[3]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_5",str.number(joint_mode[4]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_6",str.number(joint_mode[5]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_7",str.number(joint_mode[6]));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_8",str.number(joint_mode[7]));

    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_constant_force",str.number(my_position_mode_set_dialog.get_constant_force()));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_a_max",str.number(my_position_mode_set_dialog.getLine_MaxA()));
    ParaConfigIniWrite->setValue("/para_ip/PositionDialogState_v_max",str.number(my_position_mode_set_dialog.getLine_MaxV()));

    float stiffness[6];
    my_complaint_mode_set_dialog.get_complaint_stiffness_from_dialog(stiffness);
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_X",str.number(stiffness[0]));
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_Y",str.number(stiffness[1]));
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_Z",str.number(stiffness[2]));
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_Rx",str.number(stiffness[3]));
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_Ry",str.number(stiffness[4]));
    ParaConfigIniWrite->setValue("/para_ip/Stiffness_Rz",str.number(stiffness[5]));

    float damp[6];
    my_complaint_mode_set_dialog.get_complaint_damp_from_dialog(damp);
    ParaConfigIniWrite->setValue("/para_ip/Damp_X",str.number(damp[0]));
    ParaConfigIniWrite->setValue("/para_ip/Damp_Y",str.number(damp[1]));
    ParaConfigIniWrite->setValue("/para_ip/Damp_Z",str.number(damp[2]));
    ParaConfigIniWrite->setValue("/para_ip/Damp_Rx",str.number(damp[3]));
    ParaConfigIniWrite->setValue("/para_ip/Damp_Ry",str.number(damp[4]));
    ParaConfigIniWrite->setValue("/para_ip/Damp_Rz",str.number(damp[5]));


    ParaConfigIniWrite->setValue("/para_ip/check_set_DriverOnlineCheck",str.number(my_statecheckset_dialog.get_DriverOnlineCheckSwitch()));
    ParaConfigIniWrite->setValue("/para_ip/check_set_ErrorOverCheck",str.number(my_statecheckset_dialog.get_ErrorOverCheckSwitch()));
    ParaConfigIniWrite->setValue("/para_ip/check_set_CableOverTightCheck",str.number(my_statecheckset_dialog.get_CableOverTightCheckSwitch()));
    ParaConfigIniWrite->setValue("/para_ip/check_set_CableOverLoseCheck",str.number(my_statecheckset_dialog.get_CableOverLoseCheckSwitch()));
    ParaConfigIniWrite->setValue("/para_ip/check_set_VelocityOverCheck",str.number(my_statecheckset_dialog.get_VelocityOverCheckSwitch()));
    ParaConfigIniWrite->setValue("/para_ip/check_set_CurrentOverCheck",str.number(my_statecheckset_dialog.get_CurrentOverCheckSwitch()));

    delete ParaConfigIniWrite;

    m_thread->stopImmediately();//stop DK
    m_thread->wait();//waiting for stop of DK.
    delete ui;
    serial_command_.serialclose();
}


void MainWindow:: AllControlFlagInit()
{
    zeringOKflag = false;
    StartPointOKFlag = false;
    TrjMovingOKFlag = false;
    RemoteMovingOKFlag = false;

    MovingCompleteFlag = true;
    MovingCommandSendEnableFlag = false;
    InterpolationOKFlag = false;
    ZeroToStartTrjFlag = false;
    BackToZero = false;
    BackToStart = false;

    SglJointMovingFlag = false;

    TrjMovingFlag = false;//include posion and complaint mode.
    RemoteMovingFlag = false;
    AutobackTrjFlag = false;
    ZeroToStartMovingFlag = false;

    ErrorDisplayFlag = 0;
}

void MainWindow:: ZeroingInit()
{
    TrickInit(); //slider initial

    //initial  id
    ui->single_joint_mode_checkBox->setChecked(true);

    ui->radioButton_ID1->setChecked(true);
}

//control mode set.
void MainWindow:: ControlModeSet(const char mode)
{
    char sendmode = 0x00;
    if(mode == PositionModeSet)
    {
        //get joint mode
        bool joint_mode[8];
        my_position_mode_set_dialog.get_joint_mode(joint_mode);

        for(short i=0;i<8;i++)
        {
            if(joint_mode[7-i] == 1)
            {
                sendmode = sendmode | 0x01;
            }
            sendmode  = sendmode << 1;
        }
        serial_command_.my_command_ControlModeSet(sendmode);//
    }

    if(mode == ComplainceModeSet)
    {

        serial_command_.my_command_ControlModeSet(0xff);
    }
}

void MainWindow:: UiInit()
{
    ui->setupUi(this);

    ui->trjplan_groupBox->setEnabled(false);

    L0UpdateFlag = 0;

    AllControlFlagInit();

    SglJointDirFlag = 1;
}


void MainWindow:: SerialInit()
{
    connect(&serial_command_.serial_,SIGNAL(readyRead()),this,SLOT(my_serialread()));
    QList<QSerialPortInfo> infos = serial_command_.initSeialPort();

    if(infos.isEmpty())
    {
        ui->comboBox->addItem("无效");
        return;
    }
    ui->comboBox->addItem("串口");
    foreach (QSerialPortInfo info, infos) {
        ui->comboBox->addItem(info.portName());
    }
}

void MainWindow:: HandleTimeOutInit()
{
    m_pTimer = new QTimer(this);
    connect(m_pTimer, SIGNAL(timeout()), this, SLOT(handleTimeout()));//similar to timer interrupts.
    m_pTimer->start(TIMER_TIMEOUT);//set timing interval.

    //set task timing interval:
    diplayflag=0;
    controlflag=0;

    display_time = 50;
    display_time_reload=50;
    ThreadStartTime=10;
    ThreadStartTimeReload=10;

    PCtoFC_cnt = 5;
    control_time = trjplan.getPM_control_cycle()/PCtoFC_cnt;
    control_time_reload = control_time;
}


void MainWindow:: ParaReadInit()
{
    //read the expected angles that was stored end of last execute;
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);
    ExpectAngle[0] = ParaConfigIniRead->value("/para_ip/ExpectAngle1").toFloat();
    ExpectAngle[1] = ParaConfigIniRead->value("/para_ip/ExpectAngle2").toFloat();
    ExpectAngle[2] = ParaConfigIniRead->value("/para_ip/ExpectAngle3").toFloat();
    ExpectAngle[3] = ParaConfigIniRead->value("/para_ip/ExpectAngle4").toFloat();
    ExpectAngle[4] = ParaConfigIniRead->value("/para_ip/ExpectAngle5").toFloat();
    ExpectAngle[5] = ParaConfigIniRead->value("/para_ip/ExpectAngle6").toFloat();
    ExpectAngle[6] = ParaConfigIniRead->value("/para_ip/ExpectAngle7").toFloat();
    ExpectAngle[7] = ParaConfigIniRead->value("/para_ip/ExpectAngle8").toFloat();

    //    VectorXf ZeroAngle(8);
    ZeroAngle[0] = ParaConfigIniRead->value("/para_ip/AngleNodeZero1").toFloat();
    ZeroAngle[1] = ParaConfigIniRead->value("/para_ip/AngleNodeZero2").toFloat();
    ZeroAngle[2] = ParaConfigIniRead->value("/para_ip/AngleNodeZero3").toFloat();
    ZeroAngle[3] = ParaConfigIniRead->value("/para_ip/AngleNodeZero4").toFloat();
    ZeroAngle[4] = ParaConfigIniRead->value("/para_ip/AngleNodeZero5").toFloat();
    ZeroAngle[5] = ParaConfigIniRead->value("/para_ip/AngleNodeZero6").toFloat();
    ZeroAngle[6] = ParaConfigIniRead->value("/para_ip/AngleNodeZero7").toFloat();
    ZeroAngle[7] = ParaConfigIniRead->value("/para_ip/AngleNodeZero8").toFloat();
    //    trjplan.setAngleNodeZero(ZeroAngle);

    delete ParaConfigIniRead;

    //read all para from ini and write to edit.
    ReadStrucParaFromIni();
    ReadZeroPoseFromIni();
    //ReadPIDFromIni();//From para.ini;
    //ReadTtoVFromIni();

    WriteStrucParaToEdit();
    WriteZeroPoseToEdit();
    //WritePIDtoEdit();
    //WriteTtoVtoEdit();

    //dialog state read.
    ReadPositionDialogState();
    ReadComplaintDialogState();
    StateCheckSet_dialog();
}




void MainWindow:: TrickInit()       //slider initial
{
    int nMin = 0;
    int nMax = 1000;        //qc
    int nSingleStep = 50;
    // 微调框
    //    QSpinBox *pSpinBox = new QSpinBox(this);
    ui->step_angle_spinBox->setMinimum(nMin);  // 最小值
    ui->step_angle_spinBox->setMaximum(nMax);  // 最大值
    ui->step_angle_spinBox->setSingleStep(nSingleStep);  // 步长
    // 滑动条
    //    QSlider *pSlider = new QSlider(this);
    ui->step_qc_Slider->setOrientation(Qt::Horizontal);  // 水平方向
    ui->step_qc_Slider->setMinimum(nMin);  // 最小值
    ui->step_qc_Slider->setMaximum(nMax);  // 最大值
    ui->step_qc_Slider->setSingleStep(nSingleStep);  // 步长
    // 连接信号槽（相互改变）
    connect(ui->step_angle_spinBox, SIGNAL(valueChanged(int)), ui->step_qc_Slider, SLOT(setValue(int)));
    connect(ui->step_qc_Slider, SIGNAL(valueChanged(int)), ui->step_angle_spinBox, SLOT(setValue(int)));
    ui->step_angle_spinBox->setValue(300);
}

void MainWindow:: SlaveCompComm()
{
    while(serial_command_.getflag(serialconsucf) == 1)
    {
        ui->textEdit->append("Serial connected ok!");
    }


}

void MainWindow:: PIDParaConfig()
{
    for(short i=1;i<=8;i++)
    {
        serial_command_.my_command_kpset(i, CDPR_Force_PID[i-1][0]);

        serial_command_.my_command_kiset(i, CDPR_Force_PID[i-1][1]);

        serial_command_.my_command_kdset(i, CDPR_Force_PID[i-1][2]);
        QThread:: msleep(2);//waitting 10ms.
    }

    serial_command_.setflag(paraconfigf);
    //    ui->textEdit->append("Paraments config ok!");
}

void MainWindow::TtoVParaConfig(void)
{
    for(short i=1;i<=8;i++)
    {
        serial_command_.my_command_TtoVset(i,TtoV[i-1]);
    }
    serial_command_.setflag(paraconfigf);
}

void MainWindow::TtoV_zeroParaConfig(void)
{
    serial_command_.my_command_TtoV_zeroset();
    serial_command_.setflag(paraconfigf);
}

void MainWindow::DiaParaConfig(void)
{
    for(short i=1;i<=8;i++)
    {
        serial_command_.my_command_Diaset(i,dia[i-1]*1000.0);
    }
    serial_command_.setflag(paraconfigf);

}

void MainWindow::DmParaConfig(void)
{
    for(short i=1;i<=8;i++)
    {
        serial_command_.my_command_Dmset(i,Dm[i-1]*1000.0);
    }
    serial_command_.setflag(paraconfigf);

}

void MainWindow::L0ParaConfig(void)
{
    for(short i=0;i<8;i++)
    {
        L0[i] = dia[i]*ZeroAngle[i]/180.0f*3.1415926f/2.0f;

        serial_command_.my_command_L0set(i+1,L0[i]);
    }
    serial_command_.setflag(paraconfigf);
}

void MainWindow::DcParaConfig(void)
{
    serial_command_.my_command_Dcset(Dc*1000.0);
    serial_command_.setflag(paraconfigf);
}

void MainWindow::EParaConfig(void)
{
    serial_command_.my_command_Eset(E/(1E+9f));//pa transform to Gpa.
    serial_command_.setflag(paraconfigf);
}


//judg if on the zero point.
bool MainWindow::isEndOnP_Zero()
{
    double p_zero[6];
    short i=0;
    trjplan.getP_zero(p_zero);

    for(i=0;i<3;i++)
    {
        if(fabs(p_zero[i] - ExpectPose[i]) > 0.00002 )
        {
            return false;
        }
    }
    return true;
}

//judg if on the start point.
bool MainWindow::isEndOnP_Start()
{
    double p_start[6];
    short i=0;
    trjplan.getP_start(p_start);

    for(i=0;i<6;i++)
    {
        if(fabs(p_start[i] - ExpectPose[i]) > 0.00002 )
        {
            return false;
        }
    }
    return true;
}


//judg if the end near the zero point.
bool MainWindow::isNearP_Zero()
{
    double zeropose[6];
    static bool NearZeroMesShowFlag =0;

    trjplan.getP_zero(zeropose);

    if(fabs(ExpectPose[0] - zeropose[0]) >= 0.05 \
            || fabs(ExpectPose[1] - zeropose[1]) >= 0.05 \
            || fabs(ExpectPose[2] - zeropose[2]) >= 0.05 )//judge if end is nearly with zero pose.
    {
        return 0;
        NearZeroMesShowFlag = 0;
    }
    else
    {
        if(NearZeroMesShowFlag == 0)
        {
            NearZeroMesShowFlag = 1;
            QMessageBox message(QMessageBox::Warning,"Warning","Please keep the end away from zero pose!",QMessageBox::Yes,NULL);
            if(message.exec() == QMessageBox::Yes)
            {;}
        }
        return 1;
    }
}


//when combobox change, do this:
void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
    QSerialPortInfo info;
    QList<QSerialPortInfo> infos = serial_command_.updateSeialPort(arg1);

    int i = 0;
    foreach (info, infos) {
        if(info.portName() == arg1) break;
        i++;
    }
    if(i != infos.size ()){//can find
        ui->label->setText("[已开启]");

    }
    else
    {
        ui->label->setText("[SerialPort]");
    }
}


void MainWindow::my_serialread()
{
    serial_command_.serialRead();
}


void MainWindow::UI_refresh()
{
    static short cnt=0;

    //main window enable.
    if(serial_command_.getflag(serialconsucf)==0 && (ui->lockbutton->isEnabled()))
    {
        ui->lockbutton->setEnabled(false);
    }
    else if(!(ui->lockbutton->isEnabled()) && serial_command_.getflag(serialconsucf)==1)
    {
        ui->lockbutton->setEnabled(true);
    }


    if(StartPointOKFlag == true  && MovingCommandSendEnableFlag==false)//if not go to start point ever;
    {
        ui->controlmode_groupBox->setEnabled(true);
        if(isEndOnP_Start())
        {
            ui->positionmode_radioButton->setEnabled(true);
            ui->forcemode_radioButton->setEnabled(true);
            ui->teachingmode_radioButton->setEnabled(true);
            ui->remotemode_radioButton->setEnabled(true);
        }
        else
        {
            ui->positionmode_radioButton->setEnabled(false);
            ui->forcemode_radioButton->setEnabled(false);
            ui->teachingmode_radioButton->setEnabled(false);
            ui->remotemode_radioButton->setEnabled(true);
        }
    }
    else
    {
        ui->controlmode_groupBox->setEnabled(false);
    }

    //refresh run button text.
    if(MovingCommandSendEnableFlag == true)
    {
        ui->run_pushButton->setText("STOP");
    }
    else
    {
        ui->run_pushButton->setText("RUN");
    }


    //before
    //    if(StartPointOKFlag == false)//if not go to start point ever;
    //    {
    //        ui->controlmode_groupBox->setEnabled(false);
    //    }
    //    else if(StartPointOKFlag == true && TrjMovingFlag == 0 && RemoteMovingFlag == 0 && TeachingMovingFlag == 0)
    //    {
    //        ui->controlmode_groupBox->setEnabled(true);
    //    }
    //    else if(serial_command_.getState() == STOP && TrjOKFlag  == 1)
    //    {
    //        ui->controlmode_groupBox->setEnabled(true);
    //        ui->teachingmode_radioButton->setEnabled(false);
    //    }
    //    else if(MovingCommandSendEnableFlag == true)
    //    {
    //        ui->controlmode_groupBox->setEnabled(false);
    //    }


    if(cnt == 2 )
    {
        ui->Expect_pose_X_lineEdit->setText(QString::number(ExpectPose[0]*1000.0,'f',2));
        ui->Expect_pose_Y_lineEdit->setText(QString::number(ExpectPose[1]*1000.0,'f',2));
        ui->Expect_pose_Z_lineEdit->setText(QString::number(ExpectPose[2]*1000.0,'f',2));
        ui->Expect_pose_A_lineEdit->setText(QString::number(ExpectPose[3]*180.0/PI,'f',2));
        ui->Expect_pose_B_lineEdit->setText(QString::number(ExpectPose[4]*180.0/PI,'f',2));
        ui->Expect_pose_G_lineEdit->setText(QString::number(ExpectPose[5]*180.0/PI,'f',2));

        if(ComplainceMode == 1)
        {
            ui->Expect_End_Force_X_lineEdit->setText(QString::number(ExpectEndForce[0],'f',2));
            ui->Expect_End_Force_Y_lineEdit->setText(QString::number(ExpectEndForce[1],'f',2));
            ui->Expect_End_Force_Z_lineEdit->setText(QString::number(ExpectEndForce[2],'f',2));
            ui->Expect_End_Force_A_lineEdit->setText(QString::number(ExpectEndForce[3],'f',2));
            ui->Expect_End_Force_B_lineEdit->setText(QString::number(ExpectEndForce[4],'f',2));
            ui->Expect_End_Force_G_lineEdit->setText(QString::number(ExpectEndForce[5],'f',2));
        }
        else
        {
            ui->Expect_End_Force_X_lineEdit->setText(QString::number(0,'f',2));
            ui->Expect_End_Force_Y_lineEdit->setText(QString::number(0,'f',2));
            ui->Expect_End_Force_Z_lineEdit->setText(QString::number(0,'f',2));
            ui->Expect_End_Force_A_lineEdit->setText(QString::number(0,'f',2));
            ui->Expect_End_Force_B_lineEdit->setText(QString::number(0,'f',2));
            ui->Expect_End_Force_G_lineEdit->setText(QString::number(0,'f',2));

        }
        cnt = 0;
    }
    cnt++;

    if(serial_command_.getPIDRefreshFlag())
    {
        serial_command_.setPIDRefreshFlag(false);
        WritePIDtoEdit();
    }
    if(serial_command_.getTtoVRefreshFlag())
    {
        serial_command_.setTtoVRefreshFlag(false);
        WriteTtoVtoEdit();
    }
    if(serial_command_.getDiaRefreshFlag())
    {
        serial_command_.setDiaRefreshFlag(false);
        WriteDiatoEdit();
    }
    if(serial_command_.getDmRefreshFlag())
    {
        serial_command_.setDmRefreshFlag(false);
        WriteDmtoEdit();
    }
    if(serial_command_.getDcRefreshFlag())
    {
        serial_command_.setDcRefreshFlag(false);
        WriteDctoEdit();
    }
    if(serial_command_.getERefreshFlag())
    {
        serial_command_.setERefreshFlag(false);
        WriteEtoEdit();
    }

    //refresh real angle:
    ui->angle_joint1->setText(QString::number(RealAngle[0],'f',3));
    ui->angle_joint2->setText(QString::number(RealAngle[1],'f',3));
    ui->angle_joint3->setText(QString::number(RealAngle[2],'f',3));
    ui->angle_joint4->setText(QString::number(RealAngle[3],'f',3));
    ui->angle_joint5->setText(QString::number(RealAngle[4],'f',3));
    ui->angle_joint6->setText(QString::number(RealAngle[5],'f',3));
    ui->angle_joint7->setText(QString::number(RealAngle[6],'f',3));
    ui->angle_joint8->setText(QString::number(RealAngle[7],'f',3));

    //refresh real tesion:
    ui->tesion_joint1->setText(QString::number(RealTesion[0],'f',3));
    ui->tesion_joint2->setText(QString::number(RealTesion[1],'f',3));
    ui->tesion_joint3->setText(QString::number(RealTesion[2],'f',3));
    ui->tesion_joint4->setText(QString::number(RealTesion[3],'f',3));
    ui->tesion_joint5->setText(QString::number(RealTesion[4],'f',3));
    ui->tesion_joint6->setText(QString::number(RealTesion[5],'f',3));
    ui->tesion_joint7->setText(QString::number(RealTesion[6],'f',3));
    ui->tesion_joint8->setText(QString::number(RealTesion[7],'f',3));

    //refresh real Pose_ee
    ui->Pose_ee_x->setText(QString::number(RealPose[0]*1000.0,'f',3));
    ui->Pose_ee_y->setText(QString::number(RealPose[1]*1000.0,'f',3));
    ui->Pose_ee_z->setText(QString::number(RealPose[2]*1000.0,'f',3));
    ui->Pose_ee_A->setText(QString::number(RealPose[3]/PI*180.0,'f',3));
    ui->Pose_ee_B->setText(QString::number(RealPose[4]/PI*180.0,'f',3));
    ui->Pose_ee_G->setText(QString::number(RealPose[5]/PI*180.0,'f',3));

    ui->Force_x->setText(QString::number(RealEndForce[0],'f',2));
    ui->Force_y->setText(QString::number(RealEndForce[1],'f',2));
    ui->Force_z->setText(QString::number(RealEndForce[2],'f',2));
    ui->Force_A->setText(QString::number(RealEndForce[3],'f',2));
    ui->Force_B->setText(QString::number(RealEndForce[4],'f',2));
    ui->Force_G->setText(QString::number(RealEndForce[5],'f',2));

    //show expectangle
    if(ui->radioButton_ID1->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[0],'f',3));
    }
    else if(ui->radioButton_ID2->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[1],'f',3));
    }
    else if(ui->radioButton_ID3->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[2],'f',3));
    }
    else if(ui->radioButton_ID4->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[3],'f',3));
    }
    else if(ui->radioButton_ID5->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[4],'f',3));
    }
    else if(ui->radioButton_ID6->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[5],'f',3));
    }
    else if(ui->radioButton_ID7->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[6],'f',3));
    }
    else if(ui->radioButton_ID8->isChecked())
    {
        ui->sgl_mot_expect_anlge->setText(QString::number(ExpectAngle[7],'f',3));
    }
}

void MainWindow::MessageShow()
{
    //    if(SerialConSucDisFlag==0 && serial_command_.getflag(serialconsucf)==1)
    //    {
    //        SerialConSucDisFlag = 1;
    if(serial_command_.getflag(serialconsucf)==1 )
    {
        if(ConnectOKDisplayFlag == 0)
        {
            ui->textEdit->append("Serial connected ok!");
            ConnectOKDisplayFlag = 1;
        }
        //            serial_command_.resetflag(serialconsucf);
    }
    //        else
    //        {
    //            ConnectOKDisplayFlag = 0;
    //            ui->textEdit->append("WARNING:Serial disconnected!");
    //        }

    //        if(serial_command_.getflag(driveronlinef) == 1)
    //        {
    //            ui->textEdit->append("Motor drivers are all online.");
    //        }
    //        else
    //        {
    //            ui->textEdit->append("WARNING:Some motor drivers are not online.");
    //        }
    //    }

    if(serial_command_.getflag(getstartpositionf) == 1)
    {
        serial_command_.resetflag(getstartpositionf);
        ui->textEdit->append("Start position complete!");
    }


    //    if(PIDParaConfDisplayFlag == 0 && TtoVParaConfDisplayFlag==0 && serial_command_.getflag(paraconfigf)==1)
    //    {
    //        PIDParaConfDisplayFlag =1;
    //        TtoVParaConfDisplayFlag = 1;
    //        ui->textEdit->append("All Para config ok!");
    //    }


    if(PIDParaConfDisplayFlag == 0 && serial_command_.getflag(paraconfigf)==1)
    {
        PIDParaConfDisplayFlag =1;
        ui->textEdit->append("PID Para config ok!");
    }

    if(TtoVParaConfDisplayFlag == 0 && serial_command_.getflag(paraconfigf)==1)
    {
        TtoVParaConfDisplayFlag =1;
        ui->textEdit->append("TtoV Para config ok!");
    }

    if(TtoV_zeroParaConfDisplayFlag == 0)
    {
        TtoV_zeroParaConfDisplayFlag =1;
        ui->textEdit->append("TtoV_zero Para config ok!");
    }

    if(DiaParaConfDisplayFlag == 0)
    {
        DiaParaConfDisplayFlag =1;
        ui->textEdit->append("Dia Para config ok!");
    }

    if(DmParaConfDisplayFlag == 0)
    {
        DmParaConfDisplayFlag =1;
        ui->textEdit->append("Dm Para config ok!");
    }

    if(DcParaConfDisplayFlag == 0)
    {
        DcParaConfDisplayFlag =1;
        ui->textEdit->append("Dc Para config ok!");
    }

    if(EParaConfDisplayFlag == 0)
    {
        EParaConfDisplayFlag =1;
        ui->textEdit->append("E Para config ok!");
    }

    if(L0ParaConfDisplayFlag == 0)
    {
        L0ParaConfDisplayFlag =1;
        ui->textEdit->append("L0 Para config ok!");
    }

    if(TrjCompleteDisplayFlag == 0 && TrjMovingOKFlag == true)
    {
        TrjCompleteDisplayFlag = 1;
        ui->textEdit->append("All trj complete!");
    }

    //
    if(serial_command_.getErrorFlag() && ErrorDisplayFlag == 0)
    {
        ErrorDisplayFlag = 1;
        short id, errorcode;
        serial_command_.getErrorPara(id,errorcode);

        switch(errorcode)
        {
        case 0X01:
            cout << "Number "<<id<<" diver is missing!"<<endl;
            break;
        case 0X02:
            cout << "Number "<<id<<" motor angle error too big!"<<endl;
            break;
        case 0X03:
            cout << "Number "<<id<<" cable too tight!"<<endl;
            break;
        case 0X04:
            cout << "Number "<<id<<" cable too lose!"<<endl;
            break;
        case 0X05:
            cout << "Velocity of number "<<id<<" motor is too fast!"<<endl;
            break;
        case 0X06:
            cout << "Current of number "<<id<<" motor is too big!"<<endl;
            break;
        default:
            cout << "error code no match!!"<<endl;
            break;

        }
        QMessageBox::warning(this,tr("ERROR:"),tr("Please check CDPR!"));

        serial_command_.setErrorFlag(false);
    }

}

void MainWindow::on_lose_cable_pushButton_pressed()
{
    if(serial_command_.getState() == PREPARE)
    {
        //        SglJointMovingFlag = true;
        MovingMode = SglJointMode_e;
        SglJointDirFlag = 1;
    }

    else
    {    QMessageBox::warning(this,tr("Warning:"),tr("CDPR is running, please rest cdpr first!"));}
}

void MainWindow::on_tight_cable_pushButton_pressed()
{
    if(serial_command_.getState() == PREPARE)
    {
        //        SglJointMovingFlag = true;
        MovingMode = SglJointMode_e;
        SglJointDirFlag = -1;
    }

    else
    {    QMessageBox::warning(this,tr("Warning:"),tr("CDPR is running, please rest cdpr first!"));}
}

void MainWindow::on_lose_cable_pushButton_released()
{
    //    SglJointMovingFlag = false;
    MovingMode = StopMode_e;
}

void MainWindow::on_tight_cable_pushButton_released()
{
    //    SglJointMovingFlag = false;
    MovingMode = StopMode_e;
}

void MainWindow::on_ForceTight_pushButton_clicked()
{
    float Force_send;

    if(serial_command_.getState() == PREPARE)
    {
        Force_send = ui->step_qc_Slider->value()/10.0;
        if(Force_send > 30.0)//Force_send Max is 30N.
        {
            Force_send = 30.0;
        }
        short id = getRadioID();
        serial_command_.setMotCtrMode(1,id);
        serial_command_.my_command_forcecontrol_mode(id,Force_send);
    }
    else
    {
        QMessageBox::warning(this,tr("Warning:"),tr("CDPR is running, please reset cdpr first!"));
    }
}


void MainWindow::on_run_pushButton_clicked()
{
    if(zeringOKflag == true)//judge if CDPR been zeroed.
    {
        if(MovingCompleteFlag == true)
        {
            if(isEndOnP_Zero())
            {
                ZeroToStartTrjFlag = true;
                serial_command_.setState(RUNNING);
                MovingCompleteFlag = false;
                MovingMode = TrjMode_e;
            }
            else if(isEndOnP_Start())
            {
                if(ui->positionmode_radioButton->isChecked())
                {
                    if(TrjFileChooseFlag == true)
                    {
                        serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                        TrjMovingFlag = true;
                        ComplainceMode = false;
                        MovingCompleteFlag = false;
                        ControlModeSet(PositionModeSet);
                    }
                    else
                    {
                        QMessageBox::warning(this,tr("Warning:"),tr("Please choose TRJ file first!"));
                    }

                }
                else if(ui->forcemode_radioButton->isChecked())
                {
                    if(TrjFileChooseFlag == true)
                    {
                        serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                        TrjMovingFlag = true;
                        ComplainceMode = true;
                        MovingCompleteFlag = false;
                        ControlModeSet(ComplainceModeSet);
                    }
                    else
                    {
                        QMessageBox::warning(this,tr("Warning:"),tr("Please choose TRJ file first!"));
                    }
                }
                else if(ui->teachingmode_radioButton->isChecked())
                {
                    TeachingMovingFlag = true;
                    serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                    ComplainceMode = true;
                    MovingCompleteFlag = false;
                    MovingCommandSendEnableFlag = true;
                    ControlModeSet(ComplainceModeSet);
                }
                else if(ui->remotemode_radioButton->isChecked())
                {
                    RemoteMovingFlag = true;
                    serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                    ComplainceMode = false;

                    ControlModeSet(PositionModeSet);
                    remote_dialog.show();
                }
                else
                {
                    QMessageBox::warning(this,tr("Warning:"),tr("Please choose Running mode!"));
                }
            }
            else
            {
                if(ui->remotemode_radioButton->isChecked())
                {
                    RemoteMovingFlag = true;
                    serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                    ComplainceMode = false;
                    ControlModeSet(PositionModeSet);
                    remote_dialog.show();
                }
                else
                {
                    QMessageBox::warning(this,tr("Warning:"),tr("Please choose Running mode!"));
                }
            }
        }
        else
        {
            MovingCommandSendEnableFlag = !MovingCommandSendEnableFlag;
            if(MovingCommandSendEnableFlag == true)
            {
                MovingMode = TrjMode_e;
            }
            else
            {
                MovingMode = StopMode_e;
            }
        }
    }
    else
    {
        QMessageBox::warning(this,tr("Warning:"),tr("Please zering cdpr first!"));
    }



    //    //before
    //    if(zeringOKflag == true)//judge if CDPR been zeroed.
    //    {
    //        //if end is on zero, go to start;
    //        if(isEndOnP_Zero())
    //        {
    //            ZeroToStartFlag = true;
    //            ZeroToStartMovingFlag = true;
    //            LineNum = 1;
    //            serial_command_.setState(RUNNING);
    //        }

    //        if(isEndOnP_Start())
    //        {
    //            LineNum = 1;
    //        }

    //        //POSITION MODE:
    //        if(ui->positionmode_radioButton->isChecked() && TeachingMovingFlag == false && ZeroToStartFlag == false)
    //        {

    //            if(TrjMovingOKFlag == false && TrjMovingFlag == false && RemoteMovingFlag == false \
    //                    && RemoteMovingOKFlag == false && AutobackMovingFlag == false && TrjFileChooseFlag == true)
    //            {
    //                TrjMovingFlag = true;

    //                serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
    //                ControlModeSet(PositionModeSet);
    //            }
    //            else if(RemoteMovingFlag == true || RemoteMovingOKFlag ==true)
    //            {
    //                QMessageBox::warning(this,tr("Warning:"),tr("You have remmoted end, then can't move trj!"));
    //            }
    //            else if(TrjMovingOKFlag == true )
    //            {
    //                QMessageBox::warning(this,tr("Warning:"),tr("Trj complete already, can't move again!"));
    //            }
    //            else if(TrjFileChooseFlag == false)
    //            {
    //                QMessageBox::warning(this,tr("Warning:"),tr("Please choose TRJ file first!"));
    //            }
    //        }

    //        //FORCE MODE:
    //        if(ui->forcemode_radioButton->isChecked() && TeachingMovingFlag == false && ZeroToStartFlag == false)
    //        {
    //            if(ComplainceMode == 1)
    //            {
    //                if( TrjMovingFlag == false && TrjMovingOKFlag == false && RemoteMovingFlag == false && \
    //                        RemoteMovingOKFlag == false  && TrjFileChooseFlag == true )
    //                {
    //                    TrjMovingFlag = true;
    //                }
    //                else if(RemoteMovingFlag == true || TrjMovingOKFlag == true)
    //                {
    //                    RemoteMovingOKFlag = true;
    //                    RemoteMovingFlag = false;
    //                    SglPoseComplaintMode = true;
    //                }
    //                else if(TrjFileChooseFlag == false)
    //                {
    //                    QMessageBox::warning(this,tr("Warning:"),tr("Please choose TRJ file first!"));
    //                }
    //            }
    //            else
    //            {
    //                 QMessageBox::warning(this,tr("Warning:"),tr("Please set complaint mode first!"));
    //            }
    //        }

    //        //REMOTE MODE:
    //        if(ui->remotemode_radioButton->isChecked() && TeachingMovingFlag == false )
    //        {
    //            if(!remote_dialog.isActiveWindow())
    //            {
    //                remote_dialog.show();
    //            }

    //            if(RemoteMovingFlag == false)
    //            {
    //                RemoteMovingFlag = true;
    //                TrjMovingFlag = false;
    //            }
    //        }

    //        //TEACHING MODE:
    //        if(ui->teachingmode_radioButton->isChecked())
    //        {
    //            TeachingMovingFlag = true;
    //        }

    //        if(TrjMovingFlag == true || AutobackMovingFlag == true || ZeroToStartMovingFlag == true)
    //        {
    //            MovingCommandSendEnableFlag = !MovingCommandSendEnableFlag;
    //            if(MovingCommandSendEnableFlag == true)
    //            {
    //                serial_command_.my_command_stateset(RUNNING);
    //            }
    ////            else
    ////            {
    ////                serial_command_.my_command_stateset(STOP);
    ////            }
    //        }
    //    }
    //    else
    //    {QMessageBox::warning(this,tr("Warning:"),tr("Please zering cdpr first!"));}
}


void MainWindow::on_back_pushButton_clicked()
{
    if(MovingCommandSendEnableFlag==false)
    {
        if(AutobackTrjFlag)
        {
            QMessageBox::warning(this,tr("Warning:"),tr("AUTO BACK PROGRESS ING!!"));
        }
        else
        {
            if(ui->teachingmode_radioButton->isChecked() || ui->remotemode_radioButton->isChecked())
            {
                QMessageBox::warning(this,tr("Warning:"),tr("Running mode error!!"));
            }
            else
            {
                if(isEndOnP_Zero())
                {
                    QMessageBox::warning(this,tr("Warning:"),tr("Can't back anywhere!"));
                }
                else
                {
                    ZeroToStartTrjFlag = false;
                    TrjMovingFlag = false;
                    TeachingMovingFlag=false;
                    RemoteMovingFlag=false;
                    InterpolationOKFlag=false;
                    MovingMode = TrjMode_e;

                    AutobackTrjFlag=true;

                    serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
                    ComplainceMode = false;
                    ControlModeSet(PositionModeSet);

                    if(ZeroToStartTrjFlag == true || isEndOnP_Start())
                    {
                        BackToZero=true;
                    }
                    else
                    {
                        BackToStart=true;
                    }
                }
            }

        }
    }
    else
    {
        QMessageBox::warning(this,tr("Warning:"),tr("Please STOP cdpr first!"));
    }

    //before
    //    if(zeringOKflag == true)//judge if CDPR been zeroed.
    //    {
    //        if(MovingCommandSendEnableFlag == true)
    //        {
    //            QMessageBox::warning(this,tr("Warning:"),tr("Please STOP cdpr first!"));
    //        }

    //        else if(AutobackTrjFlag == true)
    //        {
    //            QMessageBox::warning(this,tr("Warning:"),tr("AUTO BACK PROGRESS ING!!"));
    //        }


    //        if(MovingCommandSendEnableFlag == false && AutobackTrjFlag == false)
    //        {
    //            TrjMovingFlag = false;
    //            TrjMovingOKFlag =false;
    //            InterpolationOKFlag = false;
    //            AutobackTrjFlag = true;
    //            if(ZeroToStartMovingFlag == false )
    //            {
    //                if(isEndOnP_Start())
    //                {
    //                    BackToZero = true;
    //                }
    //                else if(isEndOnP_Zero())
    //                {
    //                    QMessageBox::warning(this,tr("Warning:"),tr("Can't back anywhere!"));
    //                    AutobackTrjFlag = false;
    //                }
    //                else
    //                {
    //                    BackToStart = true;
    //                }
    //            }
    //            else
    //            {
    //                BackToZero = true;
    //            }
    //        }
    //    }
}

bool MainWindow::GetTrjFromFileToBuff()
{
    QFile file(TrjFileName.toStdString().data());
    int line_num = 0;
    QString str;
    QByteArray line;
    double temppose[6];

    file.open(QIODevice::ReadOnly);

    while(!(line_num==LineNum))
    {
        line = file.readLine();
        str = line.data();
        line_num++;
    }

    if(!str.compare("END\n"))
    {
        return 0;
    }

    if(str.at(0) == 'L')
    {
        TrjType = "L";
        PutPoseToBuff(ExpectPose,1);//put current pose to buff;

        TrjRelolver(str,temppose);

        PutPoseToBuff(temppose,2);//put current pose to buff;
        //        cout <<"temppose "<<":"
        //            <<temppose[0]<<"   "
        //            <<temppose[1]<<"   "
        //            <<temppose[2]<<"   "
        //            <<temppose[3]<<"   "
        //            <<temppose[4]<<"   "
        //            <<temppose[5]<<endl;
    }
    else if(str.at(0) == "C")
    {
        TrjType = "C";
        PutPoseToBuff(ExpectPose,1);//circle pose 1
        TrjRelolver(str,temppose);
        PutPoseToBuff(temppose,2);//circle pose 1

        line = file.readLine();
        str = line.data();
        LineNum++;

        TrjRelolver(str,temppose);
        PutPoseToBuff(temppose,3);//circle pose 3.
        LineNum++;
    }
    else if(str.at(0) == "FC")
    {
        TrjType = "FC";
        PutPoseToBuff(ExpectPose,1);//full circle pose 1
        TrjRelolver(str,temppose);
        PutPoseToBuff(ExpectPose,2);//full circle pose 2
    }
    else if(str.at(0) == "S")
    {
        TrjType = "S";
    }
    else
    {
        if(TrjType == "L")
        {
            PutPoseToBuff(ExpectPose,1);//full circle pose 1
            TrjRelolver(str,temppose);
            PutPoseToBuff(ExpectPose,2);//full circle pose 2
        }
        else
        {
            return 0;
            qDebug()<<"Trj code error!!"<<endl;
        }
    }

    file.close();
    LineNum++;
    return true;
}


void MainWindow::TrjRelolver(const QString str, double* temppose)
{
    short i = 0;
    char str_X[10],str_Y[10],str_Z[10],str_A[10],str_B[10],str_G[10];
    QString X,Y,Z,A,B,G;
    short n_x,n_y,n_z,n_a,n_b,n_g;

    while(i < str.length())
    {
        if(str.at(i)=='X')
        {
            i++;
            n_x=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.') | (str.at(i) == '-'))
            {
                if((n_x==0 && str.at(i) == '.') | (n_x!=0 && str.at(i) == '-'))
                {
                    //error operate.
                }
                str_X[n_x] = str.at(i).toLatin1();
                n_x++;
                i++;
            }
            str_X[n_x] = '\0';
            X = QString(QLatin1String(str_X));
            temppose[0] = X.toDouble()/1000.0;
        }

        if(str.at(i)=='Y')
        {
            i++;
            n_y=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.')| (str.at(i) == '-'))
            {
                str_Y[n_y] = str.at(i).toLatin1();
                n_y++;
                i++;
            }
            str_Y[n_y] = '\0';
            Y = QString(QLatin1String(str_Y));
            temppose[1] = Y.toDouble()/1000.0;
        }

        if(str.at(i)=='Z')
        {
            i++;
            n_z=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.') | (str.at(i) == '-'))
            {
                str_Z[n_z] = str.at(i).toLatin1();
                n_z++;
                i++;
            }
            str_Z[n_z] = '\0';

            Z = QString(QLatin1String(str_Z));
            temppose[2] = Z.toDouble()/1000.0;
        }

        if(str.at(i)=='A')
        {
            i++;
            n_a=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.') | (str.at(i) == '-'))
            {
                str_A[n_a] = str.at(i).toLatin1();
                n_a++;
                i++;
            }
            str_A[n_a] = '\0';

            A = QString(QLatin1String(str_A));
            temppose[3] = A.toDouble()/180.0*PI;
        }

        if(str.at(i)=='B')
        {
            i++;
            n_b=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.') | (str.at(i) == '-'))
            {
                str_B[n_b] = str.at(i).toLatin1();
                n_b++;
                i++;
            }
            str_B[n_b] = '\0';
            B = QString(QLatin1String(str_B));
            temppose[4] = B.toDouble()/180.0*PI;
        }

        if(str.at(i)=='G')
        {
            i++;
            n_g=0;
            while((str.at(i)<='9' && str.at(i)>='0') | (str.at(i) == '.') | (str.at(i) == '-'))
            {
                str_G[n_g] = str.at(i).toLatin1();
                n_g++;
                i++;
            }
            str_G[n_g] = '\0';
            G =  QString(QLatin1String(str_G));
            temppose[5] = G.toDouble()/180.0*PI;
        }
        i++;
    }
}

// Put number col pose to TrjPoseBuff;
void MainWindow::PutPoseToBuff(double* pose, const short col)
{
    for(short i=0;i<6;i++)
    {
        TrjPoseBuff(i,col-1) = pose[i];
    }
}

//void MainWindow::M_MatrixCal(char InterType, float (&p0)[6], float (&pt)[6], float v_max, float a_max)
void MainWindow::M_MatrixCal(char InterType, MatrixXf PoseBuff, float v_max, float a_max)
{
    int Max_num=0, Data_num_i=0,Data_cols, controlcycle;
    MatrixXd angle_node;
    MatrixXd M_matrix;
    VectorXf t_array;

    if(InterType == 'L')//line trj interpolation
    {
        float p0[6],pt[6];
        for(short i = 0;i<6;i++)
        {
            p0[i] = PoseBuff(i,0);
            pt[i] = PoseBuff(i,1);
        }
        //** Calculate PoseNode and Tplan. **//
        trjplan.PoseNodeCal(p0,pt,v_max,a_max);
    }
    else if(InterType == 'C')//circle trj interpolation
    {
        float ps[6],pc[6],pe[6];
        for(short i = 0;i<6;i++)
        {
            ps[i] = PoseBuff(i,0);
            pc[i] = PoseBuff(i,1);
            pe[i] = PoseBuff(i,2);
        }

    }
    else if(InterType == 'F')//Full circle trj interpolation
    {


    }
    else if(InterType == 'S')//spline trj interpolation
    {


    }

    //** INVERSE KINEMATIC **//
    MatrixXf pose_ee = trjplan.getPoseNode();//get pose node.
    Data_cols=pose_ee.cols();  //get number of pose node.

    MatrixXd ibp(4,8);
    MatrixXd iep(4,8);
    for(int i=0;i<4;i++)
    {
        Map <VectorXd> v_bp(bp[i],8);
        Map <VectorXd> v_ep(ep[i],8);
        ibp.row(i) = v_bp;
        iep.row(i) = v_ep;
    }

    Map<VectorXd> idia(dia,8);

    angle_node.resize(8,Data_cols);//define temp angle_node;
    VectorXd p_ee(6);               //define single pose node p_ee;
    VectorXd angle_node_col(8);     //define angle_node_col store inverse result;
    for(int i=0;i<Data_cols;i++)
    {
        float XX=pose_ee(0,i);
        float YY=pose_ee(1,i);
        float ZZ=pose_ee(2,i);
        float AX=pose_ee(3,i);
        float AY=pose_ee(4,i);
        float AZ=pose_ee(5,i);
        p_ee<<XX,YY,ZZ,AX,AY,AZ;

        //inverse kinematic.
        angle_node_col = InvsKnmtc_cdpr(p_ee,ibp,iep,idia,pulleyRad)/3.1415926*180.0;
        angle_node.col(i) = angle_node_col;
    }

    //** CALCULATE REALATIVE JOINT ANGEL OF ZERO POSITION **//
    VectorXd AngleNodeZero(8);

    for(short i=0;i<8;i++)
    {
        AngleNodeZero(i) = ZeroAngle[i];
    }
    for(int i=0;i<Data_cols;i++)
    {
        angle_node.col(i)=angle_node.col(i)-AngleNodeZero;
        //        cout<<"angle_node:"<<angle_node(3,i)<<endl;
    }
    trjplan.setAngleNode(angle_node);

    //** GET MAX_NUM**//
    t_array = trjplan.getTplan();
    controlcycle=trjplan.getPM_control_cycle();

    Max_num = ceil((t_array(Data_cols-1)-t_array(0))/((float)controlcycle/1000.0));
    trjplan.setMaxDataNum(Max_num);

    Data_num_i = 0;
    trjplan.setDataNum_i(Data_num_i);

    //** CALCULATE M_MATRIX **//
    M_matrix.resize(8,Data_cols);
    for(int i=0;i<8;i++)
    {
        M_matrix.row(i) = spline3ToM( Data_cols,angle_node.row(i),t_array,0.0,0.0);

        //        cout <<"The num " <<i<<" row value of M_Matrix..."<<endl;
        //        cout <<M_matrix.row(i)<<endl;
        //        cout <<" "<<endl;
    }
    trjplan.setM_Matrix(M_matrix);

    InterpolationOKFlag = true;
    MovingCommandSendEnableFlag = true;
}

void MainWindow::ZeroToStartMode()
{

}

short MainWindow::getRadioID(void)
{
    short id;
    if(ui->radioButton_ID1->isChecked())
    {
        id = 1;
    }
    else if(ui->radioButton_ID2->isChecked())
    {
        id = 2;
    }
    else if(ui->radioButton_ID3->isChecked())
    {
        id = 3;
    }
    else if(ui->radioButton_ID4->isChecked())
    {
        id = 4;
    }
    else if(ui->radioButton_ID5->isChecked())
    {
        id = 5;
    }
    else if(ui->radioButton_ID6->isChecked())
    {
        id = 6;
    }
    else if(ui->radioButton_ID7->isChecked())
    {
        id = 7;
    }
    else if(ui->radioButton_ID8->isChecked())
    {
        id = 8;
    }
    return id;
}

void MainWindow::SglJointMode()
{
    static short cnt=1;
    if(cnt == 1)
    {
        short id = getRadioID();
        if(serial_command_.getMotCtrMode(id))
        {
            ExpectAngle[id-1] = 0.0;
        }
        serial_command_.setMotCtrMode(0,id);
        ExpectAngle[id-1] = ExpectAngle[id-1]+SglJointDirFlag*(ui->step_qc_Slider->value()/100.0);
        serial_command_.my_command_positoncontrol_mode(id,ExpectAngle[id-1]);

        cnt=0;
    }
    cnt++;
}

void MainWindow::AutoBackMode()
{
    if(!InterpolationOKFlag)
    {

    }
}


void MainWindow::PositionControlMode()
{
    if(TrjMovingFlag == true && !InterpolationOKFlag)
    {
        bool trjOKflag = GetTrjFromFileToBuff();
        if(trjOKflag==0)
        {
            TrjMovingFlag = false;
            TrjMovingOKFlag = true;
        }
        else
        {
            float v_max,a_max;

            v_max = my_position_mode_set_dialog.getLine_MaxV();
            a_max = my_position_mode_set_dialog.getLine_MaxA();

            M_MatrixCal(TrjType.at(0).toLatin1(),TrjPoseBuff,v_max,a_max);
        }
    }
}

void  MainWindow::ComplaintControlMode()
{
    if(TrjMovingFlag == true && !InterpolationOKFlag)
    {

    }
}

void MainWindow::TrjExpectAngleCal()
{
    static short cnt = PCtoFC_cnt;
    if(serial_command_.getControlModeSetOkFlag() == true)
    {
        if(cnt == PCtoFC_cnt)
        {
            cnt=0;
            if(trjplan.getDataNum_i() <= trjplan.getMaxDataNum())//current trj not moving complete.
            {
                MatrixXf pose_ee = trjplan.getPoseNode();//get pose node.
                int Data_cols=pose_ee.cols();                       //get number of pose node.
                VectorXf t_array = trjplan.getTplan();
                int controlcycle=trjplan.getPM_control_cycle();
                int Data_num_i = trjplan.getDataNum_i();
                MatrixXd M_matrix = trjplan.getM_Matrix();
                MatrixXd angle_node = trjplan.getAngleNode();

                for(short i=0;i<8;i++)
                {
                    // calculate velocity and posion message.  //
                    Vector2f pmsg= Calculate_MotVand_Angle(i+1,Data_cols, (float)controlcycle/1000.0f,Data_num_i,\
                                                           t_array, M_matrix,angle_node);
                    ExpectAngle[i] = pmsg(1);  //set new expect angel.
                    if(i == 7)
                    {
                        ExpectAngleUpdateFlag = true;
                    }
                }

                trjplan.setDataNum_i(trjplan.getDataNum_i()+1);
            }
            else// send finished
            {
                cout<<"1111"<<endl;
                MovingCommandSendEnableFlag = false;//close send flag;
                InterpolationOKFlag = false;//set interpolation flag to false.

                if(ZeroToStartTrjFlag==true)
                {
                    StartPointOKFlag = true;
                    MovingCompleteFlag=true;
                    ZeroToStartTrjFlag = false;
                }
                else if(AutobackTrjFlag==true)
                {
                    MovingCompleteFlag=true;
                    AutobackTrjFlag = false;
                    BackToStart = false;
                    BackToZero = false;
                }
            }
        }
        cnt++;
    }

}

void MainWindow::TrjControlMode()
{
    double p_start[6],p_end[6];
    float v_max,a_max;

    v_max = my_position_mode_set_dialog.getLine_MaxV();
    a_max = my_position_mode_set_dialog.getLine_MaxA();

    if(ZeroToStartTrjFlag == true)
    {
        trjplan.getP_zero(p_start);
        trjplan.getP_start(p_end);
        PutPoseToBuff(p_start,1);
        PutPoseToBuff(p_end,2);

        M_MatrixCal('L',TrjPoseBuff,v_max,a_max);
    }
    else if(TrjMovingFlag == true)
    {
        bool trjOKflag = GetTrjFromFileToBuff();
        if(trjOKflag==0)
        {
            TrjMovingFlag = false;
            //            TrjMovingOKFlag = true;
            MovingCompleteFlag = true;
        }
        else
        {
            M_MatrixCal(TrjType.at(0).toLatin1(),TrjPoseBuff,v_max,a_max);
        }
    }
    else if(AutobackTrjFlag == true)
    {
        for(short i=0;i<6;i++)
        {
            p_start[i] = ExpectPose[i];
        }

        if(BackToStart == true)
        {
            trjplan.getP_start(p_end);
        }
        else if(BackToZero == true)
        {
            trjplan.getP_zero(p_end);
        }

        PutPoseToBuff(p_start,1);
        PutPoseToBuff(p_end,2);
        M_MatrixCal('L',TrjPoseBuff,v_max,a_max);
    }


}

void MainWindow::RemoteControlMode()
{
    static short div_cnt = 12; //subdivide number of each remote step.
    static short send_i=0;
    static bool sendflag = 0;
    static VectorXd angle(8);
    static VectorXd startangle(8);
    static float new_pose[6]={0.0};
    static VectorXd zero_angle(8);
    static MatrixXd ibp(4,8);
    static MatrixXd iep(4,8);

    //if remote button pressed and first come in.
    if((remote_dialog.X_UP_flag==1 || remote_dialog.X_DOWN_flag==1 ||remote_dialog.Y_UP_flag==1 \
        || remote_dialog.Y_DOWN_flag==1 ||remote_dialog.Z_UP_flag==1 || remote_dialog.Z_DOWN_flag==1) && send_i == 0 )
    {
        send_i = 1;
        sendflag = 1;
    }

    if(send_i==1)
    {
        for(short i=0;i<8;i++)
        {
            zero_angle(i) = ZeroAngle[i];
        }

        for(short i=0;i<6;i++)
        {
            new_pose[i] = ExpectPose[i];//set expect pose of CDPR.
        }
    }

    if(remote_dialog.X_UP_flag==1 && send_i==1)
    {
        new_pose[0] = new_pose[0]+remote_dialog.remote_step/1000.0;
    }
    else if(remote_dialog.X_DOWN_flag==1 && send_i==1)
    {
        new_pose[0] = new_pose[0]-remote_dialog.remote_step/1000.0;
    }
    else if(remote_dialog.Y_UP_flag==1 && send_i==1)
    {
        new_pose[1] = new_pose[1]+remote_dialog.remote_step/1000.0;
    }
    else if(remote_dialog.Y_DOWN_flag==1 && send_i==1)
    {
        new_pose[1] = new_pose[1]-remote_dialog.remote_step/1000.0;
    }
    else if(remote_dialog.Z_UP_flag==1 && send_i==1)
    {
        new_pose[2] = new_pose[2]+remote_dialog.remote_step/1000.0;
    }
    else if(remote_dialog.Z_DOWN_flag==1 && send_i==1)
    {
        new_pose[2] = new_pose[2]-remote_dialog.remote_step/1000.0;
    }

    if(send_i==1 && sendflag == 1)
    {
        //InvKinematic of CDPR.
        for(int i=0;i<4;i++)
        {
            Map <VectorXd> v_bp(bp[i],8);
            Map <VectorXd> v_ep(ep[i],8);
            ibp.row(i) = v_bp;
            iep.row(i) = v_ep;
        }

        Map<VectorXd> idia(dia,8);

        VectorXd p_ee(6);               //define single pose node p_ee;
        float XX=new_pose[0];
        float YY=new_pose[1];
        float ZZ=new_pose[2];
        float AX=new_pose[3];
        float AY=new_pose[4];
        float AZ=new_pose[5];
        p_ee<<XX,YY,ZZ,AX,AY,AZ;

        angle = InvsKnmtc_cdpr(p_ee,ibp,iep,idia,pulleyRad)/3.14159265354*180.0;

        angle = angle -zero_angle;

        for(short i=0;i<8;i++)
        {startangle(i) = ExpectAngle[i];}
    }

    //
    if(sendflag == 1)
    {
        for(int j =0;j<8;j++)
        {
            float sendangle = startangle(j)+(angle(j)-startangle(j))*(send_i)/div_cnt;
            //            serial_command_.my_command_positoncontrol_mode(j+1,sendangle);

            ExpectAngle[j] = sendangle;

            if(j == 7)
            {
                ExpectAngleUpdateFlag = true;
            }
        }

        if(send_i == div_cnt)
        {
            send_i = 0;
            sendflag = 0;
        }
        else
        {
            send_i++;
        }
    }
    else
    {
        send_i = 0;
    }
}

void MainWindow::MovingCommandSend()
{
    static short cnt = PCtoFC_cnt;
    if(serial_command_.getControlModeSetOkFlag() == true)
    {
        //        cout<<"7777"<<endl;
        if(MovingCommandSendEnableFlag == true)
        {
            if(ComplainceMode == false)//position mode.
            {
                if(cnt >= PCtoFC_cnt)
                {
                    //            cout<<"888"<<endl;
                    cnt = 0;

                    bool joint_mode[8];
                    my_position_mode_set_dialog.get_joint_mode(joint_mode);
                    for(short i=0;i<8;i++)
                    {
                        if(joint_mode[i]==0)//judge joint mode.
                        {
                            serial_command_.my_command_positoncontrol_mode(i+1,ExpectAngle[i]);
                            //                            if(i == 0)
                            //                                cout<<"Angle:"<<ExpectAngle[i]<<endl;
                        }
                        else
                        {
                            serial_command_.my_command_forcecontrol_mode(i+1,ExpectTesion[i]);
                        }
                    }
                }
                cnt++;
            }
            else//complaince mode.
            {
                for(short i=0;i<8;i++)
                {
                    if(ExpectTesion[i] > 40.0)
                    {
                        serial_command_.my_command_forcecontrol_mode(i+1,30.0);
                    }
                    else
                    {
                        serial_command_.my_command_forcecontrol_mode(i+1,ExpectTesion[i]);
                    }
                    //                if(i==0)
                    //                {
                    //                   cout<<"Tesion: ";
                    //                }
                    //                cout<<ExpectTesion[i]<<"\t";
                    //                if(i == 7)
                    //                {
                    //                    cout<<endl;
                    //                }
                }
            }
        }
    }
}

void MainWindow::controltask()
{
    static short cnt = PCtoFC_cnt;

    if(MovingMode == SglJointMode_e)
    {
        if(cnt == PCtoFC_cnt)
        {
            cnt = 0;
            SglJointMode();
        }
        cnt++;
    }
    else if(MovingMode == TrjMode_e)
    {
        if(ui->teachingmode_radioButton->isChecked())
        {
            //record teaching trj program;
        }
        else if(ui->remotemode_radioButton->isChecked())
        {
            if(remote_dialog.isActiveWindow())
            {
                if(!isNearP_Zero())
                {
                    RemoteControlMode();
                }
            }
            else if(RemoteMovingFlag == true)
            {
                RemoteMovingFlag = false;
                ui->positionmode_radioButton->setChecked(true);
            }
        }
        else//all trj type, zerotostart, autoback and general trj.
        {
            if(!InterpolationOKFlag)
            {
                TrjControlMode();
            }

            if(MovingCompleteFlag == 0)
            {
                TrjExpectAngleCal();
            }
        }

        MovingCommandSend();
    }

    //    //before
    //    // aflter zeroing, excute zerotostart mode. go to start point.
    //    if(ZeroToStartFlag == 1)
    //    {
    //        ZeroToStartMode();
    //    }
    //    //position control mode.
    //    else if(ui->positionmode_radioButton->isChecked() && AutobackMovingFlag == false)
    //    {
    //        PositionControlMode();
    //    }

    //    else if(ui->forcemode_radioButton->isChecked() && AutobackMovingFlag ==false)
    //    {
    //        ComplaintControlMode();
    //    }
    //    //remote control mode.
    //    else if(ui->remotemode_radioButton->isChecked() && AutobackMovingFlag ==false)
    //    {
    //        if(!isNearP_Zero())
    //        {
    //            RemoteControlMode();
    //        }
    //    }
    //    else if(AutobackMovingFlag == true)
    //    {
    //        AutoBackMode();
    //    }

    // send command to stm32.
}

void MainWindow::diplaytask()
{
    UI_refresh();
    MessageShow();
}


void MainWindow::TaskRemarks()
{

    if(display_time == 0)
    {
        display_time=display_time_reload;
        diplayflag = 1;
    }

    if(control_time == 0)
    {
        control_time=control_time_reload;
        controlflag = 1;
    }

    if(ThreadStartTime == 0)
    {
        ThreadStartTime = ThreadStartTimeReload;
        ThreadStartFlag = 1;
    }

    ThreadStartTime--;
    display_time--;
    control_time--;

}

void MainWindow::TaskProcess()
{
    if(serial_command_.getflag(serialconsucf)==1)
    {
        m_thread->start();//start my thread.

        if(serial_command_.getflag(paraconfigf) == 0)//if not config para,then do this.
        {
            //            PIDParaConfig();
            //            serial_command_.my_command_paraconfigok();
        }


        if(controlflag == 1)
        {
            controlflag=0;
            controltask();
        }
    }

    if(diplayflag == 1)
    {
        diplayflag=0;
        diplaytask();
    }
}


void MainWindow::handleTimeout()
{
    if(m_pTimer->isActive()){
        m_pTimer->start(TIMER_TIMEOUT);

        TaskRemarks();   //task remarks prossecing program.

        TaskProcess();   //task process program.
    }
}

void MainWindow::on_zero_set_button_clicked()
{
    if(serial_command_.getState() == PREPARE)   //judge state of CDPR, when state is "prepare", this operate works.
    {
        zeringOKflag = true;
        for(int i=0;i<8;i++)
        {ExpectAngle[i] = 0.0;}
        //set initial angle.
        serial_command_.my_command_zeroset();

        //get lineedit zero point.
        double p_zero[6],p_start[6];
        p_zero[0] = ui ->zero_X_linetext->text().toFloat()/1000.0;
        p_zero[1] = ui ->zero_Y_linetext->text().toFloat()/1000.0;
        p_zero[2] = ui ->zero_Z_linetext->text().toFloat()/1000.0;
        p_zero[3] = 0;//X-rotation.
        p_zero[4] = 0;//Y-rotation.
        p_zero[5] = 0;//Z-rotation.

        //p_start is add z_offset from p_zero.
        p_start[0] = p_zero[0];
        p_start[1] = p_zero[1];
        p_start[2] = p_zero[2] +trjplan.getP_start_zoffset(); //zoffset is user define.
        p_start[3] = p_zero[3];
        p_start[4] = p_zero[4];
        p_start[5] = p_zero[5];
        trjplan.setP_zero(p_zero);
        trjplan.setP_start(p_start);

        //** SET ZERO POSITION JOINTS ANGLE **//
        MatrixXd ibp(4,8);
        MatrixXd iep(4,8);
        for(int i=0;i<4;i++)
        {
            Map <VectorXd> v_bp(bp[i],8);
            Map <VectorXd> v_ep(ep[i],8);
            ibp.row(i) = v_bp;
            iep.row(i) = v_ep;
        }

        Map<VectorXd> idia(dia,8);
        Map<VectorXd> P_ZERO(p_zero,6);
        VectorXd angle_node_zero(8);
        angle_node_zero = InvsKnmtc_cdpr(P_ZERO,ibp,iep,idia,pulleyRad)/3.1415926*180.0;

        for(short i=0;i<8;i++)
        {
            ZeroAngle[i] = angle_node_zero(i);
        }
        ExpectAngleUpdateFlag = true;

        //        trjplan.setAngleNodeZero(angle_node_zero);

        if(L0UpdateFlag == true)
        {
            L0UpdateFlag =false;
            L0ParaConfDisplayFlag = 0;

            L0ParaConfig();
        }
    }

    else
    {
        QMessageBox::warning(this,tr("Warning:"),tr("CDPR is running, please rest cdpr first!"));
    }
}

void MainWindow::ReadDiaFromEdit(void)
{
    dia[0]=ui->Dia1_lineEdit->text().toFloat()/1000.0;
    dia[1]=ui->Dia2_lineEdit->text().toFloat()/1000.0;
    dia[2]=ui->Dia3_lineEdit->text().toFloat()/1000.0;
    dia[3]=ui->Dia4_lineEdit->text().toFloat()/1000.0;
    dia[4]=ui->Dia5_lineEdit->text().toFloat()/1000.0;
    dia[5]=ui->Dia6_lineEdit->text().toFloat()/1000.0;
    dia[6]=ui->Dia7_lineEdit->text().toFloat()/1000.0;
    dia[7]=ui->Dia8_lineEdit->text().toFloat()/1000.0;
}

void MainWindow::WriteDiatoEdit(void)
{
    ui->Dia1_lineEdit->setText(QString::number(dia[0]*1000.0,'f',3));
    ui->Dia2_lineEdit->setText(QString::number(dia[1]*1000.0,'f',3));
    ui->Dia3_lineEdit->setText(QString::number(dia[2]*1000.0,'f',3));
    ui->Dia4_lineEdit->setText(QString::number(dia[3]*1000.0,'f',3));
    ui->Dia5_lineEdit->setText(QString::number(dia[4]*1000.0,'f',3));
    ui->Dia6_lineEdit->setText(QString::number(dia[5]*1000.0,'f',3));
    ui->Dia7_lineEdit->setText(QString::number(dia[6]*1000.0,'f',3));
    ui->Dia8_lineEdit->setText(QString::number(dia[7]*1000.0,'f',3));
}

void MainWindow::ReadDmFromEdit(void)
{
    Dm[0] = ui->Dm1_lineEdit->text().toFloat()/1000.0;
    Dm[1] = ui->Dm2_lineEdit->text().toFloat()/1000.0;
    Dm[2] = ui->Dm3_lineEdit->text().toFloat()/1000.0;
    Dm[3] = ui->Dm4_lineEdit->text().toFloat()/1000.0;
    Dm[4] = ui->Dm5_lineEdit->text().toFloat()/1000.0;
    Dm[5] = ui->Dm6_lineEdit->text().toFloat()/1000.0;
    Dm[6] = ui->Dm7_lineEdit->text().toFloat()/1000.0;
    Dm[7] = ui->Dm8_lineEdit->text().toFloat()/1000.0;
}

void MainWindow::WriteDmtoEdit(void)
{
    ui->Dm1_lineEdit->setText(QString::number(Dm[0]*1000.0,'f',3));
    ui->Dm2_lineEdit->setText(QString::number(Dm[1]*1000.0,'f',3));
    ui->Dm3_lineEdit->setText(QString::number(Dm[2]*1000.0,'f',3));
    ui->Dm4_lineEdit->setText(QString::number(Dm[3]*1000.0,'f',3));
    ui->Dm5_lineEdit->setText(QString::number(Dm[4]*1000.0,'f',3));
    ui->Dm6_lineEdit->setText(QString::number(Dm[5]*1000.0,'f',3));
    ui->Dm7_lineEdit->setText(QString::number(Dm[6]*1000.0,'f',3));
    ui->Dm8_lineEdit->setText(QString::number(Dm[7]*1000.0,'f',3));
}

void MainWindow::ReadDcFromEdit(void)
{
    Dc = ui->Dc_lineEdit->text().toFloat()/1000.0;
}

void MainWindow::WriteDctoEdit(void)
{
    ui->Dc_lineEdit->setText(QString::number(Dc*1000.0,'f',3));
}

void MainWindow::ReadEFromEdit(void)
{
    E = ui->E_lineEdit->text().toFloat()*(1E+9);
}

void MainWindow::WriteEtoEdit(void)
{
    ui->E_lineEdit->setText(QString::number(E/(1E+9),'f',3));
}

void MainWindow::WriteStrucParaToEdit(void)
{
    ui->Bp1x_lineEdit->setText(QString::number(bp[0][0]*1000.0,'f',3));
    ui->Bp1y_lineEdit->setText(QString::number(bp[1][0]*1000.0,'f',3));
    ui->Bp1z_lineEdit->setText(QString::number(bp[2][0]*1000.0,'f',3));
    ui->Bp2x_lineEdit->setText(QString::number(bp[0][1]*1000.0,'f',3));
    ui->Bp2y_lineEdit->setText(QString::number(bp[1][1]*1000.0,'f',3));
    ui->Bp2z_lineEdit->setText(QString::number(bp[2][1]*1000.0,'f',3));
    ui->Bp3x_lineEdit->setText(QString::number(bp[0][2]*1000.0,'f',3));
    ui->Bp3y_lineEdit->setText(QString::number(bp[1][2]*1000.0,'f',3));
    ui->Bp3z_lineEdit->setText(QString::number(bp[2][2]*1000.0,'f',3));
    ui->Bp4x_lineEdit->setText(QString::number(bp[0][3]*1000.0,'f',3));
    ui->Bp4y_lineEdit->setText(QString::number(bp[1][3]*1000.0,'f',3));
    ui->Bp4z_lineEdit->setText(QString::number(bp[2][3]*1000.0,'f',3));
    ui->Bp5x_lineEdit->setText(QString::number(bp[0][4]*1000.0,'f',3));
    ui->Bp5y_lineEdit->setText(QString::number(bp[1][4]*1000.0,'f',3));
    ui->Bp5z_lineEdit->setText(QString::number(bp[2][4]*1000.0,'f',3));
    ui->Bp6x_lineEdit->setText(QString::number(bp[0][5]*1000.0,'f',3));
    ui->Bp6y_lineEdit->setText(QString::number(bp[1][5]*1000.0,'f',3));
    ui->Bp6z_lineEdit->setText(QString::number(bp[2][5]*1000.0,'f',3));
    ui->Bp7x_lineEdit->setText(QString::number(bp[0][6]*1000.0,'f',3));
    ui->Bp7y_lineEdit->setText(QString::number(bp[1][6]*1000.0,'f',3));
    ui->Bp7z_lineEdit->setText(QString::number(bp[2][6]*1000.0,'f',3));
    ui->Bp8x_lineEdit->setText(QString::number(bp[0][7]*1000.0,'f',3));
    ui->Bp8y_lineEdit->setText(QString::number(bp[1][7]*1000.0,'f',3));
    ui->Bp8z_lineEdit->setText(QString::number(bp[2][7]*1000.0,'f',3));

    ui->Ep1x_lineEdit->setText(QString::number(ep[0][0]*1000.0,'f',3));
    ui->Ep1y_lineEdit->setText(QString::number(ep[1][0]*1000.0,'f',3));
    ui->Ep1z_lineEdit->setText(QString::number(ep[2][0]*1000.0,'f',3));
    ui->Ep2x_lineEdit->setText(QString::number(ep[0][1]*1000.0,'f',3));
    ui->Ep2y_lineEdit->setText(QString::number(ep[1][1]*1000.0,'f',3));
    ui->Ep2z_lineEdit->setText(QString::number(ep[2][1]*1000.0,'f',3));
    ui->Ep3x_lineEdit->setText(QString::number(ep[0][2]*1000.0,'f',3));
    ui->Ep3y_lineEdit->setText(QString::number(ep[1][2]*1000.0,'f',3));
    ui->Ep3z_lineEdit->setText(QString::number(ep[2][2]*1000.0,'f',3));
    ui->Ep4x_lineEdit->setText(QString::number(ep[0][3]*1000.0,'f',3));
    ui->Ep4y_lineEdit->setText(QString::number(ep[1][3]*1000.0,'f',3));
    ui->Ep4z_lineEdit->setText(QString::number(ep[2][3]*1000.0,'f',3));
    ui->Ep5x_lineEdit->setText(QString::number(ep[0][4]*1000.0,'f',3));
    ui->Ep5y_lineEdit->setText(QString::number(ep[1][4]*1000.0,'f',3));
    ui->Ep5z_lineEdit->setText(QString::number(ep[2][4]*1000.0,'f',3));
    ui->Ep6x_lineEdit->setText(QString::number(ep[0][5]*1000.0,'f',3));
    ui->Ep6y_lineEdit->setText(QString::number(ep[1][5]*1000.0,'f',3));
    ui->Ep6z_lineEdit->setText(QString::number(ep[2][5]*1000.0,'f',3));
    ui->Ep7x_lineEdit->setText(QString::number(ep[0][6]*1000.0,'f',3));
    ui->Ep7y_lineEdit->setText(QString::number(ep[1][6]*1000.0,'f',3));
    ui->Ep7z_lineEdit->setText(QString::number(ep[2][6]*1000.0,'f',3));
    ui->Ep8x_lineEdit->setText(QString::number(ep[0][7]*1000.0,'f',3));
    ui->Ep8y_lineEdit->setText(QString::number(ep[1][7]*1000.0,'f',3));
    ui->Ep8z_lineEdit->setText(QString::number(ep[2][7]*1000.0,'f',3));

    //    ui->Dia1_lineEdit->setText(QString::number(dia[0]*1000.0,'f',3));
    //    ui->Dia2_lineEdit->setText(QString::number(dia[1]*1000.0,'f',3));
    //    ui->Dia3_lineEdit->setText(QString::number(dia[2]*1000.0,'f',3));
    //    ui->Dia4_lineEdit->setText(QString::number(dia[3]*1000.0,'f',3));
    //    ui->Dia5_lineEdit->setText(QString::number(dia[4]*1000.0,'f',3));
    //    ui->Dia6_lineEdit->setText(QString::number(dia[5]*1000.0,'f',3));
    //    ui->Dia7_lineEdit->setText(QString::number(dia[6]*1000.0,'f',3));
    //    ui->Dia8_lineEdit->setText(QString::number(dia[7]*1000.0,'f',3));

    ui->Pulley1_lineEdit->setText(QString::number(pulleyRad[0]*1000.0,'f',3));
    ui->Pulley2_lineEdit->setText(QString::number(pulleyRad[1]*1000.0,'f',3));
    ui->Pulley3_lineEdit->setText(QString::number(pulleyRad[2]*1000.0,'f',3));
    ui->Pulley4_lineEdit->setText(QString::number(pulleyRad[3]*1000.0,'f',3));
    ui->Pulley5_lineEdit->setText(QString::number(pulleyRad[4]*1000.0,'f',3));
    ui->Pulley6_lineEdit->setText(QString::number(pulleyRad[5]*1000.0,'f',3));
    ui->Pulley7_lineEdit->setText(QString::number(pulleyRad[6]*1000.0,'f',3));
    ui->Pulley8_lineEdit->setText(QString::number(pulleyRad[7]*1000.0,'f',3));
}

void MainWindow::ReadStrucParaFromEdit(void)
{
    bp[0][0]=ui->Bp1x_lineEdit->text().toFloat()/1000.0;
    bp[1][0]=ui->Bp1y_lineEdit->text().toFloat()/1000.0;
    bp[2][0]=ui->Bp1z_lineEdit->text().toFloat()/1000.0;
    bp[0][1]=ui->Bp2x_lineEdit->text().toFloat()/1000.0;
    bp[1][1]=ui->Bp2y_lineEdit->text().toFloat()/1000.0;
    bp[2][1]=ui->Bp2z_lineEdit->text().toFloat()/1000.0;
    bp[0][2]=ui->Bp3x_lineEdit->text().toFloat()/1000.0;
    bp[1][2]=ui->Bp3y_lineEdit->text().toFloat()/1000.0;
    bp[2][2]=ui->Bp3z_lineEdit->text().toFloat()/1000.0;
    bp[0][3]=ui->Bp4x_lineEdit->text().toFloat()/1000.0;
    bp[1][3]=ui->Bp4y_lineEdit->text().toFloat()/1000.0;
    bp[2][3]=ui->Bp4z_lineEdit->text().toFloat()/1000.0;
    bp[0][4]=ui->Bp5x_lineEdit->text().toFloat()/1000.0;
    bp[1][4]=ui->Bp5y_lineEdit->text().toFloat()/1000.0;
    bp[2][4]=ui->Bp5z_lineEdit->text().toFloat()/1000.0;
    bp[0][5]=ui->Bp6x_lineEdit->text().toFloat()/1000.0;
    bp[1][5]=ui->Bp6y_lineEdit->text().toFloat()/1000.0;
    bp[2][5]=ui->Bp6z_lineEdit->text().toFloat()/1000.0;
    bp[0][6]=ui->Bp7x_lineEdit->text().toFloat()/1000.0;
    bp[1][6]=ui->Bp7y_lineEdit->text().toFloat()/1000.0;
    bp[2][6]=ui->Bp7z_lineEdit->text().toFloat()/1000.0;
    bp[0][7]=ui->Bp8x_lineEdit->text().toFloat()/1000.0;
    bp[1][7]=ui->Bp8y_lineEdit->text().toFloat()/1000.0;
    bp[2][7]=ui->Bp8z_lineEdit->text().toFloat()/1000.0;

    ep[0][0]=ui->Ep1x_lineEdit->text().toFloat()/1000.0;
    ep[1][0]=ui->Ep1y_lineEdit->text().toFloat()/1000.0;
    ep[2][0]=ui->Ep1z_lineEdit->text().toFloat()/1000.0;
    ep[0][1]=ui->Ep2x_lineEdit->text().toFloat()/1000.0;
    ep[1][1]=ui->Ep2y_lineEdit->text().toFloat()/1000.0;
    ep[2][1]=ui->Ep2z_lineEdit->text().toFloat()/1000.0;
    ep[0][2]=ui->Ep3x_lineEdit->text().toFloat()/1000.0;
    ep[1][2]=ui->Ep3y_lineEdit->text().toFloat()/1000.0;
    ep[2][2]=ui->Ep3z_lineEdit->text().toFloat()/1000.0;
    ep[0][3]=ui->Ep4x_lineEdit->text().toFloat()/1000.0;
    ep[1][3]=ui->Ep4y_lineEdit->text().toFloat()/1000.0;
    ep[2][3]=ui->Ep4z_lineEdit->text().toFloat()/1000.0;
    ep[0][4]=ui->Ep5x_lineEdit->text().toFloat()/1000.0;
    ep[1][4]=ui->Ep5y_lineEdit->text().toFloat()/1000.0;
    ep[2][4]=ui->Ep5z_lineEdit->text().toFloat()/1000.0;
    ep[0][5]=ui->Ep6x_lineEdit->text().toFloat()/1000.0;
    ep[1][5]=ui->Ep6y_lineEdit->text().toFloat()/1000.0;
    ep[2][5]=ui->Ep6z_lineEdit->text().toFloat()/1000.0;
    ep[0][6]=ui->Ep7x_lineEdit->text().toFloat()/1000.0;
    ep[1][6]=ui->Ep7y_lineEdit->text().toFloat()/1000.0;
    ep[2][6]=ui->Ep7z_lineEdit->text().toFloat()/1000.0;
    ep[0][7]=ui->Ep8x_lineEdit->text().toFloat()/1000.0;
    ep[1][7]=ui->Ep8y_lineEdit->text().toFloat()/1000.0;
    ep[2][7]=ui->Ep8z_lineEdit->text().toFloat()/1000.0;

    //    dia[0]=ui->Dia1_lineEdit->text().toFloat()/1000.0;
    //    dia[1]=ui->Dia2_lineEdit->text().toFloat()/1000.0;
    //    dia[2]=ui->Dia3_lineEdit->text().toFloat()/1000.0;
    //    dia[3]=ui->Dia4_lineEdit->text().toFloat()/1000.0;
    //    dia[4]=ui->Dia5_lineEdit->text().toFloat()/1000.0;
    //    dia[5]=ui->Dia6_lineEdit->text().toFloat()/1000.0;
    //    dia[6]=ui->Dia7_lineEdit->text().toFloat()/1000.0;
    //    dia[7]=ui->Dia8_lineEdit->text().toFloat()/1000.0;

    pulleyRad[0]=ui->Pulley1_lineEdit->text().toFloat()/1000.0;
    pulleyRad[1]=ui->Pulley2_lineEdit->text().toFloat()/1000.0;
    pulleyRad[2]=ui->Pulley3_lineEdit->text().toFloat()/1000.0;
    pulleyRad[3]=ui->Pulley4_lineEdit->text().toFloat()/1000.0;
    pulleyRad[4]=ui->Pulley5_lineEdit->text().toFloat()/1000.0;
    pulleyRad[5]=ui->Pulley6_lineEdit->text().toFloat()/1000.0;
    pulleyRad[6]=ui->Pulley7_lineEdit->text().toFloat()/1000.0;
    pulleyRad[7]=ui->Pulley8_lineEdit->text().toFloat()/1000.0;

}

void MainWindow::WriteStrucParaToIni(void)
{
    QSettings *ParaConfigIniWrite = new QSettings("para.ini",QSettings::IniFormat);

    QString str;
    ParaConfigIniWrite->setValue("/para_ip/bp1x",str.number(bp[0][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp1y",str.number(bp[1][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp1z",str.number(bp[2][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp2x",str.number(bp[0][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp2y",str.number(bp[1][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp2z",str.number(bp[2][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp3x",str.number(bp[0][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp3y",str.number(bp[1][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp3z",str.number(bp[2][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp4x",str.number(bp[0][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp4y",str.number(bp[1][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp4z",str.number(bp[2][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp5x",str.number(bp[0][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp5y",str.number(bp[1][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp5z",str.number(bp[2][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp6x",str.number(bp[0][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp6y",str.number(bp[1][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp6z",str.number(bp[2][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp7x",str.number(bp[0][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp7y",str.number(bp[1][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp7z",str.number(bp[2][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp8x",str.number(bp[0][7]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp8y",str.number(bp[1][7]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/bp8z",str.number(bp[2][7]*1000.0));

    ParaConfigIniWrite->setValue("/para_ip/ep1x",str.number(ep[0][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep1y",str.number(ep[1][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep1z",str.number(ep[2][0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep2x",str.number(ep[0][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep2y",str.number(ep[1][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep2z",str.number(ep[2][1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep3x",str.number(ep[0][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep3y",str.number(ep[1][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep3z",str.number(ep[2][2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep4x",str.number(ep[0][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep4y",str.number(ep[1][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep4z",str.number(ep[2][3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep5x",str.number(ep[0][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep5y",str.number(ep[1][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep5z",str.number(ep[2][4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep6x",str.number(ep[0][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep6y",str.number(ep[1][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep6z",str.number(ep[2][5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep7x",str.number(ep[0][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep7y",str.number(ep[1][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep7z",str.number(ep[2][6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep8x",str.number(ep[0][7]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep8y",str.number(ep[1][7]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/ep8z",str.number(ep[2][7]*1000.0));

    ParaConfigIniWrite->setValue("/para_ip/dia1",str.number(dia[0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia2",str.number(dia[1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia3",str.number(dia[2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia4",str.number(dia[3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia5",str.number(dia[4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia6",str.number(dia[5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia7",str.number(dia[6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/dia8",str.number(dia[7]*1000.0));

    ParaConfigIniWrite->setValue("/para_ip/pul1",str.number(pulleyRad[0]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul2",str.number(pulleyRad[1]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul3",str.number(pulleyRad[2]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul4",str.number(pulleyRad[3]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul5",str.number(pulleyRad[4]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul6",str.number(pulleyRad[5]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul7",str.number(pulleyRad[6]*1000.0));
    ParaConfigIniWrite->setValue("/para_ip/pul8",str.number(pulleyRad[7]*1000.0));

    delete ParaConfigIniWrite;

}

void MainWindow::ReadStrucParaFromIni(void)
{
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);

    bp[0][0] = ParaConfigIniRead->value("/para_ip/bp1x").toFloat()/1000.0;
    bp[1][0] = ParaConfigIniRead->value("/para_ip/bp1y").toFloat()/1000.0;
    bp[2][0] = ParaConfigIniRead->value("/para_ip/bp1z").toFloat()/1000.0;
    bp[0][1] = ParaConfigIniRead->value("/para_ip/bp2x").toFloat()/1000.0;
    bp[1][1] = ParaConfigIniRead->value("/para_ip/bp2y").toFloat()/1000.0;
    bp[2][1] = ParaConfigIniRead->value("/para_ip/bp2z").toFloat()/1000.0;
    bp[0][2] = ParaConfigIniRead->value("/para_ip/bp3x").toFloat()/1000.0;
    bp[1][2] = ParaConfigIniRead->value("/para_ip/bp3y").toFloat()/1000.0;
    bp[2][2] = ParaConfigIniRead->value("/para_ip/bp3z").toFloat()/1000.0;
    bp[0][3] = ParaConfigIniRead->value("/para_ip/bp4x").toFloat()/1000.0;
    bp[1][3] = ParaConfigIniRead->value("/para_ip/bp4y").toFloat()/1000.0;
    bp[2][3] = ParaConfigIniRead->value("/para_ip/bp4z").toFloat()/1000.0;
    bp[0][4] = ParaConfigIniRead->value("/para_ip/bp5x").toFloat()/1000.0;
    bp[1][4] = ParaConfigIniRead->value("/para_ip/bp5y").toFloat()/1000.0;
    bp[2][4] = ParaConfigIniRead->value("/para_ip/bp5z").toFloat()/1000.0;
    bp[0][5] = ParaConfigIniRead->value("/para_ip/bp6x").toFloat()/1000.0;
    bp[1][5] = ParaConfigIniRead->value("/para_ip/bp6y").toFloat()/1000.0;
    bp[2][5] = ParaConfigIniRead->value("/para_ip/bp6z").toFloat()/1000.0;
    bp[0][6] = ParaConfigIniRead->value("/para_ip/bp7x").toFloat()/1000.0;
    bp[1][6] = ParaConfigIniRead->value("/para_ip/bp7y").toFloat()/1000.0;
    bp[2][6] = ParaConfigIniRead->value("/para_ip/bp7z").toFloat()/1000.0;
    bp[0][7] = ParaConfigIniRead->value("/para_ip/bp8x").toFloat()/1000.0;
    bp[1][7] = ParaConfigIniRead->value("/para_ip/bp8y").toFloat()/1000.0;
    bp[2][7] = ParaConfigIniRead->value("/para_ip/bp8z").toFloat()/1000.0;

    ep[0][0] = ParaConfigIniRead->value("/para_ip/ep1x").toFloat()/1000.0;
    ep[1][0] = ParaConfigIniRead->value("/para_ip/ep1y").toFloat()/1000.0;
    ep[2][0] = ParaConfigIniRead->value("/para_ip/ep1z").toFloat()/1000.0;
    ep[0][1] = ParaConfigIniRead->value("/para_ip/ep2x").toFloat()/1000.0;
    ep[1][1] = ParaConfigIniRead->value("/para_ip/ep2y").toFloat()/1000.0;
    ep[2][1] = ParaConfigIniRead->value("/para_ip/ep2z").toFloat()/1000.0;
    ep[0][2] = ParaConfigIniRead->value("/para_ip/ep3x").toFloat()/1000.0;
    ep[1][2] = ParaConfigIniRead->value("/para_ip/ep3y").toFloat()/1000.0;
    ep[2][2] = ParaConfigIniRead->value("/para_ip/ep3z").toFloat()/1000.0;
    ep[0][3] = ParaConfigIniRead->value("/para_ip/ep4x").toFloat()/1000.0;
    ep[1][3] = ParaConfigIniRead->value("/para_ip/ep4y").toFloat()/1000.0;
    ep[2][3] = ParaConfigIniRead->value("/para_ip/ep4z").toFloat()/1000.0;
    ep[0][4] = ParaConfigIniRead->value("/para_ip/ep5x").toFloat()/1000.0;
    ep[1][4] = ParaConfigIniRead->value("/para_ip/ep5y").toFloat()/1000.0;
    ep[2][4] = ParaConfigIniRead->value("/para_ip/ep5z").toFloat()/1000.0;
    ep[0][5] = ParaConfigIniRead->value("/para_ip/ep6x").toFloat()/1000.0;
    ep[1][5] = ParaConfigIniRead->value("/para_ip/ep6y").toFloat()/1000.0;
    ep[2][5] = ParaConfigIniRead->value("/para_ip/ep6z").toFloat()/1000.0;
    ep[0][6] = ParaConfigIniRead->value("/para_ip/ep7x").toFloat()/1000.0;
    ep[1][6] = ParaConfigIniRead->value("/para_ip/ep7y").toFloat()/1000.0;
    ep[2][6] = ParaConfigIniRead->value("/para_ip/ep7z").toFloat()/1000.0;
    ep[0][7] = ParaConfigIniRead->value("/para_ip/ep8x").toFloat()/1000.0;
    ep[1][7] = ParaConfigIniRead->value("/para_ip/ep8y").toFloat()/1000.0;
    ep[2][7] = ParaConfigIniRead->value("/para_ip/ep8z").toFloat()/1000.0;


    for(int i=0;i<8;i++)
    {
        bp[3][i] = 1;
        ep[3][i] = 1;
    }

    //    dia[0] = ParaConfigIniRead->value("/para_ip/dia1").toFloat()/1000.0;
    //    dia[1] = ParaConfigIniRead->value("/para_ip/dia2").toFloat()/1000.0;
    //    dia[2] = ParaConfigIniRead->value("/para_ip/dia3").toFloat()/1000.0;
    //    dia[3] = ParaConfigIniRead->value("/para_ip/dia4").toFloat()/1000.0;
    //    dia[4] = ParaConfigIniRead->value("/para_ip/dia5").toFloat()/1000.0;
    //    dia[5] = ParaConfigIniRead->value("/para_ip/dia6").toFloat()/1000.0;
    //    dia[6] = ParaConfigIniRead->value("/para_ip/dia7").toFloat()/1000.0;
    //    dia[7] = ParaConfigIniRead->value("/para_ip/dia8").toFloat()/1000.0;

    pulleyRad[0] = ParaConfigIniRead->value("/para_ip/pul1").toFloat()/1000.0;
    pulleyRad[1] = ParaConfigIniRead->value("/para_ip/pul2").toFloat()/1000.0;
    pulleyRad[2] = ParaConfigIniRead->value("/para_ip/pul3").toFloat()/1000.0;
    pulleyRad[3] = ParaConfigIniRead->value("/para_ip/pul4").toFloat()/1000.0;
    pulleyRad[4] = ParaConfigIniRead->value("/para_ip/pul5").toFloat()/1000.0;
    pulleyRad[5] = ParaConfigIniRead->value("/para_ip/pul6").toFloat()/1000.0;
    pulleyRad[6] = ParaConfigIniRead->value("/para_ip/pul7").toFloat()/1000.0;
    pulleyRad[7] = ParaConfigIniRead->value("/para_ip/pul8").toFloat()/1000.0;

    delete ParaConfigIniRead;
}

void MainWindow::WriteZeroPoseToEdit(void)
{

    double p_zero[6];
    trjplan.getP_zero(p_zero);
    ui->zero_X_linetext->setText(QString::number(p_zero[0]*1000,'f',2));
    ui->zero_Y_linetext->setText(QString::number(p_zero[1]*1000,'f',2));
    ui->zero_Z_linetext->setText(QString::number(p_zero[2]*1000,'f',2));

    ui->p_start_offset_linetext->setText(QString::number(trjplan.getP_start_zoffset()*1000,'f',2));
}

void MainWindow::ReadZeroPoseFromEdit(void)
{
    //get lineedit zero point.
    double p_zero[6];
    p_zero[0] = ui ->zero_X_linetext->text().toFloat()/1000.0;
    p_zero[1] = ui ->zero_Y_linetext->text().toFloat()/1000.0;
    p_zero[2] = ui ->zero_Z_linetext->text().toFloat()/1000.0;
    p_zero[3] = 0;//X-rotation.
    p_zero[4] = 0;//Y-rotation.
    p_zero[5] = 0;//Z-rotation.

    trjplan.setP_zero(p_zero);

    float offset = ui->p_start_offset_linetext->text().toFloat()/1000.0;
    trjplan.setP_start_zoffset(offset);
}

void MainWindow::WriteZeroPoseToIni(void)
{
    QSettings *ParaConfigIniWrite = new QSettings("para.ini",QSettings::IniFormat);

    double p_zero[6];
    trjplan.getP_zero(p_zero);
    ParaConfigIniWrite->setValue("/para_ip/ZERO_x",p_zero[0]);//UNIT:m
    ParaConfigIniWrite->setValue("/para_ip/ZERO_y",p_zero[1]);
    ParaConfigIniWrite->setValue("/para_ip/ZERO_z",p_zero[2]);

    ParaConfigIniWrite->setValue("/para_ip/P_Start_Offset",trjplan.getP_start_zoffset());

    delete ParaConfigIniWrite;

}

void MainWindow::ReadZeroPoseFromIni(void)
{
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);
    double p_zero[6];
    p_zero[0] = ParaConfigIniRead->value("/para_ip/ZERO_x").toFloat();
    p_zero[1] = ParaConfigIniRead->value("/para_ip/ZERO_y").toFloat();
    p_zero[2] = ParaConfigIniRead->value("/para_ip/ZERO_z").toFloat();

    p_zero[3] = 0;//X-rotation.
    p_zero[4] = 0;//Y-rotation.
    p_zero[5] = 0;//Z-rotation.

    trjplan.setP_zero(p_zero);

    float offset = ParaConfigIniRead->value("/para_ip/P_Start_Offset").toFloat();
    trjplan.setP_start_zoffset(offset);

    delete ParaConfigIniRead;
}

void MainWindow::on_ReadTxt_pushButton_clicked()
{
    ReadStrucParaFromIni();
    WriteStrucParaToEdit();
}

void MainWindow::on_SaveTxt_pushButton_clicked()
{
    ReadStrucParaFromEdit();
    WriteStrucParaToIni();
    WriteStrucParaToEdit();
}


void MainWindow::on_save_pushButton_clicked()
{
    ReadZeroPoseFromEdit();
    WriteZeroPoseToIni();
    WriteZeroPoseToEdit();

    float temp_offset = trjplan.getP_start_zoffset();
    if(temp_offset<0.05)
    {
        QMessageBox::warning(this,tr("Error:"),tr("Offset value is too small, please reset it!"));
    }

    L0UpdateFlag = true;
}

void MainWindow::on_Reset_pushButton_clicked()
{
    if(serial_command_.getState() != RUNNING)
    {
        QMessageBox messageBox(QMessageBox::NoIcon,"RESET:","CDPR will be reset, do you want to continue?",
                               QMessageBox::Yes | QMessageBox::No,NULL);
        int ret = messageBox.exec();

        if(ret==QMessageBox::Yes)
        {
            LineNum = 1;

            AllControlFlagInit();

            ui->positionmode_radioButton->setCheckable(false);
            ui->positionmode_radioButton->setCheckable(true);
            ui->forcemode_radioButton->setCheckable(false);
            ui->forcemode_radioButton->setCheckable(true);
            ui->remotemode_radioButton->setCheckable(false);
            ui->remotemode_radioButton->setCheckable(true);
            ui->teachingmode_radioButton->setCheckable(false);
            ui->teachingmode_radioButton->setCheckable(true);

            for(int i=0;i<8;i++)
            {
                ExpectAngle[i] = 0.0;
            }

            //            serial_command_.my_command_zeroset();

            serial_command_.setState(PREPARE);

            serial_command_.my_command_stateset(PREPARE);

            QMessageBox::information(this,tr("CDPR reset:"),tr("RESET OK!"));
        }
        else
        {;}
    }
    else
        QMessageBox::warning(this,tr("Error:"),tr("Please stop CDPR first!"));

}

void MainWindow::on_positionmode_radioButton_toggled()
{
    if(ui->positionmode_radioButton->isChecked())
    {
        ui->trjplan_groupBox->setEnabled(true);
    }
    else
    {
        ui->trjplan_groupBox->setEnabled(false);
    }
}


void MainWindow::on_remotemode_radioButton_toggled()
{
    if(ui->remotemode_radioButton->isChecked())
    {

    }
    else
    {remote_dialog.close();}
}

void MainWindow::WritePIDtoEdit()
{
    ui->Mot_1_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[0][0],'f',2));
    ui->Mot_2_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[1][0],'f',2));
    ui->Mot_3_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[2][0],'f',2));
    ui->Mot_4_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[3][0],'f',2));
    ui->Mot_5_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[4][0],'f',2));
    ui->Mot_6_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[5][0],'f',2));
    ui->Mot_7_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[6][0],'f',2));
    ui->Mot_8_Kp_lineEdit->setText(QString::number(CDPR_Force_PID[7][0],'f',2));
    ui->Mot_1_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[0][1],'f',2));
    ui->Mot_2_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[1][1],'f',2));
    ui->Mot_3_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[2][1],'f',2));
    ui->Mot_4_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[3][1],'f',2));
    ui->Mot_5_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[4][1],'f',2));
    ui->Mot_6_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[5][1],'f',2));
    ui->Mot_7_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[6][1],'f',2));
    ui->Mot_8_Ki_lineEdit->setText(QString::number(CDPR_Force_PID[7][1],'f',2));
    ui->Mot_1_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[0][2],'f',2));
    ui->Mot_2_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[1][2],'f',2));
    ui->Mot_3_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[2][2],'f',2));
    ui->Mot_4_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[3][2],'f',2));
    ui->Mot_5_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[4][2],'f',2));
    ui->Mot_6_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[5][2],'f',2));
    ui->Mot_7_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[6][2],'f',2));
    ui->Mot_8_Kd_lineEdit->setText(QString::number(CDPR_Force_PID[7][2],'f',2));
}

void MainWindow::WritePIDtoIni()
{
    QSettings *ParaConfigIniWrite = new QSettings("para.ini",QSettings::IniFormat);

    QString str;
    ParaConfigIniWrite->setValue("/para_ip/KP_1",str.number(CDPR_Force_PID[0][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_2",str.number(CDPR_Force_PID[1][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_3",str.number(CDPR_Force_PID[2][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_4",str.number(CDPR_Force_PID[3][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_5",str.number(CDPR_Force_PID[4][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_6",str.number(CDPR_Force_PID[5][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_7",str.number(CDPR_Force_PID[6][0]));
    ParaConfigIniWrite->setValue("/para_ip/KP_8",str.number(CDPR_Force_PID[7][0]));
    ParaConfigIniWrite->setValue("/para_ip/KI_1",str.number(CDPR_Force_PID[0][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_2",str.number(CDPR_Force_PID[1][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_3",str.number(CDPR_Force_PID[2][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_4",str.number(CDPR_Force_PID[3][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_5",str.number(CDPR_Force_PID[4][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_6",str.number(CDPR_Force_PID[5][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_7",str.number(CDPR_Force_PID[6][1]));
    ParaConfigIniWrite->setValue("/para_ip/KI_8",str.number(CDPR_Force_PID[7][1]));
    ParaConfigIniWrite->setValue("/para_ip/KD_1",str.number(CDPR_Force_PID[0][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_2",str.number(CDPR_Force_PID[1][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_3",str.number(CDPR_Force_PID[2][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_4",str.number(CDPR_Force_PID[3][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_5",str.number(CDPR_Force_PID[4][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_6",str.number(CDPR_Force_PID[5][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_7",str.number(CDPR_Force_PID[6][2]));
    ParaConfigIniWrite->setValue("/para_ip/KD_8",str.number(CDPR_Force_PID[7][2]));

    delete ParaConfigIniWrite;

}

void MainWindow::ReadPIDFromIni()//From para.ini;
{
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);


    CDPR_Force_PID[0][0] = ParaConfigIniRead->value("/para_ip/KP_1").toFloat();
    CDPR_Force_PID[1][0] = ParaConfigIniRead->value("/para_ip/KP_2").toFloat();
    CDPR_Force_PID[2][0] = ParaConfigIniRead->value("/para_ip/KP_3").toFloat();
    CDPR_Force_PID[3][0] = ParaConfigIniRead->value("/para_ip/KP_4").toFloat();
    CDPR_Force_PID[4][0] = ParaConfigIniRead->value("/para_ip/KP_5").toFloat();
    CDPR_Force_PID[5][0] = ParaConfigIniRead->value("/para_ip/KP_6").toFloat();
    CDPR_Force_PID[6][0] = ParaConfigIniRead->value("/para_ip/KP_7").toFloat();
    CDPR_Force_PID[7][0] = ParaConfigIniRead->value("/para_ip/KP_8").toFloat();
    CDPR_Force_PID[0][1] = ParaConfigIniRead->value("/para_ip/KI_1").toFloat();
    CDPR_Force_PID[1][1] = ParaConfigIniRead->value("/para_ip/KI_2").toFloat();
    CDPR_Force_PID[2][1] = ParaConfigIniRead->value("/para_ip/KI_3").toFloat();
    CDPR_Force_PID[3][1] = ParaConfigIniRead->value("/para_ip/KI_4").toFloat();
    CDPR_Force_PID[4][1] = ParaConfigIniRead->value("/para_ip/KI_5").toFloat();
    CDPR_Force_PID[5][1] = ParaConfigIniRead->value("/para_ip/KI_6").toFloat();
    CDPR_Force_PID[6][1] = ParaConfigIniRead->value("/para_ip/KI_7").toFloat();
    CDPR_Force_PID[7][1] = ParaConfigIniRead->value("/para_ip/KI_8").toFloat();
    CDPR_Force_PID[0][2] = ParaConfigIniRead->value("/para_ip/KD_1").toFloat();
    CDPR_Force_PID[1][2] = ParaConfigIniRead->value("/para_ip/KD_2").toFloat();
    CDPR_Force_PID[2][2] = ParaConfigIniRead->value("/para_ip/KD_3").toFloat();
    CDPR_Force_PID[3][2] = ParaConfigIniRead->value("/para_ip/KD_4").toFloat();
    CDPR_Force_PID[4][2] = ParaConfigIniRead->value("/para_ip/KD_5").toFloat();
    CDPR_Force_PID[5][2] = ParaConfigIniRead->value("/para_ip/KD_6").toFloat();
    CDPR_Force_PID[6][2] = ParaConfigIniRead->value("/para_ip/KD_7").toFloat();
    CDPR_Force_PID[7][2] = ParaConfigIniRead->value("/para_ip/KD_8").toFloat();
    delete ParaConfigIniRead;

}

void MainWindow::ReadPIDFromEdit(void)
{

    CDPR_Force_PID[0][0] = ui->Mot_1_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[1][0] = ui->Mot_2_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[2][0] = ui->Mot_3_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[3][0] = ui->Mot_4_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[4][0] = ui->Mot_5_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[5][0] = ui->Mot_6_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[6][0] = ui->Mot_7_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[7][0] = ui->Mot_8_Kp_lineEdit->text().toFloat();
    CDPR_Force_PID[0][1] = ui->Mot_1_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[1][1] = ui->Mot_2_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[2][1] = ui->Mot_3_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[3][1] = ui->Mot_4_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[4][1] = ui->Mot_5_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[5][1] = ui->Mot_6_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[6][1] = ui->Mot_7_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[7][1] = ui->Mot_8_Ki_lineEdit->text().toFloat();
    CDPR_Force_PID[0][2] = ui->Mot_1_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[1][2] = ui->Mot_2_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[2][2] = ui->Mot_3_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[3][2] = ui->Mot_4_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[4][2] = ui->Mot_5_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[5][2] = ui->Mot_6_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[6][2] = ui->Mot_7_Kd_lineEdit->text().toFloat();
    CDPR_Force_PID[7][2] = ui->Mot_8_Kd_lineEdit->text().toFloat();
}
void MainWindow::on_set_pid_pushButton_clicked()
{
    PIDParaConfDisplayFlag = 0;
    //save to para.ini
    ReadPIDFromEdit();
    WritePIDtoEdit();
    //    WritePIDtoIni();

    PIDParaConfig();//send to stm32;
}


void MainWindow::ReadTtoVFromEdit(void)
{
    TtoV[0]=ui->TtoV1_lineEdit->text().toFloat();
    TtoV[1]=ui->TtoV2_lineEdit->text().toFloat();
    TtoV[2]=ui->TtoV3_lineEdit->text().toFloat();
    TtoV[3]=ui->TtoV4_lineEdit->text().toFloat();
    TtoV[4]=ui->TtoV5_lineEdit->text().toFloat();
    TtoV[5]=ui->TtoV6_lineEdit->text().toFloat();
    TtoV[6]=ui->TtoV7_lineEdit->text().toFloat();
    TtoV[7]=ui->TtoV8_lineEdit->text().toFloat();
}

void MainWindow::ReadTtoVFromIni(void)
{
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);

    TtoV[0] = ParaConfigIniRead->value("/para_ip/TtoV1").toFloat();
    TtoV[1] = ParaConfigIniRead->value("/para_ip/TtoV2").toFloat();
    TtoV[2] = ParaConfigIniRead->value("/para_ip/TtoV3").toFloat();
    TtoV[3] = ParaConfigIniRead->value("/para_ip/TtoV4").toFloat();
    TtoV[4] = ParaConfigIniRead->value("/para_ip/TtoV5").toFloat();
    TtoV[5] = ParaConfigIniRead->value("/para_ip/TtoV6").toFloat();
    TtoV[6] = ParaConfigIniRead->value("/para_ip/TtoV7").toFloat();
    TtoV[7] = ParaConfigIniRead->value("/para_ip/TtoV8").toFloat();

    delete ParaConfigIniRead;
}


void MainWindow:: ReadPositionDialogState(void)
{
    bool joint_mode[8]={0};
    float costant_force;
    float a_max,v_max;

    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);

    joint_mode[0] = ParaConfigIniRead->value("/para_ip/PositionDialogState_1").toFloat();
    joint_mode[1] = ParaConfigIniRead->value("/para_ip/PositionDialogState_2").toFloat();
    joint_mode[2] = ParaConfigIniRead->value("/para_ip/PositionDialogState_3").toFloat();
    joint_mode[3] = ParaConfigIniRead->value("/para_ip/PositionDialogState_4").toFloat();
    joint_mode[4] = ParaConfigIniRead->value("/para_ip/PositionDialogState_5").toFloat();
    joint_mode[5] = ParaConfigIniRead->value("/para_ip/PositionDialogState_6").toFloat();
    joint_mode[6] = ParaConfigIniRead->value("/para_ip/PositionDialogState_7").toFloat();
    joint_mode[7] = ParaConfigIniRead->value("/para_ip/PositionDialogState_8").toFloat();

    costant_force = ParaConfigIniRead->value("/para_ip/PositionDialogState_constant_force").toFloat();
    a_max =  ParaConfigIniRead->value("/para_ip/PositionDialogState_a_max").toFloat();
    v_max =  ParaConfigIniRead->value("/para_ip/PositionDialogState_v_max").toFloat();

    delete ParaConfigIniRead;
    my_position_mode_set_dialog.set_joint_mode_from_ini(joint_mode);
    my_position_mode_set_dialog.set_constant_fore_from_ini(costant_force);
    my_position_mode_set_dialog.setLine_MaxA(a_max);
    my_position_mode_set_dialog.setLine_MaxV(v_max);
}

void MainWindow::ReadComplaintDialogState(void)
{
    float stiffness[6],damp[6];
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);

    stiffness[0] = ParaConfigIniRead->value("/para_ip/Stiffness_X").toFloat();
    stiffness[1] = ParaConfigIniRead->value("/para_ip/Stiffness_Y").toFloat();
    stiffness[2] = ParaConfigIniRead->value("/para_ip/Stiffness_Z").toFloat();
    stiffness[3] = ParaConfigIniRead->value("/para_ip/Stiffness_Rx").toFloat();
    stiffness[4] = ParaConfigIniRead->value("/para_ip/Stiffness_Ry").toFloat();
    stiffness[5] = ParaConfigIniRead->value("/para_ip/Stiffness_Rz").toFloat();

    damp[0] = ParaConfigIniRead->value("/para_ip/Damp_X").toFloat();
    damp[1] = ParaConfigIniRead->value("/para_ip/Damp_Y").toFloat();
    damp[2] = ParaConfigIniRead->value("/para_ip/Damp_Z").toFloat();
    damp[3] = ParaConfigIniRead->value("/para_ip/Damp_Rx").toFloat();
    damp[4] = ParaConfigIniRead->value("/para_ip/Damp_Ry").toFloat();
    damp[5] = ParaConfigIniRead->value("/para_ip/Damp_Rz").toFloat();

    delete ParaConfigIniRead;
    my_complaint_mode_set_dialog.set_complaint_stiffness_from_ini(stiffness);
    my_complaint_mode_set_dialog.set_complaint_damp_from_ini(damp);
}

void MainWindow::StateCheckSet_dialog(void)
{
    QSettings *ParaConfigIniRead = new QSettings("para.ini", QSettings::IniFormat);

    my_statecheckset_dialog.set_DriverOnlineCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_DriverOnlineCheck").toFloat());
    my_statecheckset_dialog.set_ErrorOverCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_ErrorOverCheck").toFloat());
    my_statecheckset_dialog.set_CableOverTightCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_CableOverTightCheck").toFloat());
    my_statecheckset_dialog.set_CableOverLoseCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_CableOverLoseCheck").toFloat());
    my_statecheckset_dialog.set_VelocityOverCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_VelocityOverCheck").toFloat());
    my_statecheckset_dialog.set_CurrentOverCheckSwitch_from_ini(ParaConfigIniRead->value("/para_ip/check_set_CurrentOverCheck").toFloat());

    StateCheckBuff = (my_statecheckset_dialog.get_DriverOnlineCheckSwitch()) |\
            (my_statecheckset_dialog.get_ErrorOverCheckSwitch() << 1) | \
            (my_statecheckset_dialog.get_CableOverTightCheckSwitch() << 2) |\
            (my_statecheckset_dialog.get_CableOverLoseCheckSwitch() << 3) |\
            (my_statecheckset_dialog.get_VelocityOverCheckSwitch() << 4) |\
            (my_statecheckset_dialog.get_CurrentOverCheckSwitch() << 5);
    delete ParaConfigIniRead;
}


void MainWindow::WriteTtoVtoIni(void)
{
    QSettings *ParaConfigIniWrite = new QSettings("para.ini",QSettings::IniFormat);

    QString str;

    ParaConfigIniWrite->setValue("/para_ip/TtoV1",str.number(TtoV[0]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV2",str.number(TtoV[1]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV3",str.number(TtoV[2]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV4",str.number(TtoV[3]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV5",str.number(TtoV[4]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV6",str.number(TtoV[5]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV7",str.number(TtoV[6]));
    ParaConfigIniWrite->setValue("/para_ip/TtoV8",str.number(TtoV[7]));

    delete ParaConfigIniWrite;
}

void MainWindow::WriteTtoVtoEdit(void)
{
    ui->TtoV1_lineEdit->setText(QString::number(TtoV[0],'f',2));
    ui->TtoV2_lineEdit->setText(QString::number(TtoV[1],'f',2));
    ui->TtoV3_lineEdit->setText(QString::number(TtoV[2],'f',2));
    ui->TtoV4_lineEdit->setText(QString::number(TtoV[3],'f',2));
    ui->TtoV5_lineEdit->setText(QString::number(TtoV[4],'f',2));
    ui->TtoV6_lineEdit->setText(QString::number(TtoV[5],'f',2));
    ui->TtoV7_lineEdit->setText(QString::number(TtoV[6],'f',2));
    ui->TtoV8_lineEdit->setText(QString::number(TtoV[7],'f',2));
}


void MainWindow::on_TtoV_set_pushButton_clicked()
{
    TtoVParaConfDisplayFlag = 0;
    ReadTtoVFromEdit();
    WriteTtoVtoEdit();
    //    WriteTtoVtoIni();

    TtoVParaConfig();
}

void MainWindow::on_clean_pushButton_clicked()
{
    ui->textEdit->setPlainText("");
}



void MainWindow::on_position_mode_set_toolButton_clicked()
{
    my_position_mode_set_dialog.set_dialog_state();
    my_position_mode_set_dialog.show();
}

void MainWindow::on_complaint_mode_toolButton_clicked()
{
    if(ui->forcemode_radioButton->isChecked())
    {
        my_complaint_mode_set_dialog.set_dialog_state();
        my_complaint_mode_set_dialog.show();
    }
}


void MainWindow::on_Tesion_zero_pushButton_clicked()
{
    TtoV_zeroParaConfDisplayFlag = 0;

    TtoV_zeroParaConfig();
}

void MainWindow::on_Dia_set_pushButton_clicked()
{
    DiaParaConfDisplayFlag = 0;
    ReadDiaFromEdit();
    WriteDiatoEdit();

    DiaParaConfig();
}

void MainWindow::on_Dm_set_pushButton_clicked()
{
    DmParaConfDisplayFlag = 0;
    ReadDmFromEdit();
    WriteDmtoEdit();
    DmParaConfig();
}

void MainWindow::on_E_set_pushButton_clicked()
{
    EParaConfDisplayFlag = 0;
    ReadEFromEdit();
    WriteEtoEdit();
    EParaConfig();
}

void MainWindow::on_Dc_set_pushButton_clicked()
{
    DcParaConfDisplayFlag = 0;
    ReadDcFromEdit();
    WriteDctoEdit();
    DcParaConfig();
}

void MainWindow::on_open_txt_pushButton_clicked()
{
    TrjFileName = QFileDialog::getOpenFileName(
                this,
                "Open Document",
                QDir::currentPath(),
                "Document files (*.txt);;All files(*.*)");
    if (!TrjFileName.isNull()) { //用户选择了文件
        // 处理文件
        TrjFileChooseFlag = true;
        //       cout << TrjFileName.toStdString().data() <<endl;
        QMessageBox::information(this, "Document", "Has document", QMessageBox::Ok | QMessageBox::Cancel);
    } else // 用户取消选择
        QMessageBox::information(this, "Document", "No document", QMessageBox::Ok | QMessageBox::Cancel);
}

void MainWindow::on_forcemode_radioButton_toggled()
{
    if(ui->forcemode_radioButton->isChecked())
    {
        ui->trjplan_groupBox->setEnabled(true);
    }
    else
    {
        ui->forcemode_radioButton->setEnabled(false);
    }
}

void MainWindow::on_statecheckset_toolButton_clicked()
{
    my_statecheckset_dialog.set_dialog_state();
    my_statecheckset_dialog.show();
}

void MainWindow::on_complaint_set_pushButton_clicked()
{
    if(ComplainceMode == 1)
    {
        ComplainceMode =0;
        ui->complaint_set_pushButton->setText("set");
        serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
        ControlModeSet(PositionModeSet);
    }
    else
    {
        ComplainceMode = 1;
        ui->complaint_set_pushButton->setText("close");
        serial_command_.setControlModeSetOkFlag(false);//waitting stm32 set control mode.
        ControlModeSet(ComplainceModeSet);
        MovingCommandSendEnableFlag = true;
    }
}

void MainWindow::on_autozeroring_pushButton_clicked()
{
    bool sendflag = 0;
    ControlModeSet(ComplainceModeSet);
    float Force_send[8] = {5,5.5,8.0,8.2,18,18.5,16.0,17.5};
    while(!sendflag)//judge if sended tesion command.
    {
        if(serial_command_.getControlModeSetOkFlag())//waitting stm32 initial drivers.
        {
            for(short i=0;i<8;i++)
            {
                serial_command_.my_command_forcecontrol_mode(i+1,Force_send[i]);
                QThread:: msleep(2);//waitting 2ms.
            }
            cout <<"Auto tesion set OK!"<<endl;

            sendflag = 1;
        }
    }
}


void MainWindow::on_PrintForce_checkBox_toggled(bool checked)
{
    if(checked)
    {
        ForcePrintFlag = 1;
        ForcePrintID = getRadioID();
    }
    else
    {
        ForcePrintFlag = 0;
    }
}

void MainWindow::on_teaching_ok_pushButton_clicked()
{
    TeachingMovingFlag = false;
}
