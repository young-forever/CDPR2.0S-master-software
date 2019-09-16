#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "mycommand.h"
#include <QObject>
#include<QDebug>
#include <QTimerEvent>
#include "trj_plan.h"
#include "remote_dialog.h"
#include "position_mode_set_dialog.h"
#include "complaint_mode_set_dialog.h"
#include "threadfromqthread.h"
#include "statecheckset_dialog.h"

#include<Eigen/Dense>
using namespace Eigen;

typedef enum
{
    SglJointMode_e,
    TrjMode_e,
    StopMode_e,
}MovingMode_e;

#define PI 3.14159265354

extern  bool ThreadStartFlag;

extern char StateCheckBuff;
extern bool ForcePrintFlag;
extern short ForcePrintID;

extern double bp[4][8];
extern double ep[4][8];
extern double dia[8];
extern float Dm[8];//diameter of measure wheel.
extern float L0[8];
extern float Dc;
extern float E;
extern double pulleyRad[8];
extern float TtoV[8];

extern float MinCalbleForce;

extern float Ks[6][6];
extern float Bs[6][6];

extern float CDPR_Force_PID[8][3];//CDPR Force pid paraments.
extern bool ConnectOKDisplayFlag;
extern bool PIDParaConfDisplayFlag;
extern bool TtoVParaConfDisplayFlag;
extern bool TtoV_zeroParaConfDisplayFlag;
extern bool DiaParaConfDisplayFlag;
extern bool DmParaConfDisplayFlag;
extern bool DcParaConfDisplayFlag;
extern bool L0ParaConfDisplayFlag;
extern bool EParaConfDisplayFlag;

extern bool ExpectAngleUpdateFlag;
extern bool RealAngleUpdateFlag;
extern bool ExpectPoseUpdateFlag;
extern bool RealPoseUpdateFlag;

extern bool SerialConSucDisFlag;
extern bool TrjCompleteDisplayFlag;
extern bool ErrorDisplayFlag;

extern bool TeachingMovingFlag;
extern bool ComplainceMode;//complaint control mode flag.
extern bool SglPoseComplaintMode;//single point complaint mode.

#define TIMER_TIMEOUT	(1)//unit ms

//ThreadFromQThread* m_thread;

class QTimer;
namespace Ui {
class MainWindow;//
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    Ui::MainWindow *ui;
    my_command serial_command_;
    Trjplan trjplan;
    QTimer *m_pTimer;
    ThreadFromQThread* m_thread;
    QString TrjFileName;
    MatrixXf TrjPoseBuff;

    bool TrjFileChooseFlag;//already choose txt file.
    bool FileOpenFlag;//txt file open flag.
    int LineNum;
    QString TrjType;

    bool L0UpdateFlag;
    bool zeringOKflag;
    bool StartPointOKFlag;
    bool TrjMovingOKFlag;
    bool RemoteMovingOKFlag;


    bool MovingCompleteFlag;
    bool MovingCommandSendEnableFlag;
    bool InterpolationOKFlag;
    bool ZeroToStartTrjFlag;
    bool BackToZero;
    bool BackToStart;

    bool SglJointMovingFlag;
    short SglJointDirFlag;

    bool TrjMovingFlag;//include posion and complaint mode.
    bool RemoteMovingFlag;
    bool AutobackTrjFlag;
    bool ZeroToStartMovingFlag;

    //task
    bool diplayflag;
    bool controlflag;
    int display_time ;
    int display_time_reload;
    short PCtoFC_cnt;
    int control_time ;
    int control_time_reload;

    int ThreadStartTime;
    int ThreadStartTimeReload;

    void UI_refresh();
    void MessageShow();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    Remote_Dialog remote_dialog;
    position_mode_set_dialog my_position_mode_set_dialog;
    complaint_mode_set_dialog my_complaint_mode_set_dialog;
    statecheckset_dialog my_statecheckset_dialog;

    void AllControlFlagInit();
    void ZeroingInit();
    void ControlModeSet(const char mode);

    void UiInit();
    void SerialInit();
    void HandleTimeOutInit();
    void ParaReadInit();//read all cdpr para from ini.
    void TrickInit();//slider initial

    void SlaveCompComm(void);//waitting comm to stm32.
    void PIDParaConfig(void);//config important paraments.
    void TtoVParaConfig(void);
    void TtoV_zeroParaConfig(void);
    void DiaParaConfig(void);
    void DmParaConfig(void);
    void L0ParaConfig(void);
    void DcParaConfig(void);
    void EParaConfig(void);

    bool isEndOnP_Zero();
    bool isEndOnP_Start();
    bool isNearP_Zero();

    //trj:
    void M_MatrixCal(char InterType, MatrixXf PoseBuff, float v_max, float a_max);
    void PutPoseToBuff(double *pose, const short col);
    bool GetTrjFromFileToBuff();
    void TrjRelolver(const QString str, double *temppose);

    short getRadioID(void);
    void SglJointMode();
    void AutoBackMode();
    void ZeroToStartMode();
    void PositionControlMode();
    void ComplaintControlMode();
    void TrjExpectAngleCal();
    void TrjControlMode();
    void RemoteControlMode();
    void MovingCommandSend();

    void controltask();
    void diplaytask();

    void TaskProcess();
    void TaskRemarks();

    //para read and write.
    void ReadDiaFromEdit(void);
    void WriteDiatoEdit(void);
    void ReadDmFromEdit(void);
    void WriteDmtoEdit(void);
    void ReadDcFromEdit(void);
    void WriteDctoEdit(void);
    void ReadEFromEdit(void);
    void WriteEtoEdit(void);

    void WriteStrucParaToEdit(void);
    void ReadStrucParaFromEdit(void);
    void WriteStrucParaToIni(void);
    void ReadStrucParaFromIni(void);

    void WriteZeroPoseToEdit(void);
    void ReadZeroPoseFromEdit(void);
    void WriteZeroPoseToIni(void);
    void ReadZeroPoseFromIni(void);

    void ReadPIDFromEdit(void);//
    void ReadPIDFromIni();//From para.ini;
    void WritePIDtoEdit();//read from Widget;
    void WritePIDtoIni();//save to para.ini.

    void ReadTtoVFromEdit(void);
    void ReadTtoVFromIni(void);
    void WriteTtoVtoIni(void);
    void WriteTtoVtoEdit(void);

    void ReadPositionDialogState(void);
    void ReadComplaintDialogState(void);
    void StateCheckSet_dialog(void);

private slots:
    void on_comboBox_currentIndexChanged(const QString &arg1);

    void my_serialread();
    void handleTimeout();

    void on_lose_cable_pushButton_pressed();
    void on_tight_cable_pushButton_pressed();
    void on_lose_cable_pushButton_released();
    void on_tight_cable_pushButton_released();
    void on_ForceTight_pushButton_clicked();//constant force mode button.

    void on_run_pushButton_clicked();       //running.
    void on_back_pushButton_clicked();      //autoback.
    void on_zero_set_button_clicked();      //zero initial.

    void on_ReadTxt_pushButton_clicked();   //read CDPR paraments txt.
    void on_SaveTxt_pushButton_clicked();   //save CDPR para.

    void on_save_pushButton_clicked();      //save zero position.

    void on_Reset_pushButton_clicked();     //reset CDPR state.

    void on_remotemode_radioButton_toggled();   //show remote dialog.
    void on_positionmode_radioButton_toggled(); //when select position mode, do this.
    void on_forcemode_radioButton_toggled();

    void on_set_pid_pushButton_clicked();

    void on_TtoV_set_pushButton_clicked();
    void on_clean_pushButton_clicked();
    void on_position_mode_set_toolButton_clicked();
    void on_complaint_mode_toolButton_clicked();
    void on_Tesion_zero_pushButton_clicked();
    void on_Dia_set_pushButton_clicked();
    void on_Dm_set_pushButton_clicked();
    void on_E_set_pushButton_clicked();
    void on_Dc_set_pushButton_clicked();
    void on_open_txt_pushButton_clicked();
    void on_statecheckset_toolButton_clicked();
    void on_complaint_set_pushButton_clicked();
    void on_autozeroring_pushButton_clicked();
    void on_PrintForce_checkBox_toggled(bool checked);
    void on_teaching_ok_pushButton_clicked();
};

#endif // MAINWINDOW_H
