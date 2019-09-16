#ifndef MYCOMMAND_H
#define MYCOMMAND_H
#include <QSerialPort>
#include <QSerialPortInfo>

//CDPR state define:
#define PREPARE     0x0f
#define RUNNING     0x1f
#define STOP        0x2f
#define AUTOBACK    0x3f

#define serialconsucf           0x01
#define driveronlinef           0x02
#define lastdriveronlinef       0x12
#define paraconfigf             0x03
#define getstartpositionf       0x21

#define PositionModeSet     0x01
#define ComplainceModeSet    0x02

extern double ZeroAngle[8];


extern float ExpectTesion[8];
extern float RealTesion[8];
extern double ExpectEndForce[6];
extern double RealEndForce[6];
extern double ExpectPose[6];
extern double RealPose[6];
extern double ExpectAngle[8];
extern float RealAngle[8];

extern bool ExpectAngleUpdateOk;
extern bool RealAngleUpdateOk;

class RobotState
{
    short State; //prepare,running...
    short MotCtrMode[8]; //single motor control mode.=0 position mode; =1 force mode.

//    float ExpectAngle[8];
//    float Angle[8];     //real angle of joints.

    float ExpectTesion[8];


    float Tesion[8];    //real tesion of joints.

//    float Pose_ee[6];   //real x,y,z,alpha,beta,gama.
//    float ExpectPose[6];//expect pose.

    short DataUpdataFlag[16];    //1~8  angle; 9~16 tesion;
    bool SerialConSucFlag;
    bool LastSerialConSucFlag;
    bool ParaConfigFlag;
    bool DriverOnlineFlag;
    bool LastDriverOnlineFlag;
    bool GetStartPositionFlag;

public:
    void setState(const short state);   //set CDPR state.
    short getState(void);               //get CDPR state.

    void setMotCtrMode(const short CtrMode, const short &id);//set motor control mode.
    short getMotCtrMode(const short &id);//get motor control mode.

    short getDataupdateflag(const short &id);
    void setDataupdatatflag(const short &id);
    void resetDataupdatatflag(const short &id);

//    float getExpectAngle(const short &id);
//    void setExpectAngle(const float &data, const short &id);
//    float getAngle(const short &id);
//    void setAngle(const float &data, const short &id);
//    void getPose_ee(float (&pose)[6]);
//    void setPose_ee(const float (&pose)[6]);
//    void getExpectPose(float (&pose)[6]);
//    void setExpectPose(const float (&pose)[6]);

    float getExpectTesion(const short &id);
    void setExpectTesion(const float &data, const short &id);

    float getTesion(const short &id);
    void setTesion(const float &data, const short &id);

    void setflag(const short flag);
    void resetflag(const short flag);
    bool getflag(const short flag);
    void tansmitflag(const short inflag, const short putflag);

    RobotState();
    ~RobotState();
};

class my_command : public QObject
{
    //friend MainWindow;
    Q_OBJECT

    bool ControlModeSetOkFlag; //slaver stm32 control mode set ok flag.

    bool PIDRefreshFlag;
    bool TtoVRefreshFlag;
    bool TtoV_zeroRefreshFlag;
    bool DiaRefreshFlag;
    bool DmRefreshFlag;
    bool DcRefreshFlag;
    bool ERefreshFlag;

    short error_id;
    short m_errorcode;
    bool error_flag;

    RobotState state_;
    void command_execute(unsigned char re_command, QByteArray re_data);

public:
    my_command();
    ~my_command();

    QSerialPort serial_;
    QList<QSerialPortInfo> initSeialPort();
    QList<QSerialPortInfo> updateSeialPort(const QString &arg1);

public:
    bool getControlModeSetOkFlag();
    void setControlModeSetOkFlag(const bool flag);

    bool getPIDRefreshFlag();
    bool getTtoVRefreshFlag();
    bool getTtoV_zeroRefreshFlag();
    bool getDiaRefreshFlag();
    bool getDmRefreshFlag();
    bool getDcRefreshFlag();
    bool getERefreshFlag();
    void setPIDRefreshFlag(const bool flag);
    void setTtoVRefreshFlag(const bool flag);
    void setTtoV_zeroRefreshFlag(const bool flag);
    void setDiaRefreshFlag(const bool flag);
    void setDmRefreshFlag(const bool flag);
    void setDcRefreshFlag(const bool flag);
    void setERefreshFlag(const bool flag);

    void setState(const short state);
    short getState(void);

    void setMotCtrMode(const short CtrMode, const short &id);//set motor control mode.
    short getMotCtrMode(const short &id);//get motor control mode.

    short getDataupdateflag(const short &id);
    void setDataupdatatflag(const short &id);
    void resetDataupdatatflag(const short &id);

    float getExpectTesion(const short &id);
    void setExpectTesion(const float &data, const short &id);

    float getTesion(const short &id);
    void setTesion(const float &data, const short &id);

    void setflag(const short flag);
    void resetflag(const short flag);
    bool getflag(const short flag);
    void tansmitflag(const short inflag, const short putflag);

    void serialclose();
    void serialsend(const QByteArray &data);
    void serialRead();

    void my_command_consucf();   //connect successful command;
    void my_command_zeroset(void);
    void my_command_ControlModeSet(const char mode);

    void my_command_kpset(const short &id, const float Kp);
    void my_command_kiset(const short &id, const float Ki);
    void my_command_kdset(const short &id, const float Kd);
    void my_command_TtoVset(const short &id, const float iTtoV);
    void my_command_TtoV_zeroset();
    void my_command_Diaset(const short &id, const float iDia);
    void my_command_Dmset(const short &id, const float iDm);
    void my_command_L0set(const short &id, const float iL0);
    void my_command_Dcset(const float iDc);
    void my_command_Eset(const float iE);
    void my_command_paraconfigok(void);

    void my_command_stateset(short state);  //cdpr state set command;

    //    void my_command_adjustcalb(short id, float angle);//zero position adjustment;
    void my_command_positoncontrol_mode(const short id, const double angle);//
    void my_command_forcecontrol_mode(const short id, const float force);//
    void my_command_special_order(const short order);//some special order like scram protection;

    bool getErrorFlag();
    bool setErrorFlag(bool flag);
    void getErrorPara(short &id, short &errorcode);
};

#endif // MYCOMMAND_H
