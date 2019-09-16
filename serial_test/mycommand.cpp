#include<iostream>
#include<iomanip>
#include "mycommand.h"
#include "mainwindow.h"

using namespace std;

double ZeroAngle[8]={0.0};

float ExpectTesion[8] = {0.0};
float RealTesion[8] = {0.0};
double ExpectEndForce[6] = {0.0};
double RealEndForce[6] = {0.0};
double ExpectPose[6] = {0.0};
double RealPose[6] = {0.0};
double ExpectAngle[8] = {0.0};
float RealAngle[8] = {0.0};

bool ExpectAngleUpdateOk=false;
bool RealAngleUpdateOk=false;

RobotState :: RobotState()
{
    for(short i=0;i<8;i++){
//        ExpectAngle[i] =0.0;
        ExpectTesion[i]=0.0;
//        Angle[i]=0.0;
        Tesion[i]=0.0;
        MotCtrMode[i] = 0;
    }
    State = PREPARE;

    for(short i=0;i<16;i++){DataUpdataFlag[i] = 0;}

    SerialConSucFlag = 0;
    LastSerialConSucFlag = 0;
    ParaConfigFlag = 0;
    DriverOnlineFlag = 0;
    LastDriverOnlineFlag = 0;
    GetStartPositionFlag = 0;
}

RobotState :: ~RobotState()
{

}

void RobotState :: setflag(const short flag)
{
    switch (flag)
    {
    case 0x01:
        SerialConSucFlag=1;
        break;
    case 0x02:
        DriverOnlineFlag=1;
        break;
    case 0x12:
        LastDriverOnlineFlag=1;
        break;
    case 0x21:
        GetStartPositionFlag=1;
        break;
    case 0x03:
        ParaConfigFlag=1;
        break;
    }
}

void RobotState :: resetflag(const short flag)
{
    switch (flag)
    {
    //    case 0x00:
    //        DataUpdataFlag=0;
    //        break;
    case 0x01:
        SerialConSucFlag=0;
        break;
    case 0x02:
        DriverOnlineFlag=0;
        break;
    case 0x12:
        LastDriverOnlineFlag=0;
        break;
    case 0x21:
        GetStartPositionFlag=0;
        break;
    case 0x03:
        ParaConfigFlag=0;
        break;
    }
}

bool RobotState::getflag(const short flag)
{
    switch (flag)
    {
    case 0x01:
        return SerialConSucFlag;
        break;
    case 0x02:
        return DriverOnlineFlag;
        break;
    case 0x12:
        return LastDriverOnlineFlag;
        break;
    case 0x21:
        return GetStartPositionFlag;
        break;
    case 0x03:
        return ParaConfigFlag;
        break;

    default: return 0;
    }
}

void RobotState :: tansmitflag(const short inflag, const short putflag)
{
    if(getflag(inflag) == 0)
    {
        resetflag(putflag);
    }
    else
    {
        setflag(putflag);
    }

}

void RobotState ::setState(const short state)
{
    State=state;
}

short RobotState ::getState(void)
{
    return State;
}

void RobotState ::setMotCtrMode(const short CtrMode, const short &id)//set motor control mode.
{
    MotCtrMode[id-1] = CtrMode;
}

short RobotState ::getMotCtrMode(const short &id)//get motor control mode.
{
    return MotCtrMode[id-1];
}


short RobotState ::getDataupdateflag(const short &id)
{
    return DataUpdataFlag[id-1];

}

void RobotState ::setDataupdatatflag(const short &id)
{
    DataUpdataFlag[id-1] = 1;

}

void RobotState ::resetDataupdatatflag(const short &id)
{
    DataUpdataFlag[id-1] = 0;

}

//float RobotState :: getExpectAngle(const short &id)
//{
//    return ExpectAngle[id-1];
//}

//void RobotState :: setExpectAngle(const float &data, const short &id)
//{
//    ExpectAngle[id-1] = data;
//}

float RobotState :: getExpectTesion(const short &id)
{
    return ExpectTesion[id-1];
}

void RobotState :: setExpectTesion(const float &data, const short &id)
{
    ExpectTesion[id-1] = data;
}

//float RobotState :: getAngle(const short &id)
//{
//    return Angle[id-1];
//}

//void RobotState :: setAngle(const float &data, const short &id)
//{
//    Angle[id-1] = data;
//}

float RobotState :: getTesion(const short &id)
{
    return Tesion[id-1];
}

void RobotState :: setTesion(const float &data, const short &id)
{
    Tesion[id-1] = data;
}

//void RobotState ::getPose_ee(float (&pose)[6])
//{
//    for(int i=0;i<6;i++)
//    {
//        pose[i] = Pose_ee[i];
//    }

//}

//void RobotState :: setPose_ee(const float (&pose)[6])
//{
//    for(int i=0;i<6;i++)
//    {
//        Pose_ee[i]= pose[i];
//    }

//}
//void RobotState ::getExpectPose(float (&pose)[6])
//{
//    for(int i=0;i<6;i++)
//    {
//        pose[i] = ExpectPose[i];
//    }

//}

//void RobotState :: setExpectPose(const float (&pose)[6])
//{
//    for(int i=0;i<6;i++)
//    {
//        ExpectPose[i]= pose[i];
//    }

//}



//call this function in construted function.
QList<QSerialPortInfo> my_command::initSeialPort()
{
    //    connect(&serial,SIGNAL(readyRead()),this,SLOT(serialRead()));
    //信号和槽函数关联，当串口缓冲区有数据时，进行读串口操作
    //get name for choose
    QList<QSerialPortInfo>  infos = QSerialPortInfo::availablePorts();

    return infos;
}

QList<QSerialPortInfo> my_command::updateSeialPort(const QString &arg1)
{
    QSerialPortInfo info;
    QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    int i = 0;
    foreach (info, infos) {
        if(info.portName() == arg1) break;
        i++;
    }

    if(i != infos.size ()){//can find
        serial_.close();
        serial_.setPort(info);
        serial_.setBaudRate(2000000);
        serial_.open(QIODevice::ReadWrite);         //读写打开
    }
    else
    {
        serial_.close();
    }

    return infos;

}


char buffer[4096]={0x00};
int addr=0;
void my_command :: serialRead()
{
    char rebuffer[1280];

    QByteArray rebuffer_arry = serial_.readAll();//readAll() change to read(6);

    memcpy(rebuffer,rebuffer_arry,rebuffer_arry.size());
    memcpy(buffer+addr,rebuffer,sizeof(rebuffer));

    addr = addr +rebuffer_arry.size();

    QByteArray buff_arry = QByteArray(buffer,addr);

    while(buff_arry.size() > 6)//juge whether the
    {
        if(buff_arry.data()[0] == 0x0d)
        {
            buff_arry.remove(0,1);

            if(buff_arry.data()[0] == 0x0a)
            {
                unsigned char re_command = buff_arry[1];

                buff_arry.remove(0,2);

                QByteArray re_data = buff_arry.left(4);

                command_execute(re_command, re_data);

                buff_arry.remove(0,4);
            }
        }
        else
        {
            buff_arry.remove(0,1);
        }
    }

    addr = buff_arry.size();
    memcpy(buffer,buff_arry,addr);
}

void my_command::serialsend(const QByteArray &data)
{
    serial_.write(data);
}

bool my_command::getControlModeSetOkFlag()
{
    return ControlModeSetOkFlag;
}

void my_command::setControlModeSetOkFlag(const bool flag)
{
    ControlModeSetOkFlag = flag;
}

bool my_command ::getPIDRefreshFlag()
{
    return PIDRefreshFlag;
}

bool my_command ::getTtoVRefreshFlag()
{
    return TtoVRefreshFlag;
}

bool my_command ::getTtoV_zeroRefreshFlag()
{
    return TtoV_zeroRefreshFlag;
}

bool my_command ::getDiaRefreshFlag()
{
    return DiaRefreshFlag;
}

bool my_command ::getDmRefreshFlag()
{
    return DmRefreshFlag;
}

bool my_command ::getDcRefreshFlag()
{
    return DcRefreshFlag;
}

bool my_command ::getERefreshFlag()
{
    return ERefreshFlag;
}

void my_command ::setPIDRefreshFlag(const bool flag)
{
    PIDRefreshFlag = flag;
}

void  my_command ::setTtoVRefreshFlag(const bool flag)
{
    TtoVRefreshFlag = flag;
}

void my_command ::setTtoV_zeroRefreshFlag(const bool flag)
{
   TtoV_zeroRefreshFlag = flag;
}

void my_command ::setDiaRefreshFlag(const bool flag)
{
    DiaRefreshFlag = flag;
}

void my_command ::setDmRefreshFlag(const bool flag)
{
    DmRefreshFlag = flag;
}

void my_command ::setDcRefreshFlag(const bool flag)
{
    DcRefreshFlag = flag;
}

void my_command ::setERefreshFlag(const bool flag)
{
    ERefreshFlag = flag;
}

void my_command ::setState(const short state)
{
    state_.setState(state);
    my_command_stateset(state);
}

short my_command ::getState(void)
{
    return state_.getState();
}

void my_command ::setMotCtrMode(const short CtrMode, const short &id)//set motor control mode.
{
    state_.setMotCtrMode(CtrMode,id);
}
short my_command ::getMotCtrMode(const short &id)//get motor control mode.
{
    return state_.getMotCtrMode(id);
}

short my_command:: getDataupdateflag(const short &id)
{
    return state_.getDataupdateflag(id);
}

void my_command:: setDataupdatatflag(const short &id)
{
    state_.setDataupdatatflag(id);
}

void my_command:: resetDataupdatatflag(const short &id)
{
    state_.resetDataupdatatflag(id);
}


//float my_command:: getExpectAngle(const short &id)
//{
//    return state_.getExpectAngle(id);
//}

//void my_command:: setExpectAngle(const float &data, const short &id)
//{
//    state_.setExpectAngle(data,id);
//}

float my_command:: getExpectTesion(const short &id)
{
    return state_.getExpectTesion(id);
}

void my_command:: setExpectTesion(const float &data, const short &id)
{
    state_.setExpectTesion(data,id);
}

//float my_command:: getAngle(const short &id)
//{
//    return state_.getAngle(id);
//}

//void my_command:: setAngle(const float &data, const short &id)
//{
//    state_.setAngle(data,id);

//}

float my_command:: getTesion(const short &id)
{
    return state_.getTesion(id);

}

void my_command:: setTesion(const float &data, const short &id)
{

    state_.setTesion(data,id);
}

//void my_command:: getPose_ee(float (&pose)[6])
//{
//    state_.getPose_ee(pose);
//}

//void my_command:: setPose_ee(const float (&pose)[6])
//{
//    state_.setPose_ee(pose);
//}

//void my_command:: getExpectPose(float (&pose)[6])
//{
//    state_.getExpectPose(pose);
//}

//void my_command:: setExpectPose(const float (&pose)[6])
//{
//    state_.setExpectPose(pose);
//}

void my_command::  setflag(const short flag)
{
    return state_.setflag(flag);
}

void my_command::  resetflag(const short flag)
{
    return state_.resetflag(flag);
}

bool my_command:: getflag(const short flag)
{
    return state_.getflag(flag);
}

void my_command::  tansmitflag(const short inflag, const short putflag)
{
    state_.tansmitflag(inflag,putflag);
}


void my_command::serialclose()
{
    serial_.close();
}

my_command :: my_command()
{
//    state_.setflag(serialconsucf);

    PIDRefreshFlag = false;
    TtoVRefreshFlag = false;
    TtoV_zeroRefreshFlag = false;
    DiaRefreshFlag = false;
    DmRefreshFlag = false;
    DcRefreshFlag = false;
    ERefreshFlag = false;
    error_flag = false;

    QByteArray rebuffer_arry = serial_.readAll();//readAll();
}

my_command :: ~my_command()
{

}

void my_command::command_execute(unsigned char re_command, QByteArray re_data)
{
    int data=0;
    float pdata=0;
    short id;

    switch(re_command & 0x0f)
    {
    case 0x00:  //serial port connect ok.
        SerialConSucDisFlag = 0;

//        if(!state_.getflag(serialconsucf))
//        {
            my_command_consucf(); //answer back connect ok command.
            state_.setflag(serialconsucf);
//        }

        break;

    case 0x01:  //
        if(re_command == 0xc1)
        {
            ControlModeSetOkFlag = true;
        }
        break;

    case 0x02:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        CDPR_Force_PID[id-1][0] = pdata;
        break;

    case 0x03:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        CDPR_Force_PID[id-1][1] = pdata;
        break;

    case 0x04:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        CDPR_Force_PID[id-1][2] = pdata;
        if(id == 8)
        {
            PIDRefreshFlag =true;
        }
        break;

    case 0x05:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        TtoV[id-1] = pdata;
        if(id == 8)
        {
            TtoVRefreshFlag =true;
        }
        break;

    case 0x07:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        dia[id-1] = pdata/1000.0;
        if(id == 8)
        {
            DiaRefreshFlag =true;
        }
        break;

    case 0x08:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        Dm[id-1] = pdata/1000.0;
        if(id == 8)
        {
            DmRefreshFlag =true;
        }
        break;

    case 0x09:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0f;
        if(id<=8)
        {
            L0[id-1] = pdata;
        }
        else if(id == 0x0D)
        {
            Dc = pdata/1000.0;
            DcRefreshFlag = true;
        }
        else if(id == 0x0E)
        {
            E = data/100000.0f*(1E+9);
            ERefreshFlag = true;
        }
        break;
    case 0x0B:
        if(re_command == 0XEB)
        {
            error_id = re_data[0];
            m_errorcode = re_data[1];
            error_flag = true;
//            cout << "error_code:"<<m_errorcode << endl;
        }
        break;

        //recived angle of CDPR.
    case 0x0C:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);        //
        pdata = data/100000.0f;
        RealAngle[id-1] = pdata;

        if(id == 8)
        {
            RealAngleUpdateFlag = true;
        }
        state_.setDataupdatatflag(1);
//        if(getMotCtrMode(id)==1 & getState() == RUNNING)//if force mode
//        {
//            ExpectAngle[id-1] = RealAngle[id-1];
//        }
        break;

        //recive data of tesion.
    case 0x0f:
        id = (re_command & 0xf0) >> 4;
        memcpy(&data,re_data,4);
        pdata = data/100000.0;
        RealTesion[id-1] = pdata;
        if(ForcePrintFlag == 1 && id == ForcePrintID)
        {
            cout <<"Tesion "<<ForcePrintID<<" :"<<RealTesion[id-1]<<endl;
        }
        break;
    }

}



void my_command :: my_command_consucf()   //connect successful command;
{
    char data[7];

    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2] = 0x00;
    data[3] = StateCheckBuff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;

    serial_.write(data,7);
}

void my_command :: my_command_kpset(const short &id, const float Kp)
{
    char data[7];
    int temp;
    temp = Kp*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x02;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);

}

void my_command :: my_command_kiset(const short &id, const float Ki)
{
    char data[7];
    int temp;
    temp = Ki*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x03;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command :: my_command_kdset(const short &id, const float Kd)
{
    char data[7];
    int temp;
    temp = Kd*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x04;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void  my_command :: my_command_TtoVset(const short &id, const float iTtoV)
{
    char data[7];
    int temp;
    temp = iTtoV*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x05;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);

}

void my_command ::my_command_TtoV_zeroset()
{
    char data[7];
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=0XB1;

    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;

    serial_.write(data,7);
}

void my_command ::my_command_Diaset(const short &id, const float iDia)
{
    char data[7];
    int temp;
    temp = iDia*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x07;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command ::my_command_Dmset(const short &id, const float iDm)
{
    char data[7];
    int temp;
    temp = iDm*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x08;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command ::my_command_L0set(const short &id, const float iL0)
{
    char data[7];
    int temp;
    temp = iL0*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x09;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command ::my_command_Dcset(const float iDc)
{
    char data[7];
    int temp;
    temp = iDc*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=0xd9;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command ::my_command_Eset(const float iE)
{
    char data[7];
    int temp;
    temp = iE*100000.0;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=0xe9;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}


void my_command :: my_command_paraconfigok(void)
{
    char data[7];

    data[0] = 0x0d;
    data[1] = 0x0a;

    data[2] = 0x02;
    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;

    serial_.write(data,7);
}

void my_command :: my_command_stateset(short state)  //cdpr state set command;
{
    char data[7];

    data[0] = 0x0d;
    data[1] = 0x0a;

    data[2] = 0x0A;
    data[3] = state;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;
    serial_.write(data,7);
}

//zero initial set command.
void my_command :: my_command_zeroset(void)
{
    char data[7];

    data[0] = 0x0d;
    data[1] = 0x0a;

    data[2] = 0x01;
    data[3] = 0xff;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;

    serial_.write(data,7);
}

//joint control mode set.
void my_command :: my_command_ControlModeSet(const char mode)
{
    char data[7];

    data[0] = 0x0d;
    data[1] = 0x0a;

    data[2] = 0xC1;

    data[3] = mode;
    data[4] = 0xff;
    data[5] = 0xff;
    data[6] = 0xff;

    serial_.write(data,7);
}


//    void my_command_adjustcalb(short id, float angle);//zero position adjustment;
void my_command::my_command_positoncontrol_mode(const short id, const double angle)//
{
    char data[7];
    int temp;
    temp = angle*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x0C;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}

void my_command::my_command_forcecontrol_mode(const short id, const float force)//
{
    char data[7];
    int temp;
    temp = force*100000;
    data[0] = 0x0d;
    data[1] = 0x0a;
    data[2]=id << 4 | 0x0F;

    data[3] = (char)(temp & 0xff);
    data[4] = (char)(temp >>8 & 0xff);
    data[5] = (char)(temp >>16 & 0xff);
    data[6] = (char)(temp >>24 & 0xff);

    serial_.write(data,7);
}


void my_command::my_command_special_order(const short order)//some special order like scram protection;
{


}


bool my_command::getErrorFlag()
{
    return error_flag;
}

bool my_command::setErrorFlag(bool flag)
{
    error_flag = flag;
}
void my_command::getErrorPara(short&id, short&errorcode)
{
    id = error_id;
    errorcode = m_errorcode;
}
