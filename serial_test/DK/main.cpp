#include <iostream>
#include "widget.h"
#include <QApplication>
#include "DKofCDPR.h"     //Include direct kinematic function of CDPR.
#include "ParaofCDPR.h"   //Include para of CDPR.
#include <vector>
#include "cablelencal.h"
#include <QDebug>
#include <sys/time.h>

using namespace std;

double ActualPose[6]={0.0};                            //define end positon of CDPR.

int main(int argc, char *argv[])
{

    QApplication a(argc, argv); //can't remove

    //**DEFIEN GUI**//
//    QWidget MainWind;
//    MainWind.show();

    double MotorAngle[8] = {30.07,30.57,31.53,31.19,20.92,21.67,20.20,19.71};       //define rotation angle of joint

    GetParaofCDPR();        //get basic paramentes of CDPR.

//** check paramentes input and output.**//

//    int i,j;
//    cout << "bp is:" <<endl;

//    for(i=0;i<4;i++)
//    {
//        cout << "  ";
//        for(j=0;j<8;j++)
//        {
//           cout <<"\t"<< bp[i][j];
//        }
//        cout << ";" << endl;
//    }


//    cout << "ep is:" <<endl;
//    for(i=0;i<4;i++)
//    {
//        cout << "  ";
//        for(j=0;j<8;j++)
//        {
//           cout <<"\t"<< ep[i][j];
//        }
//        cout << ";" << endl;
//    }

//    cout << "dia is:" <<endl;
//    for(j=0;j<8;j++)
//    {
//       cout <<"\t"<< dia[j];
//    }
//    cout << ";" << endl;

//    cout << "pulley rad is:" <<endl;
//    cout <<"\t"<< pulleyRad;
//    cout << ";" << endl;

    double Pose[6]={0.0};

    struct timeval tpstart,tpend;
    float timeuse;

    gettimeofday(&tpstart,NULL);

    DKofCDPR(MotorAngle, Pose);   //input motorangle and output pose.

    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;

    qDebug()<<timeuse<<"s (timeuse of DK.)";

    for(int i=0;i<6;i++)
    cout << Pose[i]<<endl;

    return a.exec();
}
