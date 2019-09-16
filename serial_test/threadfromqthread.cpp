#include "threadfromqthread.h"
#include<Eigen/Dense>
#include "DKofCDPR.h"     //Include direct kinematic function of CDPR.
#include "mycommand.h"
#include "mainwindow.h"
#include "forcedistribute.h"
#include <iostream>
#include "mathtool.h"

//measure time of programe
//#include <sys/time.h>
//struct timeval tpstart,tpend;
//double timeuse;
//

using namespace std;

using namespace Eigen;


/*
函数名称：位姿误差求解函数
功能描述：输入两个位姿（姿态以xyz顺序欧拉角表示），输出可用于控制的误差位姿。参考徐文福《机器人学》第3章P25；
参数描述：P1，P2表示输入的位姿（数组地址指针），dP为输出的位姿误差（数组地址指针）；
其    他：调用需要包含Eigen库，位姿单位为m和rad。
*/
void PoseErrCal(double* P1, double* P2, double* dP)
{
    Matrix<double, 6, 1>  v_P1, v_P2;
    for (int i = 0; i < 6; i++)
    {
        v_P1(i, 0) = P1[i];
        v_P2(i, 0) = P2[i];
    }

    Matrix<double, 6, 1> d_EulerPose;
    d_EulerPose = v_P1 - v_P2;
    Matrix<double, 6, 6> J_Euler;
    J_Euler <<
        MatrixXd::Zero(6, 6);
    J_Euler(0, 0) = 1;
    J_Euler(1, 1) = 1;
    J_Euler(2, 2) = 1;
    J_Euler(3, 3) = 1;
    J_Euler(3, 5) = sin(v_P2(4, 0));
    J_Euler(4, 4) = cos(v_P2(3, 0));
    J_Euler(4, 5) = -sin(v_P2(3, 0))*cos(v_P2(4, 0));
    J_Euler(5, 4) = sin(v_P2(3, 0));
    J_Euler(5, 5) = cos(v_P2(3, 0))*cos(v_P2(4, 0));

    Matrix<double, 6, 1> d_Pose;
    d_Pose = J_Euler*d_EulerPose;

    for (int i = 0; i < 6; i++)
    {
        dP[i] = d_Pose(i, 0);
    }
}

ThreadFromQThread::ThreadFromQThread(QObject* par) : QThread(par)
{
    m_isCanRun = true;
}

ThreadFromQThread::~ThreadFromQThread()
{
}

void ThreadFromQThread::stopImmediately()
{
    QMutexLocker locker(&m_lock);
    m_isCanRun = false;
}

void ThreadFromQThread::run()
{
    double MotorAngle[8];
    double dPose_temp[3] = {0.0};
    static double Near3dPose[6][3] = {0.0};

    static VectorXf d_pose(6);
            d_pose<<0,0,0,0,0,0;
    static VectorXf dd_pose(6);
    static VectorXf d_pose_last(6);

    static  int cnt = 200000;
    while(ThreadStartFlag)
    {
        ThreadStartFlag = 0;

        if(cnt == 0)
        {
            cnt = 200000;
            cout<<"Thread running ...."<<endl;
        }
        cnt--;
//        gettimeofday(&tpstart,NULL);

//        if(ExpectAngleUpdateOk)

        if(ExpectAngleUpdateFlag == true)
        {
//            cout<<"falg: "<<ExpectAngleUpdateFlag<<endl;
            ExpectAngleUpdateFlag = false;
            for(int i=0;i<8;i++)
            {
                MotorAngle[i] = (ExpectAngle[i] + ZeroAngle[i])/180.0*PI;
            }
            DKofCDPR(MotorAngle, ExpectPose);   //input motorangle and output pose.

            ExpectPoseUpdateFlag = true;
        }

//        if(RealAngleUpdateOk)
        if(RealAngleUpdateFlag == true)
        {
            RealAngleUpdateFlag = false;
            for(int i=0;i<8;i++)
            {
                MotorAngle[i] = (RealAngle[i] + ZeroAngle[i])/180.0*PI;
            }
            DKofCDPR(MotorAngle, RealPose);   //input motorangle and output pose.

            RealPoseUpdateFlag = true;
        }

        //CAL EXPECT ENDFORCE
        if(ComplainceMode == 1 && RealPoseUpdateFlag == true)
//        if(1)
        {            
            ExpectPoseUpdateFlag = false;
            RealPoseUpdateFlag = false;

            double d_P[6]={0.0};
            PoseErrCal(ExpectPose,RealPose,d_P);

            if(TeachingMovingFlag == false)
            {

                for(short i=0;i<6;i++)
                {
                    d_pose_last[i] = d_pose(i);
                    d_pose(i) = d_P[i];
//                    dd_pose(i) = d_pose(i) - d_pose_last(i);
                }
                for(int i=0;i<6;i++)
                {
                    Near3dPose[i][0] = Near3dPose[i][1];
                    Near3dPose[i][1] = Near3dPose[i][2];
                    Near3dPose[i][2] = d_pose[i];
                }

                for(int i=0;i<6;i++)
                {
//                    cout<<"Near 3 pose "<<i+1<<" : ";

                    for(int j=0;j<3;j++)
                    {
                        dPose_temp[j] = Near3dPose[i][j];
//                        cout<<dPose_temp[j]<<"   ";
                    }
//                    cout<<endl;

                    dd_pose(i) = iNumDiff(dPose_temp,1.0);
                }

                MatrixXf K = MatrixXf::Zero(6,6);
                MatrixXf B = MatrixXf::Zero(6,6);
                for(short i=0;i<6;i++)
                {
                    K(i,i) = Ks[i][i];
                    B(i,i) = Bs[i][i];
                }

//                cout<<"d_Pose: ";
//                for(int i=0;i<6;i++)
//                {
//                    cout<<d_pose(i)<<"  ";
//                }
//                cout<<endl;

//                cout<<"dd_Pose: ";
//                for(int i=0;i<6;i++)
//                {
//                    cout<<dd_pose(i)<<"  ";
//                }
//                cout<<endl;

                ExpEndForceCal(d_pose,dd_pose,K,B,ExpectEndForce);//calculate end force.
            }
            else
            {
                float endforce[6]={0,0,5.0,0,0};
                for(short i=0;i<6;i++)
                {
                    ExpectPose[i] = RealPose[i];
                    ExpectEndForce[i] = endforce[i];
                }
            }

//            for(short i = 0;i<6;i++)
//            {
//                if(i==0)
//                {
//                    cout<<"ExpectEndForce: ";
//                }
//                cout<<ExpectEndForce[i]<<"\t";
//                if(i == 5)
//                {
//                    cout<<endl;
//                }
//            }
//            for(short i = 0;i<6;i++)
//            {
//                if(i==0)
//                {
//                    cout<<"RealPose: ";
//                }
//                cout<<RealPose[i]<<"\t";
//                if(i == 5)
//                {
//                    cout<<endl;
//                }
//            }

            EndForceDistrToJointCal(ExpectTesion);//calculate joint tesion.
            //print expect tesion.
//            for(short i = 0;i<8;i++)
//            {
//                if(i==0)
//                {
//                    cout<<"ExpectTesion: ";
//                }
//                cout<<ExpectTesion[i]<<"\t";
//                if(i == 7)
//                {
//                    cout<<endl;
//                }
//            }

//            cout << "ExpectEndForce:" <<endl;
//            for(short i=0;i<6;i++)
//            {
//                cout << ExpectEndForce[i]<< "; " << endl;
//            }

//            cout << "ExpectTesion: "<<endl;
//            for(short i=0;i<8;i++)
//            {
//                cout << ExpectTesion[i]<< "; " << endl;
//            }
        }

        if(1)//calculate realtime state of end force.
        {
            double testforce[6];
            JointTesionToEndCal(ExpectTesion,testforce);
            cout<<"test endforce: ";
            for(int i=0;i<6;i++)
            {
                cout<<testforce[i]<<"   ";
            }
            cout<<endl;
            JointTesionToEndCal(RealTesion,RealEndForce);

//            cout << "RealEndForce: "<<endl;
//            for(short i=0;i<6;i++)
//            {
//                cout << RealEndForce[i]<< "; " << endl;
//            }
        }

        QMutexLocker locker(&m_lock);

        if(!m_isCanRun)
        {
            return;
        }

//        gettimeofday(&tpend,NULL);
//        timeuse = (1000000*(tpend.tv_sec - tpstart.tv_sec) + tpend.tv_usec - tpstart.tv_usec)/1000000.0;
//        cout << "time use:"<<timeuse <<"s."<<endl;
    }
}
