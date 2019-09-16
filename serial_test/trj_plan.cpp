#include<iostream>
#include "trj_plan.h"
#include<Eigen/Core>
#include<iomanip>
#include <algorithm>
#include "mainwindow.h"
using namespace Eigen;
using namespace std;

Trjplan :: Trjplan()
{
    for(short i=0;i<6;i++)
    {
        P_zero[i]=0.0;      //store zero point of cdpr.
        P_start[i]=0.0;
        P0[i] = 0.0;
        Pt[i]=0.0;
    }

    P_start_zoffset =0;       //mm

    PM_control_cycle=50;        //ms
    trj_Num=0;
//    runningflag=0;

    node_distance=15;           //mm
    ContrlMode = PositionMode;   //define control mode of trajectory plan.
    M_Matrix.resize(8,100);//matrix of interpolation.

    PoseNode.resize(6,NoChange);           //
    Tplan.resize(NoChange);               //
//    AngleNodeZero.resize(8);                //store 8 joints angle value in zero posion
    AngleNode.resize(8,NoChange);          //

    MaxDataNum = 0;
    DataNum_i = 0;
}

Trjplan :: ~Trjplan()
{

}

//void Trjplan::setrunningfalg(const bool flag)
//{
//    runningflag=flag;
//}

//bool Trjplan::getruningflag()
//{
//    return runningflag;
//}

int  Trjplan::get_trj_num()
{
    return trj_Num;
}

void Trjplan::set_trj_num(const int trj_num)
{
    trj_Num=trj_num;
}

void Trjplan::setP_zero(const double (&p_zero)[6])
{
    for(short i=0;i<6;i++)
    {
        P_zero[i] = p_zero[i] ;
    }
}

void Trjplan::getP_zero(double (&p_zero)[6])
{
    for(short i=0;i<6;i++)
    {
        p_zero[i] = P_zero[i] ;
    }
}

void Trjplan::setP_start(const double (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
        P_start[i] = pose[i];
    }
}

void Trjplan::getP_start(double (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
        pose[i]=P_start[i] ;
    }
}

void Trjplan::setP_start_zoffset(const float &offset)
{
    P_start_zoffset=offset;

}

float Trjplan::getP_start_zoffset(void)
{
    float offset = P_start_zoffset;
    return offset;
}


void  Trjplan::setP0(const float (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
        P0[i] = pose[i];
    }

}
void  Trjplan::getP0(float (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
       pose[i]  = P0[i];
    }
}

void Trjplan::setPt(const float (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
        Pt[i] = pose[i];
    }
}

void Trjplan::getPt(float (&pose)[6])
{
    for(short i=0;i<6;i++)
    {
        pose[i] = Pt[i] ;
    }
}

void Trjplan::setControlMode(const short mode)
{


}

void Trjplan::setPM_control_cycle(const int controlcycle)
{
    PM_control_cycle = controlcycle;
}


int Trjplan::getPM_control_cycle(void)
{
    return PM_control_cycle;
}

int Trjplan::getMaxDataNum()
{
    return MaxDataNum;
}

int Trjplan::getDataNum_i()
{
    return DataNum_i;
}

void Trjplan::setMaxDataNum(int max_data_num)
{
    MaxDataNum = max_data_num;
}

void Trjplan::setDataNum_i(int data_num_i)
{
    DataNum_i = data_num_i;

}

void Trjplan::setM_Matrix(const MatrixXd M)
{
    M_Matrix.resize(8,M.cols());
    M_Matrix = M;
}


MatrixXd Trjplan::getM_Matrix(void)
{
    return M_Matrix;
}


MatrixXf Trjplan::getPoseNode(void)
{
    return PoseNode;
}


VectorXf Trjplan::getTplan(void)
{
    return Tplan;

}

void Trjplan::PoseNodeCal(float (&Pose_start)[6], float (&Pose_end)[6], const float v_max, const float a_max)
{
//            cout <<"Pose_end "<<":"
//                <<Pose_end[0]<<"   "
//                <<Pose_end[1]<<"   "
//                <<Pose_end[2]<<"   "
//                <<Pose_end[3]<<"   "
//                <<Pose_end[4]<<"   "
//                <<Pose_end[5]<<endl;

//    RowVectorXi p_start,p_end;
    Map<VectorXf> p_start(Pose_start,6);
    Map<VectorXf> p_end(Pose_end,6);

    Vector3f p_end_temp,p_start_temp;
    for(int i=0;i<3;i++)
    {
        p_end_temp(i) = p_end(i);
        p_start_temp(i) = p_start(i);
    }

    Vector3f length_vector = p_end_temp-p_start_temp;
    Vector3f p_dir=(length_vector)/length_vector.norm();

    float length = length_vector.norm();

    float d_angle[3];
    for(int i=3;i<6;i++)
    {
        d_angle[i-3] = fabs(p_start(i) - p_end(i));
    }
    float angle_length;
    angle_length = d_angle[0]>d_angle[1]?d_angle[0]:d_angle[1];
    angle_length = angle_length>d_angle[2]?angle_length:d_angle[2];

    int N_l = ceil(length/(node_distance/1000))-1;
    int N_a = ceil(angle_length/(5.0/180.0*PI))-1;

//    cout<<"length:"<<length<<endl;
//    cout<<"N_l:"<<N_l<<endl;
//    cout<<"N_a:"<<N_a<<endl;
//    cout<<"angle_length:"<<angle_length<<endl;
    int N;
    bool T_array_cal_type;
    if(N_l>N_a)
    {
        N = N_l;
        T_array_cal_type = 0;
    }
    else
    {
        N = N_a;
        T_array_cal_type = 1;
    }

    if(N<2)
    {N=2;}

//    cout << "N:"<<N<<endl;

    VectorXf t_vec_G(N);
    t_vec_G<<VectorXf::Zero(N);

    VectorXf node_vec(N);
    node_vec<<VectorXf::Zero(N);

    for(int i=0;i<N;i++)
    {
      node_vec[i] = i*length/(N-1);
    }

    MatrixXf pose_ee(6,N);
    pose_ee<<MatrixXf::Zero(6,N);

    if(T_array_cal_type == 0)
    {
        float t_a = v_max/a_max;    //time of acc to v_max;
        float len_a = 0.5*a_max*pow(t_a,2);//length of acc to v_max.
        if(len_a > 0.5*length)
        {
            t_a = pow(length/a_max,0.5);
            for(int i_cal =0;i_cal<N;i_cal++)
            {
                if(node_vec(i_cal) <= length/2)
                {
                    t_vec_G(i_cal) = pow(2*node_vec(i_cal)/a_max,0.5);
                }
                else
                {
                    t_vec_G(i_cal) = 2*t_a-(pow(2*(length-node_vec(i_cal))/a_max,0.5));
                }
            }
//            cout <<"11"<<endl;
        }
        else
        {
            for(int i_cal=0;i_cal<N;i_cal++)
            {
                if(node_vec(i_cal)<=len_a)
                {
                    t_vec_G(i_cal) = pow(2*node_vec(i_cal)/a_max,0.5);
                }

                else if(node_vec(i_cal)>len_a && node_vec(i_cal)<length -len_a)
                {
                    t_vec_G(i_cal) = pow(2*len_a/a_max,0.5)+(node_vec(i_cal)-len_a)/v_max;
                }
                else
                {
                    t_vec_G(i_cal) = 2*pow((2*len_a/a_max),0.5)+(length-2*len_a)/v_max - pow(2*(length - node_vec(i_cal))/a_max,0.5);
                }
            }
            cout <<"22"<<endl;
        }
    }
    else
    {
        float t_all = angle_length/(20.0/180.0*PI);
        for(int i=0;i<N;i++)
        {
          t_vec_G[i] = i*t_all/(N-1);
        }
    }

    for(int i_cal =0;i_cal<N;i_cal++)
    {
        for(int i=0;i<3;i++){
           pose_ee(i,i_cal) = p_start(i)+p_dir[i]*node_vec(i_cal);
        }
    }

    for(int i=0;i<N;i++)
    {
      pose_ee(3,i) = Pose_start[3] + i*(Pose_end[3]-Pose_start[3])/(N-1);
      pose_ee(4,i) = Pose_start[4] + i*(Pose_end[4]-Pose_start[4])/(N-1);
      pose_ee(5,i) = Pose_start[5] + i*(Pose_end[5]-Pose_start[5])/(N-1);
    }

    PoseNode.resize(6,N);
//    for(short i=0;i<N;i++)
//    {
//        cout <<"pose_ee_node "<<i+1<<":"
//            <<pose_ee(0,i)<<"   "
//            <<pose_ee(1,i)<<"   "
//            <<pose_ee(2,i)<<"   "
//            <<pose_ee(3,i)<<"   "
//            <<pose_ee(4,i)<<"   "
//            <<pose_ee(5,i)<<endl;
//    }

    PoseNode = pose_ee;
    Tplan.resize(N);
    Tplan =t_vec_G;
    for(short i=0;i<N;i++)
    {
        cout <<"t_vec_G "<<i+1<<":"
            <<t_vec_G[i]
            <<endl;
    }
}

//void Trjplan::InverseKine(void)
//{


//}


//void Trjplan::MotorAngleCal(const short MotorNum, const int DataNum, const float circle, const float* tplan)
//{


//}

//VectorXf Trjplan::getAngleNodeZero(void)
//{
//    return AngleNodeZero;
//}


//void Trjplan::setAngleNodeZero(const VectorXf angle)
//{
//    AngleNodeZero = angle;
//}

MatrixXd Trjplan::getAngleNode(void)
{
    return AngleNode;
}

void Trjplan::setAngleNode(MatrixXd angle)
{
    AngleNode = angle;
}
