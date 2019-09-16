#ifndef TRJ_PLAN_H
#define TRJ_PLAN_H
#include<Eigen/Dense>
using namespace Eigen;


#define PositionMode    0x0a
#define ForceMode       0x0b
#define HybridMode      0x0c

class Trjplan
{
//    bool runningflag;

//    int Max_num;
    int trj_Num;
    double P_zero[6];            //store zero point of cdpr.unit:m.
    double P_start[6];
    float P_start_zoffset;
    float P0[6];
    float Pt[6];
    float node_distance;        //mm
    short ContrlMode;           //define control mode of trajectory plan.
    int PM_control_cycle;       //position mode control mode;
    int MaxDataNum;
    int DataNum_i;

    MatrixXd M_Matrix;          //matrix of interpolation.
    MatrixXf PoseNode;          //interpolation nodes.
    VectorXf Tplan;              //
//    VectorXf AngleNodeZero;
    MatrixXd AngleNode;         //


public:
    Trjplan();
    ~Trjplan();

   //flag:
//    void setrunningfalg(const bool flag);
//    bool getruningflag();

    int  get_trj_num();
    void set_trj_num(const int trj_num);
    void setP_zero(const double (&p_zero)[6]);
    void getP_zero(double (&p_zero)[6]);
    void setP_start(const double (&pose)[6]);
    void getP_start(double (&pose)[6]);

    void setP_start_zoffset(const float &offset);
    float getP_start_zoffset(void);

    void setP0(const float (&pose)[6]);
    void getP0(float (&pose)[6]);

    void setPt(const float (&pose)[6]);
    void getPt(float (&pose)[6]);
    void setControlMode(const short mode);
    void setPM_control_cycle(const int controlcycle);
    int getPM_control_cycle(void);

    int getMaxDataNum();
    int getDataNum_i();
    void setMaxDataNum(int max_data_num);
    void setDataNum_i(int data_num_i);

    void setM_Matrix(const MatrixXd M);
    MatrixXd getM_Matrix(void);
    MatrixXf getPoseNode(void);
    VectorXf getTplan(void);
    void PoseNodeCal(float (&Pose_start)[6], float (&Pose_end)[6], const float v_max, const float a_max); //
//    void InverseKine(void);
//    void MotorAngleCal(const short MotorNum, const int DataNum, const float circle, const float* tplan);

//    VectorXf getAngleNodeZero(void);
//    void setAngleNodeZero(const VectorXf angle);

    MatrixXd getAngleNode(void);
    void setAngleNode(MatrixXd angle);

};

#endif // TRJ_PLAN_H
