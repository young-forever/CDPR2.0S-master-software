#include <iostream>
#include <vector>
#include "iMath.h"

using namespace std;

int sign(float x)
{
    if(x < 0) return -1;
    else return 1;
}

void  m_cross3d(double *a, double *b,double *c)
{
    // c = axb;  a,b,c all 3 demension;
    double x1 = a[0];
    double y1 = a[1];
    double z1 = a[2];
    double x2 = b[0];
    double y2 = b[1];
    double z2 = b[2];

    c[0] = y1*z2 - y2*z1;
    c[1] = -(x1*z2 - x2*z1);
    c[2] = x1*y2 - x2*y1;
}

vector<vector<double>> matrix_multiply(vector<vector<double>> arrA, vector<vector<double>> arrB)
{
    //矩阵arrA的行数
    int rowA = arrA.size();
    //矩阵arrA的列数
    int colA = arrA[0].size();
    //矩阵arrB的行数
    int rowB = arrB.size();
    //矩阵arrB的列数
    int colB = arrB[0].size();
    //相乘后的结果矩阵
    vector<vector<double>>  res;
    if (colA != rowB)//如果矩阵arrA的列数不等于矩阵arrB的行数。则返回空
    {
        return res;
    }
    else
    {
        //设置结果矩阵的大小，初始化为为0
        res.resize(rowA);
        for (int i = 0; i < rowA; ++i)
        {
            res[i].resize(colB);
        }

        //矩阵相乘
        for (int i = 0; i < rowA; ++i)
        {
            for (int j = 0; j < colB; ++j)
            {
                for (int k = 0; k < colA; ++k)
                {
                    res[i][j] += arrA[i][k] * arrB[k][j];
                }
            }
        }
    }
    return res;
}
