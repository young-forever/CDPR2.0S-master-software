#include "Math_tool.h"
#include "Math.h"
#include "Inver_Mat.h"
#include "main.h"

  
/*******************************************
--------------------参数配置-----------------
*******************************************/

u16 Max_Data_num=0;
int Data_num[Motor_num_all]={0};
u8 Data_Row;
u8 Num_trj=0;					//规划轨迹数量
fp32 Motor_Vel=0.0,Motor_Angle=0.0;
fp32 Expect_Angle[8]={0.0};
fp32 Expect_Vel[8]={0.0};
fp32 Initial_Force=30, Control_Force=9.8;//设置初始化绳长力和运行状态最大控制力；单位：N

                       /////电机号/////1  2  3  4  5  6  7  8  
const short Motor_Dir[Motor_num_all]={1, -1,1, -1,1,-1, 1, -1 };//motor ratate direction;
                  	 /////电机号/////1  2  3  4  5  6  7  8  
const u8 Encoder_ID[Motor_num_all]={ 1, 2, 3, 4, 5, 6, 7, 8};
//const u8 Encoder_ID[Motor_num_all]={ 5, 7, 3, 4, 8, 6, 1, 2};//change 
                     ////电机号/////1  2  3  4  5  6  7  8  
const u8 Driver_ID[Motor_num_all]={ 4, 3, 1, 2, 7, 8, 5, 6};	
//const u8 Driver_ID[Motor_num_all]={ 4, 3, 7, 8, 1, 2, 5, 6};//change	
const	fp32 t_plan[10]={0,2,3.2,4,4.5,5,5.5,6.3,7.5,9.5}; 		//自动返回时间向量
const fp32 N_mA_ratio[8] = {0.0388,0.0388,0.0388,0.0388,0.0388,0.0388,0.0388,0.0388};		//1N对应电机mA值，需要进行标定测量得到；
fp32 T_Array[100]={0};
fp32 M_Matrix[Motor_num_all][100]={0},Cable_Length[Motor_num_all][100];
fp32 Control_Cycle;	//单个电机控制周期间隔
fp32 Inteval_Motor=0.005;	//相邻电机控制周期间隔
fp32 Cable_Length_start[8] ={0};

/********************************************
						定义60个轨迹参数量
********************************************/
u8 step_num01=0,step_num02=0,	step_num03=0,step_num04=0,step_num05=0,step_num06=0,	step_num07=0,step_num08=0,step_num09=0,step_num010=0;
u8 step_num011=0,step_num012=0,	step_num013=0,step_num014=0,step_num015=0,step_num016=0,	step_num017=0,step_num018=0,step_num019=0,step_num020=0;
u8 step_num021=0,step_num022=0,	step_num023=0,step_num024=0,step_num025=0,step_num026=0,	step_num027=0,step_num028=0,step_num029=0,step_num030=0;
u8 step_num031=0,step_num032=0,	step_num033=0,step_num034=0,step_num035=0,step_num036=0,	step_num037=0,step_num038=0,step_num039=0,step_num040=0;
u8 step_num041=0,step_num042=0,	step_num043=0,step_num044=0,step_num045=0,step_num046=0,	step_num047=0,step_num048=0,step_num049=0,step_num050=0;
u8 step_num051=0,step_num052=0,	step_num053=0,step_num054=0,step_num055=0,step_num056=0,	step_num057=0,step_num058=0,step_num059=0,step_num060=0;
//u8 step_num61=0,step_num62=0,	step_num63=0,step_num64=0,step_num65=0,step_num66=0,	step_num67=0,step_num68=0,step_num69=0,step_num70=0;
//u8 step_num71=0,step_num72=0,	step_num73=0,step_num74=0,step_num75=0,step_num76=0,	step_num77=0,step_num78=0,step_num79=0,step_num80=0;
//u8 step_num81=0,step_num82=0,	step_num83=0,step_num84=0,step_num85=0,step_num86=0,	step_num87=0,step_num88=0,step_num89=0,step_num90=0;
//u8 step_num91=0,step_num92=0,	step_num93=0,step_num94=0,step_num95=0,step_num96=0,	step_num97=0,step_num98=0,step_num99=0,step_num100=0;

fp32 CDPM_Motor_Plan01[8][11]={0.0},CDPM_Motor_Plan02[8][11]={0.0},	CDPM_Motor_Plan03[8][11]={0.0},CDPM_Motor_Plan04[8][11]={0.0},CDPM_Motor_Plan05[8][11]={0.0},CDPM_Motor_Plan06[8][11]={0.0},	CDPM_Motor_Plan07[8][11]={0.0},CDPM_Motor_Plan08[8][11]={0.0},CDPM_Motor_Plan09[8][11]={0.0},CDPM_Motor_Plan010[8][11]={0.0};
fp32 CDPM_Motor_Plan011[8][11]={0.0},CDPM_Motor_Plan012[8][11]={0.0},	CDPM_Motor_Plan013[8][11]={0.0},CDPM_Motor_Plan014[8][11]={0.0},CDPM_Motor_Plan015[8][11]={0.0},CDPM_Motor_Plan016[8][11]={0.0},	CDPM_Motor_Plan017[8][11]={0.0},CDPM_Motor_Plan018[8][11]={0.0},CDPM_Motor_Plan019[8][11]={0.0},CDPM_Motor_Plan020[8][11]={0.0};
fp32 CDPM_Motor_Plan021[8][11]={0.0},CDPM_Motor_Plan022[8][11]={0.0},	CDPM_Motor_Plan023[8][11]={0.0},CDPM_Motor_Plan024[8][11]={0.0},CDPM_Motor_Plan025[8][11]={0.0},CDPM_Motor_Plan026[8][11]={0.0},	CDPM_Motor_Plan027[8][11]={0.0},CDPM_Motor_Plan028[8][11]={0.0},CDPM_Motor_Plan029[8][11]={0.0},CDPM_Motor_Plan030[8][11]={0.0};
fp32 CDPM_Motor_Plan031[8][11]={0.0},CDPM_Motor_Plan032[8][11]={0.0},	CDPM_Motor_Plan033[8][11]={0.0},CDPM_Motor_Plan034[8][11]={0.0},CDPM_Motor_Plan035[8][11]={0.0},CDPM_Motor_Plan036[8][11]={0.0},	CDPM_Motor_Plan037[8][11]={0.0},CDPM_Motor_Plan038[8][11]={0.0},CDPM_Motor_Plan039[8][11]={0.0},CDPM_Motor_Plan040[8][11]={0.0};
fp32 CDPM_Motor_Plan041[8][11]={0.0},CDPM_Motor_Plan042[8][11]={0.0},	CDPM_Motor_Plan043[8][11]={0.0},CDPM_Motor_Plan044[8][11]={0.0},CDPM_Motor_Plan045[8][11]={0.0},CDPM_Motor_Plan046[8][11]={0.0},	CDPM_Motor_Plan047[8][11]={0.0},CDPM_Motor_Plan048[8][11]={0.0},CDPM_Motor_Plan049[8][11]={0.0},CDPM_Motor_Plan050[8][11]={0.0};
fp32 CDPM_Motor_Plan051[8][11]={0.0},CDPM_Motor_Plan052[8][11]={0.0},	CDPM_Motor_Plan053[8][11]={0.0},CDPM_Motor_Plan054[8][11]={0.0},CDPM_Motor_Plan055[8][11]={0.0},CDPM_Motor_Plan056[8][11]={0.0},	CDPM_Motor_Plan057[8][11]={0.0},CDPM_Motor_Plan058[8][11]={0.0},CDPM_Motor_Plan059[8][11]={0.0},CDPM_Motor_Plan060[8][11]={0.0};
//fp32 CDPM_Motor_Plan61[8][11]={0.0},CDPM_Motor_Plan62[8][11]={0.0},	CDPM_Motor_Plan63[8][11]={0.0},CDPM_Motor_Plan64[8][11]={0.0},CDPM_Motor_Plan65[8][11]={0.0},CDPM_Motor_Plan66[8][11]={0.0},	CDPM_Motor_Plan67[8][11]={0.0},CDPM_Motor_Plan68[8][11]={0.0},CDPM_Motor_Plan69[8][11]={0.0},CDPM_Motor_Plan70[8][11]={0.0};
//fp32 CDPM_Motor_Plan71[8][11]={0.0},CDPM_Motor_Plan72[8][11]={0.0},	CDPM_Motor_Plan73[8][11]={0.0},CDPM_Motor_Plan74[8][11]={0.0},CDPM_Motor_Plan75[8][11]={0.0},CDPM_Motor_Plan76[8][11]={0.0},	CDPM_Motor_Plan77[8][11]={0.0},CDPM_Motor_Plan78[8][11]={0.0},CDPM_Motor_Plan79[8][11]={0.0},CDPM_Motor_Plan80[8][11]={0.0};
//fp32 CDPM_Motor_Plan81[8][11]={0.0},CDPM_Motor_Plan82[8][11]={0.0},	CDPM_Motor_Plan83[8][11]={0.0},CDPM_Motor_Plan84[8][11]={0.0},CDPM_Motor_Plan85[8][11]={0.0},CDPM_Motor_Plan86[8][11]={0.0},	CDPM_Motor_Plan87[8][11]={0.0},CDPM_Motor_Plan88[8][11]={0.0},CDPM_Motor_Plan89[8][11]={0.0},CDPM_Motor_Plan90[8][11]={0.0};
//fp32 CDPM_Motor_Plan91[8][11]={0.0},CDPM_Motor_Plan92[8][11]={0.0},	CDPM_Motor_Plan93[8][11]={0.0},CDPM_Motor_Plan94[8][11]={0.0},CDPM_Motor_Plan95[8][11]={0.0},CDPM_Motor_Plan96[8][11]={0.0},	CDPM_Motor_Plan97[8][11]={0.0},CDPM_Motor_Plan98[8][11]={0.0},CDPM_Motor_Plan99[8][11]={0.0},CDPM_Motor_Plan100[8][11]={0.0};

u8 t_plan01[11]={0.0},t_plan02[11]={0.0},	t_plan03[11]={0.0},t_plan04[11]={0.0},t_plan05[11]={0.0},t_plan06[11]={0.0},	t_plan07[11]={0.0},t_plan08[11]={0.0},t_plan09[11]={0.0},t_plan010[11]={0.0};
u8 t_plan011[11]={0.0},t_plan012[11]={0.0},	t_plan013[11]={0.0},t_plan014[11]={0.0},t_plan015[11]={0.0},t_plan016[11]={0.0},	t_plan017[11]={0.0},t_plan018[11]={0.0},t_plan019[11]={0.0},t_plan020[11]={0.0};
u8 t_plan021[11]={0.0},t_plan022[11]={0.0},	t_plan023[11]={0.0},t_plan024[11]={0.0},t_plan025[11]={0.0},t_plan026[11]={0.0},	t_plan027[11]={0.0},t_plan028[11]={0.0},t_plan029[11]={0.0},t_plan030[11]={0.0};
u8 t_plan031[11]={0.0},t_plan032[11]={0.0},	t_plan033[11]={0.0},t_plan034[11]={0.0},t_plan035[11]={0.0},t_plan036[11]={0.0},	t_plan037[11]={0.0},t_plan038[11]={0.0},t_plan039[11]={0.0},t_plan040[11]={0.0};
u8 t_plan041[11]={0.0},t_plan042[11]={0.0},	t_plan043[11]={0.0},t_plan044[11]={0.0},t_plan045[11]={0.0},t_plan046[11]={0.0},	t_plan047[11]={0.0},t_plan048[11]={0.0},t_plan049[11]={0.0},t_plan050[11]={0.0};
u8 t_plan051[11]={0.0},t_plan052[11]={0.0},	t_plan053[11]={0.0},t_plan054[11]={0.0},t_plan055[11]={0.0},t_plan056[11]={0.0},	t_plan057[11]={0.0},t_plan058[11]={0.0},t_plan059[11]={0.0},t_plan060[11]={0.0};
//u8 t_plan61[11]={0.0},t_plan62[11]={0.0},	t_plan63[11]={0.0},t_plan64[11]={0.0},t_plan65[11]={0.0},t_plan66[11]={0.0},	t_plan67[11]={0.0},t_plan68[11]={0.0},t_plan69[11]={0.0},t_plan70[11]={0.0};
//u8 t_plan71[11]={0.0},t_plan72[11]={0.0},	t_plan73[11]={0.0},t_plan74[11]={0.0},t_plan75[11]={0.0},t_plan76[11]={0.0},	t_plan77[11]={0.0},t_plan78[11]={0.0},t_plan79[11]={0.0},t_plan80[11]={0.0};
//u8 t_plan81[11]={0.0},t_plan82[11]={0.0},	t_plan83[11]={0.0},t_plan84[11]={0.0},t_plan85[11]={0.0},t_plan86[11]={0.0},	t_plan87[11]={0.0},t_plan88[11]={0.0},t_plan89[11]={0.0},t_plan90[11]={0.0};
//u8 t_plan91[11]={0.0},t_plan92[11]={0.0},	t_plan93[11]={0.0},t_plan94[11]={0.0},t_plan95[11]={0.0},t_plan96[11]={0.0},	t_plan97[11]={0.0},t_plan98[11]={0.0},t_plan99[11]={0.0},t_plan100[11]={0.0};

	/************************			因此轨迹规划数量不能超过50，每段轨迹节点数量为11			*************************/


//掉电后重新启动自动回零位函数
void PowerDownAutoBack(void)
{
    int i,j;		

		for(i=0;i<Motor_num_all;i++)
    {
			Cable_Length[i][9]=-Expect_Angle[i]; 
    }
		
		for(j=0;j<9;j++)
    {
      for(i=0;i<Motor_num_all;i++)
      Cable_Length[i][j]=Cable_Length[i][9]/9.0*j;       
    }
    Data_Row=10;
		for (i=0;i<Data_Row;i++)
		T_Array[i]=t_plan[i];
				
		Max_Data_num=(int)((T_Array[Data_Row-1]-T_Array[0])/Control_Cycle)+1;
    for (i=0;i<Motor_num_all;i++)
    {
//      Data_num[i]=(int)((T_Array[Data_Row-1]-T_Array[0]-Inteval_Motor*(i+1))/Control_Cycle+2)+1;
			spline3ToM( Data_Row,Control_Cycle,Cable_Length[i],T_Array,0,0,M_Matrix[i]);
    }		
		 Motor_Fw_Flag=1;			//掉电自动返回这里必须设为1，自己分析逻辑
		 Moving_Init_Flag=1;
}



    
    //电机自动归零
void Moto_Auto_Moving_back (void)
{
// fp32 Tem_Angle[10];
    int i,j;
  
    for(i=0;i<Motor_num_all;i++)
    {
			Cable_Length[i][9]=Expect_Angle[i];      
    }      
  for(j=0;j<9;j++)
    {
      for(i=0;i<Motor_num_all;i++){
			if(AutobackFlag == 1){
				Cable_Length[i][j]=Cable_Length_start[i]+(Cable_Length[i][9]-Cable_Length_start[i])/9.0*j;
			}       	
			else{
				Cable_Length[i][j]=Cable_Length[i][9]/9.0*j;}       
			}
		}
    Data_Row=10;
		for (i=0;i<Data_Row;i++)
		T_Array[i]=t_plan[i];
				
		Max_Data_num=(int)((T_Array[Data_Row-1]-T_Array[0])/Control_Cycle)+1;
    for (i=0;i<Motor_num_all;i++)
    {
//      Data_num[i]=(int)((T_Array[Data_Row-1]-T_Array[0]-Inteval_Motor*(i+1))/Control_Cycle+2)+1;
			spline3ToM( Data_Row,Control_Cycle,Cable_Length[i],T_Array,0,0,M_Matrix[i]);
    }		
		 Motor_Fw_Flag=-1;
		 Moving_Init_Flag=1;
}
    
    
                      
u8 Moving_Data_Processing()
{
	u8 i,j;	
	#include <CDPR_Motor_Data.h>
  
	for(i=0;i<8;i++){
	Cable_Length_start[i] = CDPM_Motor_Plan02[i][0];}   
		
        switch(Trj_num)
        {
      case 1://第1个轨迹
				
			          Data_Row=step_num01; //数据规划个数
			          
			          for(i=0;i<Data_Row;i++)
			          {
									 for (j=0;j<Motor_num_all;j++)
								    Cable_Length[j][i]=CDPM_Motor_Plan01[j][i];
								}
								
			         for (i=0;i<Data_Row;i++)
								T_Array[i]=t_plan01[i];
              
								break;    
            case 2://第2个轨迹
        
									 Data_Row=step_num02; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan02[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan02[i];
						
                break;  
                    
            case 3://第3个轨迹
                   Data_Row=step_num03; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan03[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan03[i];
            break;  
            case 4://第4个轨迹
                  Data_Row=step_num04; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan04[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan04[i];
            break;
						case 5://第5个轨迹
                 Data_Row=step_num05; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan05[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan05[i];
										
            break;
				case 6://第6个轨迹
              Data_Row=step_num06; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan06[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan06[i]; 
               break;     
            case 7://第7个轨迹
               Data_Row=step_num07; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan07[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan07[i]; 
										
                break;  
                    
            case 8://第8个轨迹
                   Data_Row=step_num08; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan08[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan08[i]; 
										
            break;  
            case 9://第9个轨迹
                   Data_Row=step_num09; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan09[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan09[i]; 
            break;
						case 10://第10个轨迹
                  Data_Row=step_num010; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan010[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan010[i]; 
            break;
				case 11://第11个轨迹
                
				          Data_Row=step_num011; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan011[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan011[i]; 
                break;  
                    
            case 12://第12个轨迹
                   Data_Row=step_num012; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan012[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan012[i]; 
                break;  
                    
            case 13://第13个轨迹
                  Data_Row=step_num013; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan013[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan013[i]; 
            break;  
            case 14://第14个轨迹
                   Data_Row=step_num014; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan014[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan014[i]; 
            break;
						case 15://第15个轨迹
                   Data_Row=step_num015; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan015[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan015[i]; 
            break;
				case 16://第16个轨迹
                     Data_Row=step_num016; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan016[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan016[i]; 
                break;  
                    
            case 17://第17个轨迹
                    Data_Row=step_num017; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan017[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan017[i]; 
                break;  
                    
            case 18://第18个轨迹
                  Data_Row=step_num018; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan018[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan018[i]; 
            break;  
            case 19://第19个轨迹
                   Data_Row=step_num019; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan019[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan019[i]; 
            break;
						case 20://第20个轨迹
                   Data_Row=step_num020; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan020[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan020[i]; 
            break;
						case 21://第21个轨迹
                  Data_Row=step_num021; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan021[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan021[i]; 
            break;
						case 22://第22个轨迹
                  Data_Row=step_num022; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan022[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan022[i]; 
            break;
						case 23://第23个轨迹
                   Data_Row=step_num023; //数据规划个数
										
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan023[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan023[i]; 
            break;
						case 24://第24个轨迹
                   Data_Row=step_num024; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan024[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan024[i]; 
            break;
						case 25://第25个轨迹
                    Data_Row=step_num025; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan025[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan025[i]; 
            break;
					case 26://第26个轨迹
                   Data_Row=step_num026; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan026[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan026[i]; 
            break;
										
						case 27://第27个轨迹
							
                   Data_Row=step_num027; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan027[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan027[i]; 
            break;
						
						case 28://第28个轨迹
							
                   Data_Row=step_num028; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan028[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan028[i]; 
            break;		
										
						case 29://第29个轨迹
							
                   Data_Row=step_num029; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan029[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan029[i]; 
            break;	

						
						case 30://第30个轨迹
							
                   Data_Row=step_num030; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan030[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan030[i]; 
            break;		
						
						case 31://第31个轨迹
							
                   Data_Row=step_num031; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan031[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan031[i]; 
            break;						
						
						case 32://第32个轨迹
							
                   Data_Row=step_num032; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan032[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan032[i]; 
            break;			

						
						case 33://第33个轨迹
							
                   Data_Row=step_num033; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan033[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan033[i]; 
            break;				

						
						case 34://第34个轨迹
							
                   Data_Row=step_num034; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan034[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan034[i]; 
            break;					
						
						case 35://第35个轨迹
							
                   Data_Row=step_num035; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan035[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan035[i]; 
            break;				
						
						case 36://第36个轨迹
							
                   Data_Row=step_num036; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan036[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan036[i]; 
            break;		
																
						case 37://第37个轨迹
							
                   Data_Row=step_num037; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan037[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan037[i]; 
            break;		
																
						case 38://第38个轨迹
							
                   Data_Row=step_num038; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan038[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan038[i]; 
            break;		
																
						case 39://第39个轨迹
							
                   Data_Row=step_num039; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan039[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan039[i]; 
            break;		
																
						case 40://第40个轨迹
							
                   Data_Row=step_num040; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan040[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan040[i]; 
            break;		
						
						case 41://第41个轨迹
							
                   Data_Row=step_num041; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan041[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan041[i]; 
            break;		
																
						case 42://第42个轨迹
							
                   Data_Row=step_num042; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan042[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan042[i]; 
            break;		
						
						case 43://第43个轨迹
							
                   Data_Row=step_num043; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan043[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan043[i]; 
            break;		
																
						case 44://第44个轨迹
							
                   Data_Row=step_num044; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan044[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan044[i]; 
            break;		
																
						case 45://第45个轨迹
							
                   Data_Row=step_num045; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan045[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan045[i]; 
            break;		
																
						case 46://第46个轨迹
							
                   Data_Row=step_num046; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan046[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan046[i]; 
            break;		
																
						case 47://第47个轨迹
							
                   Data_Row=step_num047; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan047[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan047[i]; 
            break;		
																
						case 48://第48个轨迹
							
                   Data_Row=step_num048; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan048[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan048[i]; 
            break;		
																
						case 49://第49个轨迹
							
                   Data_Row=step_num049; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan049[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan049[i]; 
            break;		
																
						case 50://第50个轨迹
							
                   Data_Row=step_num050; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan050[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan050[i]; 
            break;		
										
						case 51://第51个轨迹
							
                   Data_Row=step_num051; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan051[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan051[i]; 
            break;		
																
						case 52://第52个轨迹
							
                   Data_Row=step_num052; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan052[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan052[i]; 
            break;		
						
						case 53://第53个轨迹
							
                   Data_Row=step_num053; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan053[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan053[i]; 
            break;		
																
						case 54://第54个轨迹
							
                   Data_Row=step_num054; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan054[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan054[i]; 
            break;		
																
						case 55://第55个轨迹
							
                   Data_Row=step_num055; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan055[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan055[i]; 
            break;		
																
						case 56://第56个轨迹
							
                   Data_Row=step_num056; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan056[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan056[i]; 
            break;		
																
						case 57://第57个轨迹
							
                   Data_Row=step_num057; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan057[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan057[i]; 
            break;		
																
						case 58://第58个轨迹
							
                   Data_Row=step_num058; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan058[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan058[i]; 
            break;		
																
						case 59://第59个轨迹
							
                   Data_Row=step_num059; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan059[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan059[i]; 
            break;		
																
						case 60://第60个轨迹
							
                   Data_Row=step_num060; //数据规划个数
										for(i=0;i<Data_Row;i++)
										{
											 for (j=0;j<Motor_num_all;j++)
												Cable_Length[j][i]=CDPM_Motor_Plan060[j][i];
										}
										
									 for (i=0;i<Data_Row;i++)
										T_Array[i]=t_plan060[i]; 
            break;		
										
										

										
        }
         
								Max_Data_num=(int)((T_Array[Data_Row-1]-T_Array[0])/Control_Cycle)+1;
								for (i=0;i<Motor_num_all;i++)
									{
										spline3ToM( Data_Row,Control_Cycle,Cable_Length[i],T_Array,0,0,M_Matrix[i]);
									}
									

    return 1;
}


/////////////给电机号查驱动器号

u8  CDPM_Driver_ID_num(u8 Motor_Num)
{
    	u8 ID; 
		  ID=Driver_ID[Motor_Num-1];
			return ID;
}

/////////////给驱动器号查电机号
u8  CDPM_Mot_num(u8 Dri_ID)
{
  u8 Mot_num=0; 
	u8 i;
	
	for (i=0;i<Motor_num_all;i++)
	{
	  if(Dri_ID == Driver_ID[i])
	  Mot_num = i+1;
	}
	return Mot_num;
}

/***************************
Description: 给定电机号，查出编码器号
*****************************/

u8  CDPM_Encoder_Real_num(u8 Motor_Num)//给定电机号，查出编码器号
{
	
	  return Encoder_ID[Motor_Num-1];

}


/***************************
Description: 给定编码器号，查出电机号

*****************************/

u8  CDPM_Motor_num_Back(u8 Ec_Num)//给定编码器号，查出电机号
{
	
	u8 i;
	
	for (i=0;i<Motor_num_all;i++)
	{
	  if(Ec_Num==Encoder_ID[i])
	  return (i+1);
	}
	return 0;

}


int Motor_Real_Direction(u8 Motor_num)//电机实际运行的方向
{
	
	 return (int)(Motor_Dir[Motor_num-1]);
      
  }
    
  
/******
三次样条插值
即三弯矩插值
修改后是根据输入线长转化出M值数表
******/
 void spline3ToM( u8 num,fp32 interval,fp32*angle,fp32*t,fp32 v0,fp32 vt,fp32 *Mat_M)//n组角度数据，interval相当于dangle M矩阵是行
 {
	 u16 i,j,n;
//	 fp32 *h,*f,*Mat_M,*Mat_P,*lamda,*u,*K,*m,*inver_K,time; // h[i]为时间区间，f[i]代表区间内的平均斜率，Mat_P为矩阵P
//   fp32 time;
	 fp32 *h=malloc(sizeof(fp32)*num);
	 fp32 *f=malloc(sizeof(fp32)*num);
	 fp32 *Mat_P=malloc(sizeof(fp32)*num);
	 fp32 *lamda=malloc(sizeof(fp32)*num);
	 fp32 *u=malloc(sizeof(fp32)*num);
	 fp32 *K=malloc(sizeof(fp32)*num*num);
	 fp32 *inver_K=malloc(sizeof(fp32)*num*num);

//	 
	 n=num;//数据个数

   for(i=0;i<(n-1);i++)
	 {
	   *(h+i)=t[i+1]-t[i];//计算各个区间长度
     *(f+i)=(*(angle+i+1)-*(angle+i))/(*(h+i));//计算各个区间内的平均斜率，f[tj,tj+1]
	 }
	 
	
	 
	 
   *(Mat_P+0)=6.0/(*(h+0))*(*(f+0)-v0); //方程组右边矩阵P
	
	 for(i=0;i<(n-2);i++)//左边方程内参数
	 {
    *(lamda+i)=*(h+i)/(*(h+i+1)+*(h+i));//方程组左边的几个参数的值lamda
    *(u+i)=1-*(lamda+i);//方程组左边矩阵[K]的几个参数的值u
    *(Mat_P+i+1)=6*(*(f+i+1)-*(f+i))/(*(h+i+1)+*(h+i));//方程组右边矩阵[Mat_P]
	 }
	 *(Mat_P+n-1)=6*(vt-*(f+n-2))/(*(h+n-2));

		free(f);
	 
	 for(i=0;i<n;i++)
		  for(j=0;j<n;j++) 
		     *(K+i*n+j)=0;//矩阵 [K]全部赋值0
		
		
	  *(K+0)=2;  *(K+1)=1;//第一行赋值

    for(i=1;i<n;i++)//参数矩阵赋值，之后求逆，就可以求出未知参数
		{
			*(K+i*n+i-1)=*(lamda+i-1);
			*(K+i*n+i)=2;
			if(i<(n-1))
				*(K+i*n+i+1)=*(u+i-1);
			
    }
	  *(K+(n-1)*n+n-2)=1;
		
		free(lamda);
		
	  Inv_Mtrx(K, n , inver_K);	
		
    free(K);
    free(u);
	
    T1T2(n,n,1,inver_K,Mat_P,Mat_M);
    free(Mat_P);
	  free(inver_K);
    free(h); 
		
 }
//////////////////////////////////////////////

void Time_Array(int num,int step,int seq[])//根据数据组数和步长定义时间数组
{
	int i;
	for(i=0;i<num;i++)
	{
		seq[i]=i*step;
	}
}

//计算当前电机号号位置点序号对应的电机转角和速度
void Data_V_S_Compute(u8 Motor_num,int Data_Num_i)
{

			if(Data_Num_i==(Max_Data_num-1))
			 {				
	 
					if( Data_num[Motor_num-1]<Max_Data_num)
					 Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Max_Data_num-2,1);	
					else
					 Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Max_Data_num-1,0);
			 }
			else
			{
				Calculate_MotVand_Angle(Motor_num,Data_Row,Inteval_Motor*Motor_num ,Data_Num_i,0);				 
			}
}



//计算样条曲线的规划速度和位置
//fp32 S5;
void Calculate_MotVand_Angle(u8 Mot_Num,u8 n,fp32 Init_interval ,short Data_i,u8 Flag)
{
	fp32 time,V1,V2,V3,V4,S1,S2,S3,S4,step;//
	u16 j=0;
		
	if(Data_i>0)
		time=Init_interval+Control_Cycle*(Data_i-1);	//当前时间
	else
		time=0;	
		
		if(time> T_Array[n-1])
		{
			time=T_Array[n-1];
			j=n-2;
		}
		else
		 for(j=0;j<(n-1);j++)
		 {
			 if((time>= T_Array[j]) && (time< T_Array[j+1]))
			break;
			 
			 if(j==(n-2))
			 {
				if(time==T_Array[j+1])
					 break;
			 }
			}

			//速度
			step= T_Array[j+1]- T_Array[j];
			
			V1=-M_Matrix[Mot_Num-1][j]*(pow( T_Array[j+1]-time,2))/(2.0*step);		
			V2=M_Matrix[Mot_Num-1][j+1]*(pow(time- T_Array[j],2))/(2.0*step);		
			V3=(Cable_Length[Mot_Num-1][j+1]-Cable_Length[Mot_Num-1][j])/step;
			V4=-(M_Matrix[Mot_Num-1][j+1]-M_Matrix[Mot_Num-1][j])*step/6.0;		
			Motor_Vel=V1+V2+V3+V4;//电机速度
			//位移			
			S1=M_Matrix[Mot_Num-1][j]*(pow( T_Array[j+1]-time,3))/(6.0*step);
			S2=M_Matrix[Mot_Num-1][j+1]*(pow(time- T_Array[j],3))/(6.0*step);
			S3=(Cable_Length[Mot_Num-1][j]-M_Matrix[Mot_Num-1][j]*(pow(step,2))/6.0)*( T_Array[j+1]-time)/step;
			S4=(Cable_Length[Mot_Num-1][j+1]-M_Matrix[Mot_Num-1][j+1]*(pow(step,2))/6.0)*(time- T_Array[j])/step;

		 Motor_Angle=S1+S2+S3+S4;

}



