#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>

//volatile

// 5ms one time.
void *SensorDataGetTask(void *arg) {//线程执行函数，执行10次加法
    while(1){
        usleep(5000);


    }


}


void *ControlTask(void *arg)//线程执行函数，执行10次减法
{
    for(int i=0;i<10;i++)
    {
    usleep(1000);
//    num--;
//        printf("sub-1,result is:%d\n",num);
    }
}

void MyThreadCreate() {
    pthread_t tid1,tid2;
    pthread_create(&tid1,NULL,SensorDataGetTask,NULL);//创建线程
    pthread_create(&tid2,NULL,ControlTask,NULL);

    pthread_exit(&tid1);
    pthread_exit(&tid2);
//    return 0;
}
