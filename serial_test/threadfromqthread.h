#ifndef THREADFROMQTHREAD_H
#define THREADFROMQTHREAD_H
#include <QThread>
#include <QMutex>
#include<Eigen/Dense>

class ThreadFromQThread : public QThread
{
    Q_OBJECT

public slots:
    void stopImmediately();


private:
    QMutex m_lock;
    bool m_isCanRun;

public:
   ThreadFromQThread(QObject* par);
   ~ThreadFromQThread();

   void run();

};



#endif // THREADFROMQTHREAD_H
