#ifndef REMOTE_DIALOG_H
#define REMOTE_DIALOG_H

#include <QDialog>

namespace Ui {
class Remote_Dialog;
}

class Remote_Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Remote_Dialog(QWidget *parent = 0);
    ~Remote_Dialog();

    bool X_UP_flag;
    bool X_DOWN_flag;
    bool Y_UP_flag;
    bool Y_DOWN_flag;
    bool Z_UP_flag;
    bool Z_DOWN_flag;

    float remote_step;//unit: mm

    void RemoteTrickInit();

private slots:

    void on_remote_step_Slider_valueChanged(int value);

    void on_Y_UP_pushButton_released();

    void on_Y_UP_pushButton_pressed();

    void on_Y_DOWN_pushButton_pressed();

    void on_Y_DOWN_pushButton_released();

    void on_X_DOWN_pushButton_pressed();

    void on_X_DOWN_pushButton_released();

    void on_X_UP_pushButton_pressed();

    void on_X_UP_pushButton_released();

    void on_Z_UP_pushButton_pressed();

    void on_Z_UP_pushButton_released();

    void on_Z_DOWN_pushButton_pressed();

    void on_Z_DOWN_pushButton_released();

private:
    Ui::Remote_Dialog *ui;
};

#endif // REMOTE_DIALOG_H
