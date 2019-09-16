#ifndef POSITION_MODE_SET_DIALOG_H
#define POSITION_MODE_SET_DIALOG_H

#include <QDialog>

namespace Ui {
class position_mode_set_dialog;
}

class position_mode_set_dialog : public QDialog
{
    Q_OBJECT
    bool Joint_mode_choose[8];
    float constant_force;
    float Line_MaxV;            //m/s
    float Line_MaxA;            //m/s2

public:
    explicit position_mode_set_dialog(QWidget *parent = 0);
    ~position_mode_set_dialog();

    void set_joint_mode_from_ini(bool* joint_mode);
    void set_joint_mode_from_dialog(void);
    void get_joint_mode(bool *joint_mode);

    void set_constant_fore_from_dialog(void);
    void set_constant_fore_from_ini(float force);
    float get_constant_force(void);
    void set_dialog_state(void);

    void setLine_MaxV(const float v_max);
    float getLine_MaxV(void);
    void setLine_MaxA(const float a_max);
    float getLine_MaxA(void);

private slots:
    void on_position_mode_apply_pushButton_clicked();

private:
    Ui::position_mode_set_dialog *ui;
};

#endif // POSITION_MODE_SET_DIALOG_H
