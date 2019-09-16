#ifndef COMPLAINT_MODE_SET_DIALOG_H
#define COMPLAINT_MODE_SET_DIALOG_H

#include <QDialog>

namespace Ui {
class complaint_mode_set_dialog;
}

class complaint_mode_set_dialog : public QDialog
{
    Q_OBJECT

public:
    explicit complaint_mode_set_dialog(QWidget *parent = 0);
    ~complaint_mode_set_dialog();

    void set_complaint_stiffness_from_ini(float *stiffness);
    void set_complaint_damp_from_ini(float *damp);
    void set_complaint_stiffness_from_dialog(void);
    void set_complaint_damp_from_dialog(void);
    void get_complaint_stiffness_from_dialog(float *stiffness);
    void get_complaint_damp_from_dialog(float *damp);

    void set_dialog_state(void);

private slots:
    void on_stiffness_set_pushButton_clicked();

private:
    Ui::complaint_mode_set_dialog *ui;
};

#endif // COMPLAINT_MODE_SET_DIALOG_H
