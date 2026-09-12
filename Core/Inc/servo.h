/************************************************************************************************
* ����汾��V1.0
* �������ڣ�2022-7-2
* �������ߣ�719������ʵ���ң� ����������ʤ�������ճ�						      
************************************************************************************************/
#ifndef   _SERVO_H_
#define   _SERVO_H_

#include "tim.h"

void pwm_start(void);
//����PWM���
void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6);
//���Ƕ�ת��Ϊ�����������ı�PWM���ռ�ձ�
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5,  double angle_6);
//�ײ�PWM���
void servo_catch(void);
//��еצ�ս�
void servo_test(void);
//�Ի�еצ���в���
void servo_reset_begin(void);
//����ʼʱ�Ļ�еצ��ʼ��
void servo_reset(void);
//ƽ���ؽ���еצ���Ƕȹ���
void servo_release(void);
//��еצ�ɿ�
void servo_control(double target_angle_1, double target_angle_2, double target_angle_3, double target_angle_4, double target_angle_5,  double target_angle_6);
//���ƻ�е��ƽ���˶�
void servo_angle_calculate(float target_x, float target_y, float target_z);
//�������Ƕ�

void servo_lift(void);
//ץȡ�����������е��̧��
void servo_lift_return(void);
//�ͷ���Ʒ�󷵻ص��м�׶�
void servo_transfer(void);
//机械臂移动到指定固定角度

void servo_transfer_blue(void);
//识别到蓝色物品后执行分拣动作
void servo_transfer_yellow(void);
//识别到黄色物品后执行分拣动作

void blue_task(void);
//识别到蓝色物品后执行分拣动作
void yellow_task(void);
//识别到黄色物品后执行分拣动作
void blue_task(void);
//ʶ����ɫ��Ʒ��ִ�зּ�����
void yellow_task(void);
//ʶ�𵽻�ɫ��Ʒ��ִ�зּ�����
#endif
