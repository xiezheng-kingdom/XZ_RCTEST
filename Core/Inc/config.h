/************************************************************************************************
* 程序版本：V1.0
* 程序日期：2022-7-2
* 程序作者：719飞行器实验室： 张天鹏、刘胜昔						      
************************************************************************************************/
#ifndef   _CONFIG_H_
#define   _CONFIG_H_

double angle_1;
double angle_2;
double angle_3;
double angle_4;
double angle_5;
double angle_6;

double now_angle_1 = 0;
double now_angle_2 = 0;
double now_angle_3 = 0;
double now_angle_4 = 0;
double now_angle_5 = 0;
double now_angle_6 = 90;

double target_angle_1;
double target_angle_2;
double target_angle_3;
double target_angle_4;
double target_angle_5;
double target_angle_6;

double pulse_1;
double pulse_2;
double pulse_3;
double pulse_4;
double pulse_5;
double pulse_6;

double theta_1;
double theta_2;
double theta_3;
double theta_4;
double theta_4;


//机械臂尺寸配置部分
float R = 5;           //底座半径，单位cm
float length_1 = 2;  //底座高度，单位cm
float length_2 = 8;    //机械臂1长度，单位cm
float length_3 = 5;    //机械臂2长度，单位cm
float length_4 = 12;   //机械臂3长度，单位cm


//圆周率常数定义
double pi = 3.141593;


#endif
