/************************************************************************************************
* ����汾��V1.0
* �������ڣ�2022-7-2
* �������ߣ�719������ʵ���ң� ����������ʤ�������ճ�						      
************************************************************************************************/
#include "tim.h"
#include "servo.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

double pulse_1 = 0;
double pulse_2 = 0;
double pulse_3 = 0;
double pulse_4;  // ֻ�� pulse_4 ���� extern����Ϊ���� main.c �ﶨ�����
double pulse_5 = 0;
double pulse_6 = 0;

extern double angle_1;
extern double angle_2;
extern double angle_3;
extern double angle_4;
extern double angle_5;
extern double angle_6;

double now_angle_1 = 0;
double now_angle_2 = -35;  // ��̬-35��
double now_angle_3 = 0;
double now_angle_4 = 0;
double now_angle_5 = 0;
double now_angle_6 = 0;

double target_angle_1 = 0;
double target_angle_2 = 0;
double target_angle_3 = 0;
double target_angle_4 = 0;
double target_angle_5 = 0;
double target_angle_6 = 0;

extern double R;        //�����뾶
extern double length_1; //�����߶�
extern double length_2; //��е��1����
extern double length_3; //��е��2����
extern double length_4; //��е��3����

extern float pi;//Բ���ʳ���

extern TIM_HandleTypeDef htim3;  // ���߱�������htim3�ڱ�ĵط�������

double a1;
double a2;
double a3;
double a4;
double a5;
double a6;
    
double abs_angle_error_1;
double abs_angle_error_2;
double abs_angle_error_3;
double abs_angle_error_4;
double abs_angle_error_5;
double abs_angle_error_6;

/**************************************************************************
�������ܣ�����PWM���
��ڲ�������
����  ֵ����
��ע    ����
**************************************************************************/
void pwm_start(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWM_1
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PWM_2
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//PWM_3
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PWM_4
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//PWM_5
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//PWM_6
}

/**************************************************************************
�������ܣ����Ƕ�ת��Ϊ�����������ı�PWM���ռ�ձ�
��ڲ������Ƕ�
����  ֵ����
��ע    ���Ƕȷ�ΧΪ-90�ȵ�90�ȣ�������Ƕ�Ϊ-90��ʱ������Ϊ0.5ms��������Ƕ�Ϊ0��ʱ������Ϊ1.5ms��������Ƕ�Ϊ90��ʱ������Ϊ2.5ms��
**************************************************************************/

void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6)
{
	//���̣���׼�����������̬���㣬���Ƶװ�Ƕȣ���Χ��-90��90��
	pulse_1 = (((angle_1 + 90) / 90 ) + 0.5)*(20000/20);
	
	//�ؽ�1����׼�����������̬���㣬������б�Ƕȣ���Χ��-90��90��
	pulse_2 = (((angle_2 +90) / 90 ) + 0.5)*(20000/20);
	
	//�ؽ�2�����������������̬���㣬������б�Ƕȣ���Χ��-90��90��
	pulse_3 = (((angle_3 +90) / 90 ) + 0.5)*(20000/20);	
	
	//�ؽ�3����׼�����������̬���㣬������б�Ƕȣ���Χ��-90��90��
	pulse_4 = (((angle_4 +90) / 90 ) + 0.5)*(20000/20);	
	
	//�ؽ�4��ת�����ɶȣ�����
	pulse_5 = ((((angle_5 / 90 * 65) + 90) / 90 ) + 0.5)*(20000/20);	
	
	//צͷ������ĩ�˿�����ץȡ����Χ��-90��90��
	pulse_6 = (((angle_6 ) / 90 ) + 0.5)*(20000/20);				
}


/**************************************************************************
�������ܣ��ײ�PWM���
��ڲ��������Ŀ��Ƕ�
����  ֵ����
��ע    ���������Ƶ��Ϊ50Hz������Ϊ20ms��PSC = 72 - 1��ARR = 200 - 1��f = 72MHz /( PSC + 1 )( ARR +1 )
				ռ�ձ� = pulse / ARR
				���ߵ�ƽʱ��Ϊ0.5msʱ������Ƕ�Ϊ0��
				���ߵ�ƽʱ��Ϊ1.5msʱ������Ƕ�Ϊ90��
				���ߵ�ƽʱ��Ϊ2.5msʱ������Ƕ�Ϊ180��
**************************************************************************/
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5, double angle_6)
{
    double target[6] = {angle_1, angle_2, angle_3, angle_4, angle_5, angle_6};
    double current[6] = {now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6};
    double max_err = 0;
    int i, steps;

    for (i = 0; i < 6; i++) {
        double e = fabs(target[i] - current[i]);
        if (e > max_err) max_err = e;
    }

    if (max_err < 2.0) {
        translate_angle_to_pulse(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse_1);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_2);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse_3);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_4);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pulse_5);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pulse_6);
        now_angle_1 = angle_1; now_angle_2 = angle_2; now_angle_3 = angle_3;
        now_angle_4 = angle_4; now_angle_5 = angle_5; now_angle_6 = angle_6;
        return;
    }

    steps = (int)(max_err);

    for (i = 1; i <= steps; i++) {
        double frac = (double)i / (double)steps;
        double a1 = current[0] + (target[0] - current[0]) * frac;
        double a2 = current[1] + (target[1] - current[1]) * frac;
        double a3 = current[2] + (target[2] - current[2]) * frac;
        double a4 = current[3] + (target[3] - current[3]) * frac;
        double a5 = current[4] + (target[4] - current[4]) * frac;
        double a6 = current[5] + (target[5] - current[5]) * frac;
        translate_angle_to_pulse(a1, a2, a3, a4, a5, a6);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse_1);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_2);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse_3);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_4);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pulse_5);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pulse_6);
        HAL_Delay(20);
    }

    translate_angle_to_pulse(angle_1, angle_2, angle_3, angle_4, angle_5, angle_6);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse_1);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_2);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse_3);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_4);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pulse_5);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pulse_6);

    now_angle_1 = angle_1; now_angle_2 = angle_2; now_angle_3 = angle_3;
    now_angle_4 = angle_4; now_angle_5 = angle_5; now_angle_6 = angle_6;
}void servo_angle_calculate(float target_x, float target_y, float target_z)
{
		//��Ŀ��λ���ں���ץȡ��Χ��
		if(target_y<5 || target_y>15 )
		{
			target_angle_1 = 0;
			target_angle_2 = 0;
			target_angle_3 = 0;
			target_angle_4 = 0;
			printf("\r\n[MSG]OUT_OF_RANGE\r\n");
				return;
		}
        	
		float len_1, len_2, len_3, len_4,bottom_r;   		//���̸߶ȡ���1����2����3����4�����̰뾶 
		float j1,j2,j3,j4 ;   													//�ĸ���̬��
		float L, H;																			//L =	a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); PΪ�ײ�Բ�̰뾶R
		float j_sum;																		//j2,j3,j4֮��
		float len, high;   															//�ܳ���,�ܸ߶�
		float cos_j3, sin_j3; 													//�����洢cos_j3,sin_j3��ֵ
		float cos_j2, sin_j2;														//�����洢cos_j2,sin_j2��ֵ
		float k1, k2;
		
		int i;				//forѭ����������
		int n, m;			//nΪȫ�����н�������mΪȫ�����н�N���м�ֵ
		
		//���н������ʼ��
		n = 0;
		m = 0;

		//��е�۳ߴ��ʼ��
		bottom_r = 10.0;				//�����뾶10.5
		len_1	 = 15.6;		//�����߶�9.5
		len_2 = 10.5;		//��е��1����
		len_3 = 8.8;		//��е��2����
		len_4 = 16.6;		//��е��3����
		
		//����X���������̽Ƕ�j1
		if (target_x == 0)
			j1 = 90;
		else
			j1 = 90 - atan2(target_x, target_y + bottom_r) * (180.0 / 3.1415927);

		//�������п��н⣬����n��¼���н������
		for (i = 0; i <= 180; i ++)
		{
				j_sum = 3.1415927 * i / 180;
				
				len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
				high = target_z;

				L = len - len_4 * sin(j_sum);
				H = high - len_4 * cos(j_sum) - len_1;

				cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
				sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

				j3 = atan2(sin_j3, cos_j3) * (180.0 / 3.1415927);

				k2 = len_3 * sin(j3 / (180.0 / 3.1415927));
				k1 = len_2 + len_3 * cos(j3 / (180.0 / 3.1415927));

				cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
				sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

				j2 = atan2(sin_j2, cos_j2) * (180.0 / 3.1415927);
				j4 = j_sum * (180.0 / 3.1415927) - j2 - j3;
				
				//���ƽ�ķ�Χ�ں�����Χ
				if (j2 > 0 && j3 > 0 && j4 > -90 && j2 <90 && j3 <90 && j4 < 90)
				{
					n++;//���н���������				
				}
		}
		
		
		//mΪȫ�����н�����n���м�ֵ�����¼�����н�ֱ���õ�m��Ӧ�����˶�ѧ��
		for (i = 0; i <= 180; i ++)
		{
				j_sum = 3.1415927 * i / 180;

				len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
				high = target_z;

				L = len - len_4 * sin(j_sum);
				H = high - len_4 * cos(j_sum) - len_1;

				cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
				sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

				j3 = atan2(sin_j3, cos_j3) * (180.0 / 3.1415927);

				k2 = len_3 * sin(j3 / (180.0 / 3.1415927));
				k1 = len_2 + len_3 * cos(j3 / (180.0 / 3.1415927));

				cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
				sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

				j2 = atan2(sin_j2, cos_j2) * (180.0 / 3.1415927);
				j4 = j_sum * (180.0 / 3.1415927) - j2 - j3;


				//���ƽ�ķ�Χ�ں�����Χ
				if (j2 > 0 && j3 > 0 && j4 > -90 && j2 <90 && j3 <90 && j4 < 90)
				{
						m++;//���н���������
						
						//���ؾ�����������������©�����н�
						if ( m == n / 2 || m == (n + 1) / 2) 
						{
								break;
						}			
				}
		}
		
		
		target_angle_1 = j1-90;
		target_angle_2 = j2;
		target_angle_3 = j3;
		target_angle_4 = j4;
		
		printf("\r\n[EXP]BOTTOM:%.2f,JOINT1:%.2f,JOINT2:%.2f,JOINT3:%.2f\r\n",j1,j2,j3,j4);
		printf("\r\n[MSG]RESULT_NUM:%d,ACTUAL_RESULT:%d\r\n",n,m);
		

}




/**************************************************************************
�������ܣ����ƻ�еצץȡĿ��
��ڲ�������
����  ֵ����
��ע    ����
**************************************************************************/

void servo_control(double temp_target_angle_1, double temp_target_angle_2, double temp_target_angle_3,
                   double temp_target_angle_4, double temp_target_angle_5, double temp_target_angle_6)
{
    /* ƽ�������������� pwm_out �У��˴������ò����ȶ���ʱ */
    pwm_out(temp_target_angle_1, temp_target_angle_2, temp_target_angle_3,
            temp_target_angle_4, temp_target_angle_5, temp_target_angle_6);
    HAL_Delay(500);
}void servo_reset_begin(void)
{
    /* ƽ����ʼ����ʹ�� servo_control ȷ�����ؽ�ƽ�ȵ�λ */
    servo_control(0, 0, 0, 70, 0, 20);  // Ĭ����̬
}void servo_catch(void)
{
	servo_control(target_angle_1,target_angle_2,target_angle_3,target_angle_4,0,70);
}

void servo_lift(void)
{
    servo_control(target_angle_1,15,15,35,0,70);
    HAL_Delay(500);
}


void servo_transfer_blue(void)
{
		//ת������ɫ���������ķ�λ
    servo_control(-60,15,15,35,0,70);
    HAL_Delay(500);
		//����ɫ������������
		servo_control(-60,30,40,40,0,70);
		HAL_Delay(500);
		//�ͷ���ɫ���
		servo_control(-60,30,40,40,0,20);
		HAL_Delay(500);
		//�ظ�����ʼλ��
		servo_control(0.1,0.1,0.1,0.1,0,20);
		HAL_Delay(100);
		servo_control(0,0,0,0,0,20);
		HAL_Delay(500);
}

void servo_transfer_yellow(void)
{
		//ת������ɫ���������ķ�λ
    servo_control(-90,15,15,35,0,70);
    HAL_Delay(500);
		//����ɫ������������
		servo_control(-90,30,40,40,0,70);
		HAL_Delay(500);
		//�ͷŻ�ɫ���
		servo_control(-90,30,40,40,0,20);
		HAL_Delay(500);
		//�ظ�����ʼλ��
		servo_control(0.1,0.1,0.1,0.1,0,20);
		HAL_Delay(100);
		servo_control(0,0,0,0,0,20);
		HAL_Delay(500);
}




