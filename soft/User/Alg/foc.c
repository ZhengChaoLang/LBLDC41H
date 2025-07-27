/* ***************************************************
 * 
 * 描述 ： foc算法
 * 
 * 日期:2025-7-27
 * 
 ***************************************************
 */
#include "foc.h"
#include "pid.h"



/*
 * @brief: 克拉克变换
 * @param[in]:
 * @return:
 */
void FOC_Clark(foc_phase_prarm_t* phase_prarm, foc_clark_prarm_t* out_prarm)
{
	out_prarm->alpha = FOC_CLARK_GAIN * (phase_prarm->a - 0.5f*phase_prarm->b - 0.5f*phase_prarm->c);
	out_prarm->beta = FOC_CLARK_GAIN * (0.8660254f*phase_prarm->b - 0.8660254f*phase_prarm->c);
}


/*
 * @brief: 帕克变换
 * @param[in]:
 * @return:
 */
void FOC_Park(foc_clark_prarm_t* clark_prarm, float theta, foc_park_prarm_t* out_prarm)
{
	out_prarm->d =  clark_prarm->alpha * FOC_COS(theta) + clark_prarm->beta * FOC_SIN(theta);
	out_prarm->q = -clark_prarm->alpha * FOC_SIN(theta) + clark_prarm->beta * FOC_COS(theta);
}

/*
 * @brief: 反帕克变换
 * @param[in]:
 * @return:
 */
void FOC_InvPark(foc_park_prarm_t* park_prarm, float theta, foc_clark_prarm_t* out_param)
{
	out_param->alpha = park_prarm->d * FOC_COS(theta) - park_prarm->q * FOC_SIN(theta);
	out_param->beta	 = park_prarm->d * FOC_SIN(theta) + park_prarm->q * FOC_COS(theta);
}


/*
 * @brief: SVPWM模块
 * @param[in]:
 * @return:
 */
void FOC_Svpwm(foc_motor_t* motor, foc_clark_prarm_t * u_two_aixs)
{
	float u1, u2, u3;
	
	uint8_t a=0, b=0, c=0;
	uint8_t sector = 0;
	
	u1 = u_two_aixs->beta;
	u2 = (u_two_aixs->alpha - u_two_aixs->beta)/2.0f;
	u3 = -(1.7320508f*u_two_aixs->alpha + u_two_aixs->beta)/2.0f;
	
	/*扇区判断*/
	if(u1>0)a = 1;
	if(u2>0)b = 1;
	if(u3>0)c = 1;	
	
	switch(a+2*b+4*c){
		case 3:
				sector = 1;
			break;
		case 1:
				sector = 2;
			break;
		case 5:
				sector = 3;
			break;
		case 4:
				sector = 4;
			break;
		case 6:
				sector = 5;
			break;		
		case 2:
				sector = 6;
			break;	
		default:
			return;
		
	}
	
	//矢量作用时间计算
	float x,y,z,k;
	k = FOC_SQRT_3*FOC_PWM_T_COUNT/motor->u_dc;
	x = k*u1;
	y = -k*u3;
	z = -k*u2;
	float t1, t2, t_temp;
	switch(sector){
		case 1:
			t1 = -z;
			t2 = x;
			break;
		case 2:
			t1 = z;
			t2 = y;
			break;
		case 3:
			t1 = x;
			t2 = -y;
			break;
		case 4:
			t1 = -x;
			t2 = z;
			break;
		case 5:
			t1 = -y;
			t2 = -z;
			break;		
		case 6:
			t1 = y;
			t2 = -x;
			break;	
		default:
			return;
			
	}
	//防止过调制
	if(t1 + t2 > FOC_PWM_T_COUNT){
		t_temp = t1;
		t1 = FOC_PWM_T_COUNT*t1/(t1+t2);
		t2 = FOC_PWM_T_COUNT*t1/(t_temp+t2);
	}
	//t0 = (FOC_PWM_T_COUNT-t1-t2)/2;
	float ta, tb, tc;
	ta = (FOC_PWM_T_COUNT - t1 -t2)/4.0f;
	tb = (ta + t1/2.0f);
	tc = tb + t2/2;
	
	float t_cm1,t_cm2 ,t_cm3;
	
		switch(sector){
		case 1:
			t_cm1 = ta;
			t_cm2 = tb;
			t_cm3 = tc;
			break;
		case 2:
			t_cm1 = tb;
			t_cm2 = ta;
			t_cm3 = tc;
			break;
		case 3:
			t_cm1 = tc;
			t_cm2 = ta;
			t_cm3 = tb;
			break;
		case 4:
			t_cm1 = tc;
			t_cm2 = tb;
			t_cm3 = ta;
			break;
		case 5:
			t_cm1 = tb;
			t_cm2 = tc;
			t_cm3 = ta;
			break;		
		case 6:
			t_cm1 = ta;
			t_cm2 = tc;
			t_cm3 = tb;
		default:
			return;		
	}
		
	if(t_cm1 > FOC_PWM_T_COUNT)t_cm1=FOC_PWM_T_COUNT;
	if(t_cm2 > FOC_PWM_T_COUNT)t_cm2=FOC_PWM_T_COUNT;
	if(t_cm3 > FOC_PWM_T_COUNT)t_cm3=FOC_PWM_T_COUNT;
	
	FOC_PWM_SET_VLAUE_U((uint32_t)t_cm1);	FOC_PWM_SET_VLAUE_V((uint32_t)t_cm2);
	FOC_PWM_SET_VLAUE_W((uint32_t)t_cm3);
}

/*
 * @brief:	基本电流环,放中断里
 * @param[in]:
 * @return:
 */
foc_clark_prarm_t* FOC_CurrentLoopCal(foc_motor_t * motor, float ref_iq)
{
	foc_clark_prarm_t i_alph_beta;
	foc_park_prarm_t i_d_q;
	foc_park_prarm_t u_d_q;
	
	//clark
	FOC_Clark(&motor->phase_i, &i_alph_beta);
	//转子位置更新 or 无感观测器	
	
	//park
	FOC_Park(&i_alph_beta,motor->theta_e, &i_d_q);
	//电流环pi	
	motor->ctrl.pid_setRef(motor->ctrl.pid_currentLoop_iq, ref_iq);
	u_d_q.d = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_id, i_d_q.d);
	u_d_q.q = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_iq, i_d_q.q);	
	//inv_park
	FOC_InvPark(&u_d_q, motor->theta_e, &motor->u_alpha_beta);
	//给svpwm
	return &motor->u_alpha_beta;
}

float FOC_SpeedLoopCal(foc_motor_t * motor, float ref_speed)
{
	//速度环PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_speedLoop, OMEGA_TO_RPM(motor->det_theta_m));    
}

float FOC_PositionLoopCal(foc_motor_t * motor, float ref_position_rad)
{
	//位置环PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_positionLoop, OMEGA_TO_RPM(motor->theta_m));    
}
/*
 * @brief: 将角度映射限制 [-pi , pi]
 * @param[in]:
 * @return:
 */
float FOC_MapPi(float rad){
	while(rad>PI){
		rad-=PI;
	}
	while(rad<-PI){
		rad+=PI;
	}
	return rad;
}



