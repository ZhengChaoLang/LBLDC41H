/* ***************************************************
 * 
 * ÃèÊö £º focËã·¨
 * 
 * ÈÕÆÚ:2025-7-27
 * 
 ***************************************************
 */
#include "foc.h"
#include "pid.h"



/*
 * @brief: å…‹æ‹‰å…‹å˜æ¢
 * @param[in]:
 * @return:
 */
void FOC_Clark(foc_phase_prarm_t* phase_prarm, foc_clark_prarm_t* out_prarm)
{
	out_prarm->alpha = FOC_CLARK_GAIN * (phase_prarm->a - 0.5f*phase_prarm->b - 0.5f*phase_prarm->c);
	out_prarm->beta = FOC_CLARK_GAIN * (0.8660254f*phase_prarm->b - 0.8660254f*phase_prarm->c);
}


/*
 * @brief: å¸•å…‹å˜æ¢
 * @param[in]:
 * @return:
 */
void FOC_Park(foc_clark_prarm_t* clark_prarm, float theta, foc_park_prarm_t* out_prarm)
{
	out_prarm->d =  clark_prarm->alpha * FOC_COS(theta) + clark_prarm->beta * FOC_SIN(theta);
	out_prarm->q = -clark_prarm->alpha * FOC_SIN(theta) + clark_prarm->beta * FOC_COS(theta);
}

/*
 * @brief: å¸•å…‹é€†å˜æ¢
 * @param[in]:
 * @return:
 */
void FOC_InvPark(foc_park_prarm_t* park_prarm, float theta, foc_clark_prarm_t* out_param)
{
	out_param->alpha = park_prarm->d * FOC_COS(theta) - park_prarm->q * FOC_SIN(theta);
	out_param->beta	 = park_prarm->d * FOC_SIN(theta) + park_prarm->q * FOC_COS(theta);
}


/*
 * @brief: SVPWM
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
	
	/*ÉÈÇøÅÐ¶Ï*/
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
	
	//Ê¸Á¿×÷ÓÃÊ±¼ä¼ÆËã
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
	//·ÀÖ¹¹ýµ÷ÖÆ
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
 * @brief:	FOCç”µæµçŽ¯è®¡ç®—
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
	//×ª×ÓÎ»ÖÃ¸üÐÂ or ÎÞ¸Ð¹Û²âÆ÷	
	
	//park
	FOC_Park(&i_alph_beta,motor->theta_e, &i_d_q);
	//µçÁ÷»·pi	
	motor->ctrl.pid_setRef(motor->ctrl.pid_currentLoop_iq, ref_iq);
	u_d_q.d = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_id, i_d_q.d);
	u_d_q.q = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_iq, i_d_q.q);	
	//inv_park
	FOC_InvPark(&u_d_q, motor->theta_e, &motor->u_alpha_beta);
	//¸øsvpwm
	return &motor->u_alpha_beta;
}

//é€Ÿåº¦çŽ¯è®¡ç®—
float FOC_SpeedLoopCal(foc_motor_t * motor, float ref_speed)
{
	//ËÙ¶È»·PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_speedLoop, OMEGA_TO_RPM(motor->det_theta_m));    
}

//ä½ç½®çŽ¯è®¡ç®—
float FOC_PositionLoopCal(foc_motor_t * motor, float ref_position_rad)
{
	//Î»ÖÃ»·PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_positionLoop, OMEGA_TO_RPM(motor->theta_m));    
}
/*
 * @brief: ½«½Ç¶ÈÓ³ÉäÏÞÖÆ [-pi , pi]
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



