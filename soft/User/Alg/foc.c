/* ***************************************************
 * 
 * ���� �� foc�㷨
 * 
 * ����:2025-7-27
 * 
 ***************************************************
 */
#include "foc.h"
#include "pid.h"
#include "rtthread.h"

#define FOC_STRCMP(str1,str2)  rt_strcmp(str1, str2)

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


/**
 * @brief 帕克变换（Park变换）
 *        将α-β静止坐标系下的分量转换到d-q旋转坐标系
 * @param[in] clark_prarm 输入α-β坐标系下的分量（如电流）
 * @param[in] theta 旋转坐标系与静止坐标系的夹角（电角度）
 * @param[out] out_prarm 输出d-q坐标系下的分量
 */
void FOC_Park(foc_clark_prarm_t* clark_prarm, float theta, foc_park_prarm_t* out_prarm)
{
    // d轴分量计算：α分量乘cosθ加上β分量乘sinθ
    out_prarm->d =  clark_prarm->alpha * FOC_COS(theta) + clark_prarm->beta * FOC_SIN(theta);
    // q轴分量计算：-α分量乘sinθ加上β分量乘cosθ
    out_prarm->q = -clark_prarm->alpha * FOC_SIN(theta) + clark_prarm->beta * FOC_COS(theta);
}

/*
 * @brief: 帕克逆变换
 * @param[in]:
 * @return:
 */
void FOC_InvPark(foc_park_prarm_t* park_prarm, float theta, foc_clark_prarm_t* out_param)
{
	out_param->alpha = park_prarm->d * FOC_COS(theta) - park_prarm->q * FOC_SIN(theta);
	out_param->beta	 = park_prarm->d * FOC_SIN(theta) + park_prarm->q * FOC_COS(theta);
}

uint32_t aa,bb,cc;
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
	u2 = (1.7320508f*u_two_aixs->alpha - u_two_aixs->beta)/2.0f;
	u3 = -(1.7320508f*u_two_aixs->alpha + u_two_aixs->beta)/2.0f;
	
	/*�����ж�*/
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
            break;               
		
	}
	
	//ʸ������ʱ�����
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
			break;
			
	}
	//��ֹ������
	if(t1 + t2 > FOC_PWM_T_COUNT){
		t_temp = t1;
		t1 = FOC_PWM_T_COUNT*t1/(t1+t2);
		t2 = FOC_PWM_T_COUNT*t2/(t_temp+t2);
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
            break;
		default:
			break;		
	}
		
	if(t_cm1 > FOC_PWM_T_COUNT)t_cm1=FOC_PWM_T_COUNT;
	if(t_cm2 > FOC_PWM_T_COUNT)t_cm2=FOC_PWM_T_COUNT;
	if(t_cm3 > FOC_PWM_T_COUNT)t_cm3=FOC_PWM_T_COUNT;
	aa = t_cm1;
    bb = t_cm2;
    cc = t_cm3;
	FOC_PWM_SET_VLAUE_U((uint32_t)t_cm1);	
    FOC_PWM_SET_VLAUE_V((uint32_t)t_cm2);
	FOC_PWM_SET_VLAUE_W((uint32_t)t_cm3);
}

#include "filter.h"
ema_filter_t filter_iq ={0.5f,};
ema_filter_t filter_id ={0.5f,};
/**
 * @brief FOC控制中的电流环计算函数
 * @param motor 电机控制结构体指针，包含电机状态、控制参数等信息
 * @param ref_iq q轴电流参考值（目标电流）
 * @return 指向alpha-beta坐标系下电压指令的指针
 */
foc_clark_prarm_t* FOC_CurrentLoopCal(foc_motor_t * motor, float ref_iq)
{
    foc_clark_prarm_t i_alph_beta;  // 存储Clark变换后的alpha-beta坐标系电流
    foc_park_prarm_t i_d_q;        // 存储park变换后的d-q坐标系电流
    foc_park_prarm_t u_d_q;        // 存储d-q坐标系下的电压指令

    // Clark变换：将三相电流(i_a, i_b, i_c)转换为两相静止坐标系(alpha, beta)电流
    // 消除了三相电流中的零序分量，简化计算
    FOC_Clark(&motor->phase_i, &i_alph_beta);

    // Park变换：将alpha-beta坐标系电流转换为同步旋转坐标系(d-q)电流
    // 使交流量转换为直流量，便于使用PI控制器进行调节
    // motor->theta_e为电角度，用于坐标变换的角度参考
    FOC_Park(&i_alph_beta, motor->theta_e, &i_d_q);
    i_d_q.q= Filter_LPS(&filter_iq, i_d_q.q);
    i_d_q.d = Filter_LPS(&filter_id, i_d_q.d);
    motor->i_d_q = i_d_q;
    
    // 设置q轴电流PI控制器的参考值（目标值）
    motor->ctrl.pid_setRef(motor->ctrl.pid_currentLoop_iq, ref_iq);
    motor->ctrl.pid_setRef(motor->ctrl.pid_currentLoop_id, 0);
    
    // d轴电流环PI计算：通常目标是将d轴电流控制为0（最大转矩控制策略）
    // 输入为当前d轴电流，输出为d轴电压指令
    u_d_q.d = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_id, i_d_q.d);
    
    // q轴电流环PI计算：根据参考电流与实际电流的偏差计算q轴电压指令
    // q轴电流直接影响电机转矩，这里实现对转矩的闭环控制
    u_d_q.q = motor->ctrl.pid_cal(motor->ctrl.pid_currentLoop_iq, i_d_q.q);

    // 逆Park变换：将d-q坐标系下的电压指令转换回alpha-beta坐标系
    // 为后续的空间矢量脉宽调制(SVPWM)做准备
    FOC_InvPark(&u_d_q, motor->theta_e, &motor->u_alpha_beta);

    // 返回alpha-beta坐标系下的电压指令，用于SVPWM模块生成PWM信号
    return &motor->u_alpha_beta;
}

//速度环计算
float FOC_SpeedLoopCal(foc_motor_t * motor, float ref_speed)
{
	//�ٶȻ�PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_speedLoop, OMEGA_TO_RPM(motor->det_theta_m));    
}

//位置环计算
float FOC_PositionLoopCal(foc_motor_t * motor, float ref_position_rad)
{
	//λ�û�PID
    return motor->ctrl.pid_cal(motor->ctrl.pid_positionLoop, OMEGA_TO_RPM(motor->theta_m));    
}
/*
 * @brief: ���Ƕ�ӳ������ [-pi , pi]
 * @param[in]:
 * @return:
 */
float FOC_MapPi(float rad){
	while(rad>2*PI){
		rad-=2*PI;
	}
	while(rad<0){
		rad+=2*PI;
	}
	return rad;
}

/*
 * @brief: foc计算前的钩子函数
 * @param[in]:
 * @return:
 */
int Foc_AddCalBeforeHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func, void *param, const char* name)
{
    if(motor==NULL)return 1;
    for(uint8_t i =0; i < FOC_HOOK_NUMBER; i++){
        if(motor ->cal_before_hook[i].hook == NULL){
            motor ->cal_before_hook[i].hook = hook_func;
            motor ->cal_before_hook[i].name = name;
            motor ->cal_before_hook[i].param = param;
            return 0;
        }
    } 
    return 1;
}

/*
 * @brief: 远行Hook函数 foc前
 * @param[in]:
 * @return:
 */
void Foc_BeforeHookRun(foc_motor_t * motor)
{
    uint8_t i=0;
    if(motor){
        while(i<FOC_HOOK_NUMBER){
            if(motor->cal_before_hook[i].hook)
                motor->cal_before_hook[i].hook(motor->cal_before_hook[i].param);
            else
                return;
            i++;
        }   
    }
}

uint8_t Foc_SetBeforeHookParam(foc_motor_t * motor, const char*name, void*param)
{
    if(motor){
        for(uint8_t i=0; i < FOC_HOOK_NUMBER; i++){
            if(motor->cal_before_hook[i].hook){
                if(FOC_STRCMP(name,motor->cal_before_hook[i].name) == 0){
                    motor->cal_before_hook[i].param = param;
                    return 0;
                }
            }
        }    
    }
    return 1;
}

uint8_t Foc_SetAfterHookParam(foc_motor_t * motor, const char*name, void*param)
{
    if(motor){
        for(uint8_t i=0; i < FOC_HOOK_NUMBER; i++){
            if(motor->cal_after_hook[i].hook){
                if(FOC_STRCMP(name,motor->cal_after_hook[i].name) == 0){
                    motor->cal_after_hook[i].param = param;
                    return 0;
                }
            }
        }    
    }
    return 1;
}
/*
 * @brief: foc计算后钩子函数
 * @param[in]:
 * @return:
 */
int Foc_AddCalAfterHookFunc(foc_motor_t * motor, Foc_Hook_t hook_func, void *param, const char* name)
{
    if(motor==NULL)return 1;
    for(uint8_t i =0; i < FOC_HOOK_NUMBER; i++){
        if(motor ->cal_after_hook[i].hook == NULL){
            motor ->cal_after_hook[i].hook = hook_func;
            motor ->cal_after_hook[i].name = name;
            motor ->cal_after_hook[i].param = param;
            return 0;
        }
    } 
    return 1;
}

//转换为电角度
float Foc_Mech2ElecAngle(foc_motor_t * motor, float angle)
{
    if(!motor||motor->pole == 0)return -9999;
    return angle*motor->pole;
}

//转换为机械角度
float Foc_Elec2MechAngle(foc_motor_t * motor, float angle)
{
    if(!motor||motor->pole == 0)return -9999;
    return angle/motor->pole;
}

