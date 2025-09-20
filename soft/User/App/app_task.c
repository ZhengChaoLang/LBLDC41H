#include "app_task.h"
#include "main.h"
#include "tim.h"
#include "drv_mc.h"
#include "foc.h"
#include "rtthread.h"
#include "drv_hall.h"
extern foc_motor_t foc_m1;
void app_test_thrad(void *arg);
int app_thread_creat()
{
    rt_thread_t app_test = rt_thread_create("app_test",app_test_thrad, NULL, 512, 15, 5);
    
    rt_thread_startup(app_test);
 
}
INIT_APP_EXPORT(app_thread_creat);
#include "foc.h"
extern float debug_theta;
extern uint32_t aa,bb,cc;
void app_test_thrad(void *arg)
{
    uint16_t ad_v =0;
    uint16_t ad_u =0;
    uint16_t ad_w =0;
    uint16_t dy =0;
    while(1){
        

        
        
//        ad_u = (uint16_t)(foc_m1.phase_i.a*100);
//        ad_v = (uint16_t)(foc_m1.phase_i.b*100);  
//        ad_w = (uint16_t)(foc_m1.phase_i.c*100);          
//        rt_kprintf("%d,%d,%d\n", ad_u, ad_v, ad_w);
        
//        rt_kprintf("%d,%d,%d\n", aa, bb, cc);
          foc_clark_prarm_t i_alph_beta;
          FOC_Clark(&foc_m1.phase_i, &i_alph_beta);

        FOC_Park(&i_alph_beta,foc_m1.theta_e, &foc_m1.i_d_q);
        //rt_kprintf("%d,%d,%d,%d\n", (int)(i_alph_beta.alpha*1000), (int)(i_alph_beta.beta*1000),(int)(foc_m1.i_d_q.d*1000), (int)(foc_m1.i_d_q.q*1000));
      /*   rt_kprintf("%d,%d,%d,%d\n", (int)(foc_m1.ctrl.pid_currentLoop_iq->ref*1000), (int)(foc_m1.i_d_q.q*1000),\
                    (int)(foc_m1.ctrl.pid_currentLoop_id->ref*1000), (int)(foc_m1.i_d_q.d*1000)); */

         rt_kprintf("%d,%d\n", (int)(foc_m1.ctrl.pid_speedLoop->ref*10), (int)(foc_m1.det_theta_e*10));         
        //rt_kprintf("%d,%d,%d\n", (int)(foc_m1.phase_i.a*1000), (int)(foc_m1.phase_i.b*1000),(int)(foc_m1.phase_i.c*1000));
        //rt_kprintf("%d,%d\n", (int)(foc_m1.i_d_q.d*1000), (int)(foc_m1.i_d_q.q*1000));
//        rt_kprintf("%d,%d,%d\n", (uint16_t)(foc_m1.ctrl.pid_currentLoop_iq->ref*100),\
//                            (uint16_t)(foc_m1.i_d_q.q*100), (uint16_t)(foc_m1.ctrl.pid_currentLoop_id->out_last*100));
        
        rt_thread_mdelay(1);
    }
}
 //INIT_APP_EXPORT(app_test_thrad);


void App_Foc_thread(void *arg)
{
    
    
    
    
    while(1)
    {
        
        
    
    
    }
}



