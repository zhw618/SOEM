/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096]={};
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

uint16 flag1=0;

// //定义输出映像字段
// typedef struct {
//     uint16 control_word;
//     uint32 target_pos;
//     uint16 probe_function;
//     uint32 force_DO;
// } Omap_t;
// Omap_t * Omap;

// //定义输入映像字段
// typedef struct {
//    uint16 error_code;
//    uint16 state_word;
//    uint32 current_pos;
//    uint16 current_torque;
//    uint32 pos_diff;
//    uint16 probe_status;
//    uint32 probe1_pos;
//    uint32 probe2_pos;
//    uint32 DI;
// } Imap_t;
// Imap_t * Imap;

void simpletest(char *ifname)
{
    int i, j, oloop, iloop; //, chk;
    needlf = FALSE;
    inOP = FALSE;

   printf("Starting simple test zhw\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname)<=0)
   {  //初始化网络接口失败.
      printf("No socket connection on %s\nExecute as root\n",ifname);
      return;
   }

   printf("ec_init on %s succeeded.\n",ifname);
   
   /* find and auto-config slaves,寻找并自动配置slave从站 */
   if ( ec_config_init(FALSE) <= 0 )
   {  //未找到任何从站
      printf("No slaves found!\n");
      
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
      return;
   }

   //ec_config_init(FALSE)调用成功后,此时找到的slave信息已放在ec_slave[]全局数组中.
   printf("%d slaves found and configured.\n",ec_slavecount);
   //  其中ec_slavecount 为 soem/ethercatmain.c 中定义的全局变量

   //映射PDO-->IOMap内存中.
   ec_config_map(&IOmap);
   uint16 * p_control_word = (uint16*) IOmap;
   int32 * p_target_pos = (int32*) (IOmap + 2);
   uint16 * p_status_word = (uint16*) (IOmap + 12 + 2);


   printf("Slaves mapped, state to SAFE_OP.\n");
   /* wait for all slaves to reach SAFE_OP state */
   flag1 = ec_statecheck(1, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
   if(flag1 != EC_STATE_SAFE_OP)
   {
      printf("slave1 未切换到Safe_OP模式\n");
   }

     //配置DC(分布式时钟)
   uint16 flag1 = ec_configdc();
   if(flag1==1){
      printf("已找到:带DC的从站.\n");
   }
   //ec_dcsync0(1,TRUE,2000000,0);

   //output 和 input 的所需的字节数
   oloop = ec_slave[1].Obytes;
   if ((oloop == 0) && (ec_slave[1].Obits > 0)) oloop = 1;
   if (oloop > 8) oloop = 8;
   iloop = ec_slave[1].Ibytes;
   if ((iloop == 0) && (ec_slave[1].Ibits > 0)) iloop = 1;
   if (iloop > 8) iloop = 8;

   printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments,
                                          ec_group[0].IOsegment[0],
                                          ec_group[0].IOsegment[1],
                                          ec_group[0].IOsegment[2],
                                          ec_group[0].IOsegment[3]);
   
   //请求OP模式并等待,检查状态结果: 先取消掉(State=0x14 StatusCode=0x0027 : Freerun not supported).
   // printf("Request operational state for all slaves\n");
   // expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   // printf("Calculated workcounter %d\n", expectedWKC);
   // ec_slave[0].state = EC_STATE_OPERATIONAL;
   // /* send one valid process data to make outputs in slaves happy*/
   // ec_send_processdata();
   // ec_receive_processdata(EC_TIMEOUTRET);
   // /* request OP state for all slaves */
   // ec_writestate(0);
   // chk = 200;
   // /* wait for all slaves to reach OP state */
   // do
   // {
   //    ec_send_processdata();
   //    ec_receive_processdata(EC_TIMEOUTRET);
   //    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
   // }
   // while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));


   //检查并确保全部slaves都是OP状态
   // if (ec_slave[0].state != EC_STATE_OPERATIONAL )
   // {
   //    printf("Not all slaves reached operational state.\n");
   //    ec_readstate();
   //    for(i = 1; i<=ec_slavecount ; i++)
   //    {
   //       if(ec_slave[i].state != EC_STATE_OPERATIONAL)
   //       {
   //          printf("  Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
   //                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, 
   //                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
   //       }
   //    }

   //    printf("\nRequest init state for all slaves\n");
   //    ec_slave[0].state = EC_STATE_INIT;
   //    /* request INIT state for all slaves */
   //    ec_writestate(0);

   //    printf("End simple test, close socket\n");
   //    /* stop SOEM, close socket */
   //    ec_close();

   //    return;
   // }

   
   //已验证:所有slaves均为OP
   //printf("Operational state reached for all slaves.\n");
   //inOP = TRUE;
   
   //计算WKC期望值
   //expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   //输出位置递增
   int32 pos_value = 0;
   boolean first =0;

   /* cyclic loop */
   for(i = 1; i <= 10000; i++)
   {
      ec_dcsync0(1,TRUE,2000000,0);  //设置从站1的DC sync0 周期20ms, 周期抖动1ms

      if(i%100 == 0){
         printf("status word: 0x%04d\n", (*p_status_word) );
      }

      //启动电机
      if( (*p_status_word)==0 ){

         if(first ==0)
         {
            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
            first=1;
         }
   
      }else if( ((*p_status_word) & 0x004f)== 0x0040 ){
         (*p_control_word) = 0x0006;

      }else if( ((*p_status_word) & 0x006f) == 0x0021 ){
         (*p_control_word) = 0x0007;

      }else if( ((*p_status_word) & 0x006f) == 0x0023 ){
         (*p_control_word) = 0x000f;
         (*p_target_pos) = 0;
      
      }else if( ((*p_status_word) & 0x006f) == 0x0027 ){
         //Operation Enable
         if(i%100 == 0){

            pos_value += 2000;
            (*p_target_pos) = pos_value ;
         }

      }

      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      

      //检测wkc处理error情况
      if(wkc >= expectedWKC)
      {

         printf("Processdata cycle %4d, WKC %d , O:", i, wkc);
         for(j = 0 ; j < oloop; j++)
         {
               printf(" %2.2x", *(ec_slave[1].outputs + j));
         }

         printf(" I:");
         for(j = 0 ; j < iloop; j++)
         {
               printf(" %2.2x", *(ec_slave[1].inputs + j));
         }
         printf(" T:%"PRId64"\r",ec_DCtime);
         needlf = TRUE;
      }

      //5000us = 5ms, 重复10000为 50s
      osal_usleep(1500);

   }

   printf("循环次数耗尽, 从 while 中 正常退出.\n");
   inOP = FALSE;

   printf("\nRequest init state for all slaves\n");
   ec_slave[1].state = EC_STATE_INIT;
   /* request INIT state for all slaves */
   ec_writestate(0);


   printf("End simple test, close socket\n");
   /* stop SOEM, close socket */
   ec_close();
 
    
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test zhw\n");

   if (argc > 1)
   {
      /* create thread to handle slave error handling in OP */
   // pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      //osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      ec_adaptert * adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n");
   return (0);
}
