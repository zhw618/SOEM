/* *****************************************************************************
 * SOEM控制伺服电机(汇川SV660N), PP模式/PV模式, 采用第2组PDO映射
 * 参考csdn: https://blog.csdn.net/weixin_44880138/article/details/95632913
 * 
 * 使用PDO方式高速传送速度/位置指令!
 * 
 *  * Usage : pppv_pdo2 [ifname1]
 *      ifname is NIC interface, f.e. eth0
 *
 * wrritted by zhw618@qq.com, 
 * 2022.02.08 凌晨00点-上午10点(不间断工作10个小时调试成功!!)
 * ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <signal.h>

#include "ethercat.h"

#define NSEC_PER_SEC 1000000000  //每秒有多少ns
/* Mode of Operation */
enum mode_of_operation_t
{
  NONE = 0,                          // None Mode
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  TORQUE_PROFILE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  CYCLIC_SYNCHRONOUS_TORQUE = 10,    // Cyclic Synchronous Torque Mode
};
int8  User_Selected_Mode = PROFILE_VELOCITY;  //用户选中的运行模式#
//int8  User_Selected_Mode = PROFILE_POSITION;  //用户选中的运行模式#

/* ----   映射空间 ---*/ 
char IOmap[4096]={};
//uint8 * AddrOut = ec_slave[1].outputs;
uint16 * p_control_word    ;//uint16*) IOmap;
int32 * p_target_position  ;//int32*) (IOmap +2);
int32 * p_target_velocity  ;//int32*) (IOmap +2 +4);
int16 * p_target_torque    ;//int16*) (IOmap +2 +4 +4);
int8 * p_operation_mode    ;//int8*)  (IOmap +2 +4 +4 +2);
uint32 * p_max_velocity    ;
//input//
uint16* p_error_code       ;//(uint16*) (IOmap + 19);
uint16* p_status_word      ;//(uint16*) (IOmap + 19 +2);
int32 * p_current_position ;//(int32*)  (IOmap + 19 +2 +2);
int16 * p_current_torque   ;//(int16*)  (IOmap + 19 +2 +2 +4);
int8 *  p_current_mode     ;//(int8*)   (IOmap + 19 +2 +2 +4 +2);

boolean flag_Slave1Run = 0 ;  //OP后若此flag置0, 则传递停机信号.

unsigned int t_cycle = 2000000U; //循环时间(单位ns): 2ms
int t_off = 0;  //与DC参考时钟的偏差补偿值


#pragma region "配置SDO的工具函数"
static int moog_write8 (uint16 slave, uint16 index, uint8 subindex, int8 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}

static int moog_read8 (uint16 slave, uint16 index, uint8 subindex, void * pvalue)
{
   int wkc;  int len=sizeof(uint8);
   wkc = ec_SDOread (slave, index, subindex, FALSE, &len, pvalue, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}

static int moog_write16 (uint16 slave, uint16 index, uint8 subindex, int16 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}
static int moog_read16 (uint16 slave, uint16 index, uint8 subindex, void * pvalue)
{
   int wkc;  int len=sizeof(uint16);
   wkc = ec_SDOread (slave, index, subindex, FALSE, &len, pvalue, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}

static int moog_write32 (uint16 slave, uint16 index, uint8 subindex, int32 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}
static int moog_read32 (uint16 slave, uint16 index, uint8 subindex, void * pvalue)
{
   int wkc;  int len=sizeof(uint16);
   wkc = ec_SDOread (slave, index, subindex, FALSE, &len, pvalue, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}
#pragma endregion "配置SDO的工具函数"


#pragma region "同步时钟相关"

//两个timespec相加
inline void add_timespec(struct timespec *ts, int64_t addtime)
{
  int64_t sec, nsec;
  nsec = addtime % NSEC_PER_SEC;
  sec = (addtime - nsec) / NSEC_PER_SEC;

  ts->tv_sec += sec;
  ts->tv_nsec += nsec;
  if (ts->tv_nsec >= NSEC_PER_SEC)
  {
    nsec = ts->tv_nsec % NSEC_PER_SEC;
    ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
    ts->tv_nsec = nsec;
  }
}

//时间相减
inline void diff_timespec(const struct timespec * t1, const struct timespec * t2, struct timespec * t_diff)
{
  if ((t1->tv_nsec - t2->tv_nsec) < 0)
  {
    t_diff->tv_sec = t1->tv_sec - t2->tv_sec - 1;
    t_diff->tv_nsec = t1->tv_nsec - t2->tv_nsec + 1000000000;
  }
  else
  {
    t_diff->tv_sec = t1->tv_sec - t2->tv_sec;
    t_diff->tv_nsec = t1->tv_nsec - t2->tv_nsec;
  }
}

//转化为秒
inline double to_sec(const struct timespec * t)
{
  return t->tv_sec + t->tv_nsec / 1000000000.0;
}

//转化为纳秒
inline unsigned long to_nsec(const struct timespec * t)
{
  return t->tv_sec * NSEC_PER_SEC + t->tv_nsec;
}

// 同步DC时钟,计算出时间偏差
//   t_ref  -- (IN) 参考时钟,单位ns.
//   cycletime -- (IN) 循环周期,单位ns.
//   t_off  -- (OUT) 时钟偏差,单位ns.
void ec_sync(uint64 t_ref, uint64 cycletime, int32 * t_off)
{
   static int32 integral = 0;  //积分值
   int64 delta = (t_ref - (cycletime / 2)) % cycletime;  //取模

   if (delta > (int64)(cycletime / 2))
   {
      delta = delta - cycletime;
   }
   if (delta > 0) integral++;
   if (delta < 0) integral--;

   *t_off = -(delta / 100) - (integral / 20);
}

#pragma endregion "同步时钟相关"

//检查并打印slave从站状态, 0为主站
void print_ec_state(uint16 slave_idx)
{
   switch (ec_slave[slave_idx].state)
   {
   case EC_STATE_NONE:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "NONE");
      break;
   case EC_STATE_INIT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "INIT");
      break;
   case EC_STATE_PRE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "PRE_OP");
      break;
   case EC_STATE_BOOT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "BOOT");
      break;
   case EC_STATE_SAFE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "SAFE_OP");
      break;
   case EC_STATE_OPERATIONAL:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "OPERATIONAL");
      break;
   //case EC_STATE_ACK:
   //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
   //  break;
   case EC_STATE_PRE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "PRE_OP", "ERROR");
      break;
   case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "SAFE_OP", "ERROR");
      break;
   case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "OPERATIONAL", "ERROR");
      break;
   }
}


#pragma region "控制字操作函数"

// Fault Reset
/* When Fault happens, after the condition that caused the error has been
* resolved, write 80h to object 0x6040 to clear the error code in object
* 0x603F and object 0x200F. */
int fault_reset(uint16 slave_idx)
{  
   int wkc=0;
   uint16 control_word = 0x0080;
   //wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
   wkc += ec_SDOwrite(slave_idx, 0x6040, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTRXM);
   return wkc;
}

// Clear Alarm
/* When Warning happens, after the condition that caused the error has been
* resolved, write 01h to object 0x2006 to clear the error code in object
* 0x603F and object 0x200F. */
// int clear_alarm(uint16 slave_idx)
// {
//    int wkc=0;
//    uint8 clear_alarm = 0x01;
//    //wkc += writeSDO<uint8>(slave_idx, CLEAR_ALARM_IDX, 0x00, clear_alarm);
//    wkc += ec_SDOwrite(slave_idx, 0x2006, 0x00, FALSE, sizeof(clear_alarm), &clear_alarm, EC_TIMEOUTRXM);
//    return wkc;
// }

boolean ready_to_switch_on()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      //const uint16 slave_idx = 1 + i;
      //rx_pdo[slave_idx].control_word = 0x0006;
      *p_control_word =  0x0006;
   }
   return TRUE;
}

boolean switch_on()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      //const uint16 slave_idx = 1 + i;
      // rx_pdo[slave_idx].control_word = 0x0007;
      *p_control_word =  0x0007;
   }
   return TRUE;
}

boolean enable_operation()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      //const uint16 slave_idx = 1 + i;
      // rx_pdo[slave_idx].control_word = 0x000F;
      *p_control_word =  0x000F;
   }
   return TRUE;
}

boolean switch_off()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      //const uint16 slave_idx = 1 + i;
      // rx_pdo[slave_idx].control_word &= 0xFFFE;
      *p_control_word =  0xFFFE;
   }
   return TRUE;
}

#pragma endregion "控制字操作函数"

//写出RxPDO到网络, 从网络中读取出TxPDO, 计算出DC时钟补偿偏差 输出给 t_off
int stack_update(int32 * t_off)
{
    int wkc=0;
//    for (int i = 0; i < ec_slavecount; i++)
//    {
//       const uint16 slave_idx = 1 + i;
//       //rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
//       RxPDO0_write_to_addr( rx_pdo + slave_idx, ec_slave[slave_idx].outputs );
//    }

   ec_send_processdata();
   wkc += ec_receive_processdata(EC_TIMEOUTRET);

//    for (int i = 0; i < ec_slavecount; i++)
//    {
//       const uint16 slave_idx = 1 + i;
//       //tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
//       TxPDO0_read_from_addr( tx_pdo + slave_idx, ec_slave[slave_idx].inputs );
//    }

   ec_sync(ec_DCtime, t_cycle, t_off);
   return wkc;
}

// Interrupt信号(SIGINT=2)处理函数
void endSignal(int sig )
{
    printf("\nReceived Signal Interrupt~~~ EtherCAT do Quick-Stop.\n\n");

    flag_Slave1Run = 0;  //传递到 control_loop()中,执行停止动作.

    sleep(3);
	 //signal( SIGINT, SIG_DFL );
    signal( sig, SIG_DFL );
    exit(EXIT_SUCCESS);  //退出整个进程.
   
}

// ===== 一些SDO配置 =============================
int some_SDO_config(uint16 slave)
{
   int wkc=0;
   //moog_write8(slave, 0x6060, 0x00, 3); //pv模式3, pp模式1

   //PV模式下,向0x60FF中写入目标速度值
   //moog_write32(slave, 0x607F, 0x00, 100000000); // max_target_velocity
   //moog_write32(slave, 0x60FF, 0x00, 8000000); // target_velocity

   /* ----------------配置映射SDO --------------------*/
   uint16 v_0x1c12=0x1702, v_0x1c13=0x1b02;   //使用第2组映射(0x1702和0x1b02)
   wkc=0;
   wkc += moog_write8 (slave, 0x1C12, 0, 0);
   wkc += moog_write8 (slave, 0x1C13, 0, 0);
   wkc += moog_write16 (slave, 0x1C12, 1, v_0x1c12);
   wkc += moog_write16 (slave, 0x1C13, 1, v_0x1c13);
   wkc += moog_write16 (slave, 0x1C12, 0, 1);
   wkc += moog_write16 (slave, 0x1C13, 0, 1);
   
   if(wkc!=6){
      printf("wkc=%d, Config PDO Mapping Failed!\n", wkc);
   }

   //显示 PDO Mapping的配置结果
   uint8 v_0x6061;
   moog_read8(slave, 0x6061, 0, &v_0x6061);
   printf("--Debug: MODE_OF_OPERATION_DISPLAY 0x6061= 0x%02x \n", v_0x6061);
   moog_read16(slave, 0x1c12, 0x01, &v_0x1c12 );
   moog_read16(slave, 0x1c13, 0x01, &v_0x1c13 );
   printf("--Debug: 0x1c12= 0x%04x, 0x1c13= 0x%04x \n", v_0x1c12, v_0x1c13);

   // ============  结束SDO配置 =============================
   return wkc;
}


void* control_loop()
{ 

   struct timespec ts={}, ts_1={}, ts_left={}; 
   struct timespec ts_period;  //存放实际的周期
   clock_gettime(CLOCK_MONOTONIC, &ts);  //获取当前时间,单位ns

   uint64 FOREVER = 18446744073709551615LLU;  //2^64-1, 按纳秒算也有五百多年.
   //for (uint64 iter = 1; iter <= 60000; iter++)
   for (uint64 iter = 1; iter <= FOREVER; iter++)
   {
        add_timespec(&ts, t_cycle + t_off);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &ts_left);  //直到下次时间到才被唤醒,开始又一轮的处理
        diff_timespec(&ts, &ts_1, &ts_period);  //计算实际周期值
        ts_1 = ts;   //保存此次唤醒时间, 供下次循环使用.

        //若周期偏差大于±0.1ms,则打印
        unsigned long t_period_nsec = to_nsec(&ts_period);
        if ( t_period_nsec < (t_cycle - 100000)   ||  t_period_nsec > (t_cycle + 100000) )
        {
            printf( "[Warning]循环同步周期抖动: iter= %"PRIu64", t_period(ns): %lu\n", iter, t_period_nsec );
        }

        if (iter == 50)
        {
            printf("Fault Reset Slave 1... ");
            if (fault_reset(1)) {
               printf("SUCCESS \n");
            }else{
               printf("FAILURE \n");
               return NULL;
            }
         }

      if (iter == 100)
      {
         printf("Ready to Switch On ... ");
         if (ready_to_switch_on())
         {
            printf("SUCCESS \n");
         }
         else
         {
            printf("FAILURE \n");
            return NULL;
         }
      }

      if (iter == 200)
      {
         printf("Switch On ... ");
         if (switch_on())
         {
            printf("SUCCESS \n");
         }
         else
         {
            printf("FAILURE \n");
            return NULL;
         }
      }

      if (iter == 400)
      {
         printf("Enable Operation ... ");
         if (enable_operation())
         {
            printf("SUCCESS \n");
         }
         else
         {
            printf("FAILURE \n");
            return NULL;
         }
      }
      
      if ( iter >= 500  )
      {
         // struct timespec ts_cmd;
         // diff_timespec(&ts, &ts0_cmd, &ts_cmd);  //本次与iter=500次的时间差.(并未使用)

         if( (iter-500)%3000 <2000 ){
             *p_target_velocity = -15000000;   //顺时针转2s,慢进 -15000000
         }else{
             *p_target_velocity = 30000000;    //逆时针倒1s,快退 30000000
         }
             
      }

      if(iter%1500==0){
            printf("--- iter= %"PRIu64" \n", iter);
            printf("  show Error_Code :  0x%04x \n", *p_error_code);
            printf("  show Status_Word:  0x%04x \n", *p_status_word);
            printf("  show Cur_Position: %d \n",   *p_current_position);
            printf("  show Cur_Torque:   %d \n",   *p_current_torque);
            printf("  show Cur_Op_Mode:  %02x \n", *p_current_mode);

            printf("  show Target_Velovity:  %d \n", *p_target_velocity);

      }
        
        //输出PDO实时值
        *p_target_position = 8000;   //在pv模式下,目标位置失效
        //*p_control_word= 0x000F;
        //*p_target_velocity = 5000000;
        //*p_target_torque = 0;
        *p_operation_mode = User_Selected_Mode;  //pv=3
        *p_max_velocity = 50000000;  //必须最大值一起发送,否则速度为0不转!!
        
        if(flag_Slave1Run==0)  //若收到信号则停机
        {
           printf("--Debug: control_loop 收到停机信号!");
           *p_control_word = 0x02;
           //*p_target_velocity = 0;
           return NULL;
        }
        
        stack_update(&t_off);

   }
   return NULL;
}


void simpletest(char *ifname)
{
    //int i, j, oloop, iloop; //, chk;
    //needlf = FALSE;
    //inOP = FALSE;
    //int wkc=0;
    
   printf("Starting PP/PV mode SDO control test~\n");

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
   printf("%d slaves found and configured.\n",ec_slavecount); //其中ec_slavecount 为 soem/ethercatmain.c 中定义的全局变量
   
   //检查 PreOP状态
   ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   print_ec_state(0);
   print_ec_state(1);

   // Distributed Clock
   boolean flag2 = ec_configdc();
    if(flag2==1){
      printf("已找到带DC的从站,开始配置...\n");
   }
   for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
   {
      ec_dcsync0(slave_idx, TRUE, t_cycle, 0); //cyclic time 2ms, cyclicShift 0
   }
     

   // 设置hook函数: Pre-Operational -> Safe-Operational
   for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
   {
      //在 PreOP 模式下做一些SDO配置
      ec_slave[slave_idx].PO2SOconfig = some_SDO_config;
   }
   

   //映射PDO-->IOMap内存中.
   int used_mem = ec_config_map(&IOmap);
   //output
   uint8 * AddrOut = ec_slave[1].outputs;
   p_control_word     = (uint16*)(AddrOut);
   p_target_position  = (int32*) (AddrOut +2);
   p_target_velocity  = (int32*) (AddrOut +2 +4);
   p_target_torque    = (int16*) (AddrOut +2 +4 +4);
   p_operation_mode   = (int8*)  (AddrOut +2 +4 +4 +2);
   p_max_velocity     = (uint32*)(AddrOut +2 +4 +4 +2 +1 +2);
   //input
   uint8 * AddrIn = ec_slave[1].inputs;
   p_error_code       = (uint16*) (AddrIn);
   p_status_word      = (uint16*) (AddrIn +2);
   p_current_position = (int32*)  (AddrIn +2 +2);
   p_current_torque   = (int16*)  (AddrIn +2 +2 +4);
   p_current_mode     = (int8*)   (AddrIn +2 +2 +4 +2);
   printf("io_map size: %d\n", used_mem);


   printf("Slaves mapped, state to SAFE_OP.\n");
   /* wait for all slaves to reach SAFE_OP state */
   uint16 flag1 = ec_statecheck(1, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
   if( flag1 != EC_STATE_SAFE_OP)
   {
      printf("slave1 未切换到Safe_OP模式\n");
   }
   print_ec_state(0);
   print_ec_state(1);

   //清除所有站的 fault 和 alarm 信息
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
      //clear_alarm(slave_idx);
   }
   
   //=======end of ec_master.init() =================

   //=======begin of ec_master.start() =================
   printf("Request operational state for all slaves\n");
   for (int i = 0; i < ec_slavecount; i++)
   {
      //const uint16 slave_idx = 1 + i;
      //rx_pdo[slave_idx].control_word = 0x0006;
      //*p_control_word = 0x0006;
      //rx_pdo[slave_idx].target_position = 0;
      //rx_pdo[slave_idx].target_velocity = 0;
      //*p_target_velocity = 5000000;
      //rx_pdo[slave_idx].target_torque = 0;
      //rx_pdo[slave_idx].mode_of_operation = 0;
      *p_operation_mode = User_Selected_Mode;    //pv为3, pp为1
      //rx_pdo[slave_idx].touch_probe_function = 0;
      //rx_pdo[slave_idx].max_velocity = 0x0000;
      //*p_max_velocity = 100000000;
   }

   stack_update(&t_off);

   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
   }
   
   flag_Slave1Run = 1;  //设置启动OP标记.
   //=======end of ec_master.start() =================

#pragma region "POSIX Thread for cyclic loops"
   //POSIX Thread for cyclic loops
   control_loop();

#pragma endregion "POSIX Thread for cyclic loops"


   //sleep(10);
   printf("\n [注意]程序已从 control_loop 中正常退出.\n");

   printf("Request stop for all slaves\n");
   //向控制字0x6040中写入0x0002 
   moog_write16(1, 0x6040, 0x00, 0x0000); // quick_stop(0x0002)

   printf("End simple test, close socket\n");
   /* stop SOEM, close socket */
   ec_close();
 
    
}



int main(int argc, char *argv[])
{
   printf("SOEM PP/PV mode run test by zhw\n");
   signal( SIGINT , endSignal );

   strcpy( ec_slave[0].name, "RPI-SOEM");  //设置主站name

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
      printf("Usage: pv_sdo ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n\n");
   return (0);
}
