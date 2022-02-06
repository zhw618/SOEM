/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test. SOEM控制汇川SV660N伺服,进入CSV模式. 使用第2组PDO映射(0x1702和0x1B02)
 * 2022.2.6联机调试通过.
 * 
 * 参考: https://github.com/nicola-sysdesign/ewdl-test
 * 
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "ethercat.h"
#include "registry_idx.h"
#include "rxpdo.h"
#include "txpdo.h"
#include "time.h"  //tiemspec 加减转化等函数

#define EC_TIMEOUTMON 500

//char IOmap[4096]={};
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

uint16 flag1 = 0;

// ====== variables ===========================
uint16 ec_state_act = EC_STATE_NONE;

unsigned int t_cycle = 2000000U; //循环时间(单位ns): 2ms
int t_off = 0;

#define MAX_IO_MAP_SIZE 4096
uint8 io_map[MAX_IO_MAP_SIZE];

#define MAX_SLAVES_COUNT 10 //本主站程序带的最大站数量

RxPDO2t rx_pdo[MAX_SLAVES_COUNT]; //此处使用第2组PDO映射!!
TxPDO2t tx_pdo[MAX_SLAVES_COUNT];
// ======= end of variables=====================

#pragma region "配置SDO的工具函数"
static int moog_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}

static int moog_write16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}

static int moog_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
{
   int wkc;
   wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);
   while(ec_iserror()) { printf( ec_elist2string() ); }
   return wkc;
}
#pragma endregion "配置SDO的工具函数"

/* ======= functions ========================= */
//打印slave从站状态
inline void print_ec_state(uint16 slave_idx)
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

//进入SafeOP前的slave从站: PDO映射选择
int slave_setup(uint16 slave)
{
   int wkc = 0;

   /* ----------------配置SDO --------------------*/
   // Sync Managers assignment
   uint16 v_0x1c12=0x1702, v_0x1c13=0x1b02;   //使用第2组映射(0x1702和0x1b02)
   int len_0x1c12=2, len_0x1c13=2;
   //uint8 v0=0x00, v1=0x01;

   wkc += moog_write8 (slave, 0x1C12, 0, 0);
   wkc += moog_write8 (slave, 0x1C13, 0, 0);

   wkc += moog_write16 (slave, 0x1C12, 1, v_0x1c12);
   wkc += moog_write16 (slave, 0x1C12, 0, 1);

   wkc += moog_write16 (slave, 0x1C13, 1, v_0x1c13);
   wkc += moog_write16 (slave, 0x1C13, 0, 1);
   

   //运行模式设置为CSV模式
   int8 op_mode = CYCLIC_SYNCHRONOUS_VELOCITY;
   wkc += ec_SDOwrite(1, MODE_OF_OPERATION_IDX, 0x00, FALSE, sizeof(op_mode), &op_mode, EC_TIMEOUTSTATE);
   while(ec_iserror()) { printf(ec_elist2string()); }

   if(wkc!=7){
      printf("wkc=%d, Config SDO Failed!\n", wkc);
   }

   printf("--Debug: slave= %d \n", slave);
   int len_0x6061; 
   uint8 v_0x6061;
   wkc += ec_SDOread(slave, MODE_OF_OPERATION_DISPLAY_IDX, 0x00, FALSE, &len_0x6061, &v_0x6061, EC_TIMEOUTSTATE);
   printf("--Debug: MODE_OF_OPERATION_DISPLAY 0x6061= 0x%02x \n", v_0x6061);
   ec_SDOread(slave, 0x1c12, 0x01, FALSE, &len_0x1c12, &v_0x1c12, EC_TIMEOUTSTATE );
   ec_SDOread(slave, 0x1c13, 0x01, FALSE, &len_0x1c13, &v_0x1c13, EC_TIMEOUTSTATE );
   printf("--Debug: 0x1c12= 0x%04x, 0x1c13= 0x%04x \n", v_0x1c12, v_0x1c13);


   // // Sync Managers synchronization
   // uint16 sdo_1c32[] = {0x20, 0x0002};
   // uint16 sdo_1c33[] = {0x20, 0x0002};
   // //wkc += writeSDO<uint16>(slave, 0x1c32, 0x01, sdo_1c32[1]);
   // //wkc += writeSDO<uint16>(slave, 0x1c33, 0x01, sdo_1c33[1]);
   // wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(sdo_1c32[1]), &(sdo_1c32[1]), EC_TIMEOUTRXM);
   // wkc += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(sdo_1c33[1]), &(sdo_1c33[1]), EC_TIMEOUTRXM);

   /* Explicitly set flags that are (probably) invalid in EEPROM */
   ec_slave[slave].SM[2].SMflags = 0x10064;

   /* Explicitly disable sync managers that are activated by EEPROM */
   ec_slave[slave].SM[4].StartAddr = 0;
   ec_slave[slave].SM[5].StartAddr = 0;

   /* Set a slave name */
   strncpy (ec_slave[slave].name, "InoSV660N", EC_MAXNAME);

   return wkc;
}

// //子站名字的数组,比较是否都已配置.
// inline bool network_configuration()
// {
//     for (int i = 0; i < slaves.size(); i++)
//     {
//       const uint16 slave_idx = 1 + i;
//       if (strcmp(ec_slave[slave_idx].name, slaves[i].c_str()))
//       {
//         return false;
//       }
//     }
//     return true;
// }

//打印同步管理器SM 的相关信息
inline void print_sm(uint16 slave, int sm)
{
   uint16 A = ec_slave[slave].SM[sm].StartAddr;
   uint16 L = ec_slave[slave].SM[sm].SMlength;
   uint32 F = ec_slave[slave].SM[sm].SMflags;
   uint8 Type = ec_slave[slave].SMtype[sm];

   printf("SM%d - StartAddr:%4.4x Length:%4d Flags:%8.8x Type:%d\n", sm, A, L, F, Type);
}

//打印FMMU相关信息
inline void print_fmmu(uint16 slave, int fmmu)
{
   uint32 Ls = ec_slave[slave].FMMU[fmmu].LogStart;
   uint16 Ll = ec_slave[slave].FMMU[fmmu].LogLength;
   uint8 Lsb = ec_slave[slave].FMMU[fmmu].LogStartbit;
   uint8 Leb = ec_slave[slave].FMMU[fmmu].LogEndbit;
   uint16 Ps = ec_slave[slave].FMMU[fmmu].PhysStart;
   uint8 Psb = ec_slave[slave].FMMU[fmmu].PhysStartBit;
   uint8 Ty = ec_slave[slave].FMMU[fmmu].FMMUtype;
   uint8 Act = ec_slave[slave].FMMU[fmmu].FMMUactive;

   printf("FMMU%d - LogicStart:%.8x LogicLength:%4.2d Lsb:%d Leb:%d PhysStart:%.4x Psb:%d Type:%.2d Act:%.2d\n", fmmu, Ls, Ll, Lsb, Leb, Ps, Psb, Ty, Act);
}

// Fault Reset
/* When Fault happens, after the condition that caused the error has been
* resolved, write 80h to object 0x6040 to clear the error code in object
* 0x603F and object 0x200F. */
int fault_reset(uint16 slave_idx)
{
   uint16 control_word = 0x0080;
   //wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
   wkc += ec_SDOwrite(slave_idx, CONTROL_WORD_IDX, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTRXM);
   return wkc;
}

// Clear Alarm
/* When Warning happens, after the condition that caused the error has been
* resolved, write 01h to object 0x2006 to clear the error code in object
* 0x603F and object 0x200F. */
int clear_alarm(uint16 slave_idx)
{
   uint8 clear_alarm = 0x01;
   //wkc += writeSDO<uint8>(slave_idx, CLEAR_ALARM_IDX, 0x00, clear_alarm);
   wkc += ec_SDOwrite(slave_idx, CLEAR_ALARM_IDX, 0x00, FALSE, sizeof(clear_alarm), &clear_alarm, EC_TIMEOUTRXM);
   return wkc;
}

boolean ready_to_switch_on()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
   const uint16 slave_idx = 1 + i;
   rx_pdo[slave_idx].control_word = 0x0006;
   }
   return TRUE;
}

boolean switch_on()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
   const uint16 slave_idx = 1 + i;
   rx_pdo[slave_idx].control_word = 0x0007;
   }
   return TRUE;
}


boolean switch_off()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFE;
   }
   return TRUE;
}

boolean enable_operation()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
   }
   return TRUE;
}

// 同步DC时钟,计算出时间偏差
//   t_ref  -- (IN) 参考时钟,单位ns.
//   cycletime -- (IN) 循环周期,单位ns.
//   t_off  -- (OUT) 时钟偏差,单位ns.
void ec_sync(uint64 t_ref, uint64 cycletime, int32 *t_off)
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

//写出RxPDO到网络, 从网络中读取出TxPDO并处理同步
int update()
{
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      //rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
      RxPDO2_write_to_addr( rx_pdo + slave_idx, ec_slave[slave_idx].outputs );
   }

   ec_send_processdata();
   wkc += ec_receive_processdata(EC_TIMEOUTRET);

   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      //tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
      TxPDO2_read_from_addr( tx_pdo + slave_idx, ec_slave[slave_idx].inputs );
   }

   ec_sync(ec_DCtime, t_cycle, &t_off);
   return wkc;
}
/* ======= end of functions ========================= */

struct timespec t={}, t_1={}, t0_cmd={};
void* control_loop()
{
  clock_gettime(CLOCK_MONOTONIC, &t);  //当前时间,单位ns

  for (int iter = 1; iter < 1800000; iter++)
  {
    add_timespec(&t, t_cycle + t_off);

    struct timespec t_left;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);  //直到下次时间到才被唤醒,开始又一轮的处理

    struct timespec t_period;  //存放实际的周期
    diff_timespec(&t, &t_1, &t_period);  //计算实际周期值

   //若周期偏差大于±0.1ms,则打印
   unsigned long t_period_nsec = to_nsec(&t_period);
    if ( t_period_nsec < (t_cycle - 100000)   ||  t_period_nsec > (t_cycle + 100000) )
    {
      printf("t_period: %lu\n", t_period_nsec );
    }

    if (iter == 50)
    {
      printf("Fault Reset Slave 1... ");
      if (fault_reset(1))
      {
        printf("SUCCESS \n");
      }
      else
      {
         printf("FAILURE \n");
         return 0;
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
        return 0;
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
        return 0;
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
        return 0;
      }
    }

    if (iter == 500) t0_cmd = t;
    if (iter >= 500)
    {
      struct timespec t_cmd;
      diff_timespec(&t, &t0_cmd, &t_cmd);

      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;

        uint16 error_code = tx_pdo[slave_idx].error_code;
        uint16 status_word = tx_pdo[slave_idx].status_word;
        int32 position_actual_value = tx_pdo[slave_idx].position_actual_value;
        //int32 velocity_actual_value = tx_pdo[slave_idx].velocity_actual_value;

        //int32 target_position = 0;  //若设置位置0,刚启动时可能会有跳动.
        //int32 target_position = position_actual_value;  //保持实际位置,避免跳动!
        //rx_pdo[slave_idx].target_position = target_position;
        int8 mode_of_operation = CYCLIC_SYNCHRONOUS_VELOCITY;
        //int8 mode_of_operation = PROFILE_VELOCITY;
        int32 target_velocity = 10;  //目标速度值,单位:编码器脉冲数/s.
        //int32 direct = ( iter % 10000 < 5000 )?1:(-1);  //1万次是20秒,每10s改变一次方向
        //target_velocity *= direct;
        rx_pdo[slave_idx].mode_of_operation = mode_of_operation;
        rx_pdo[slave_idx].target_velocity = target_velocity;


        if(iter % 1000 == 0){
           printf("iter=%d ---------------------------\n", iter);
           printf(" error_code: 0x%04x\n", error_code);
           printf(" status_word: 0x%04x\n", status_word);
           printf(" position_actual_value: %d\n", position_actual_value);
           printf(" target_velocity: %d\n", target_velocity);
        }
      }
    }

    update();
    t_1 = t;
  }

  printf("Finished.\n");
  return NULL;
}



void simpletest(char *ifname)
{
   //int i, j, oloop, iloop, chk;
   needlf = FALSE;
   inOP = FALSE;

   printf("Starting simple test idle\n");

#pragma region ec_master.init()

   /* =====================
    *  ec_master.init()
    * ===================== */
   if (ec_init(ifname) > 0)
   {
      printf("EtherCAT socket on: %s\n", ifname);
   }
   else
   {
      printf("Coludn't initialize EtherCAT Master socket on: %s\n", ifname);
      return;
   }

   if (ec_config_init(FALSE) > 0)
   {
      printf("Slaves found and configured: %d\n", ec_slavecount);
   }
   else
   {
      printf("Coludn't find and configure any slave.\n");
      return;
   }

   // PRE OPERATIONAL
   ec_state_act = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
   print_ec_state(0);

   // network configuration
   //  if (!network_configuration())
   //  {
   //    printf("Mismatch of network units!\n");
   //    return false;
   //  }

   // Distributed Clock
   ec_configdc();
   for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
   {
      ec_dcsync0(slave_idx, TRUE, t_cycle, 0);
   }

   // 设置hook函数: Pre-Operational -> Safe-Operational
   for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
   {
      ec_slave[slave_idx].PO2SOconfig = slave_setup;
   }

   unsigned int used_mem = ec_config_map(&io_map);
   if (used_mem > sizeof(io_map))
   {
      printf("IO Map size: %d > MAX_IO_MAP_SIZE: %du\n", used_mem, sizeof(io_map));
      return;
   }
   printf("io_map size: %d\n", used_mem);

   // print slaves configuration
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      print_sm(slave_idx, 0);   // SM0
      print_sm(slave_idx, 1);   // SM1
      print_sm(slave_idx, 2);   // SM2 (output)
      print_sm(slave_idx, 3);   // SM3 (input)
      print_fmmu(slave_idx, 0); // FMMU0
      print_fmmu(slave_idx, 1); // FMUU1
   }

   // SAFE OPERATIONAL
   ec_state_act = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
   print_ec_state(0);

   //清除所有站的 fault 和 alarm 信息
   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
      clear_alarm(slave_idx);
   }

   //=======end of ec_master.init() =================
#pragma endregion ec_master.init()

   // Start
   // const int n_slaves = MAX_SLAVES_COUNT;
   // int a_pos[n_slaves];
   // int a_vel[n_slaves];
   // int a_eff[n_slaves];
   // int a_pos_cmd[n_slaves];
   // int a_vel_cmd[n_slaves];
   // int a_eff_cmd[n_slaves];

#pragma region ec_master.start()

   /* =====================
    *  ec_master.start()
    * ===================== */

   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
      //rx_pdo[slave_idx].mode_of_operation = 0;
      rx_pdo[slave_idx].target_position = 0;
      rx_pdo[slave_idx].touch_probe_function = 0;
      //rx_pdo[slave_idx].physical_outputs = 0x0000;
   }

   update();

   for (int i = 0; i < ec_slavecount; i++)
   {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
   }

   // OPERATIONAL
   // ec_state_act = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
   // print_ec_state(0);

//=======end of ec_master.start() =================
#pragma endregion ec_master.start()

#pragma region "POSIX Thread for cyclic loops"
   //POSIX Thread for cyclic loops
   (* control_loop)();

#pragma endregion "POSIX Thread for cyclic loops"

   //结束程序
   // printf("输入任意字符,停止servo motor运行.");
   // char c = getchar();

   printf("\nRequest init state for all slaves\n");
   ec_slave[1].state = EC_STATE_INIT;
   /* request INIT state for all slaves */
   ec_writestate(0);

   printf("End simple test, close socket\n");
   /* stop SOEM, close socket */
   ec_close();
 
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
   int slave;
   (void)ptr; /* Not used */

   while (1)
   {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
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
               else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
               {
                  printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                  ec_slave[slave].state = EC_STATE_OPERATIONAL;
                  ec_writestate(slave);
               }
               else if (ec_slave[slave].state > EC_STATE_NONE)
               {
                  if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d reconfigured\n", slave);
                  }
               }
               else if (!ec_slave[slave].islost)
               {
                  /* re-check state */
                  ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                  if (ec_slave[slave].state == EC_STATE_NONE)
                  {
                     ec_slave[slave].islost = TRUE;
                     printf("ERROR : slave %d lost\n", slave);
                  }
               }
            }
            if (ec_slave[slave].islost)
            {
               if (ec_slave[slave].state == EC_STATE_NONE)
               {
                  if (ec_recover_slave(slave, EC_TIMEOUTMON))
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d recovered\n", slave);
                  }
               }
               else
               {
                  ec_slave[slave].islost = FALSE;
                  printf("MESSAGE : slave %d found\n", slave);
               }
            }
         }
         if (!ec_group[currentgroup].docheckstate)
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
      //pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
      osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);

      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      ec_adaptert *adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf("\nAvailable adapters:\n");
      adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }

   printf("End program\n");
   return (0);
}
