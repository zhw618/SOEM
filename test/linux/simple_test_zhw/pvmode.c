#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

char IOmap[4096];

uint16 control;
int16 speed;
uint16 control_world;
uint16 state;
int32 curr_position;
int32 position;
int32 position2;
int add_position;
int i;
int ret;
char run = 1;
// 使从站进入op状态
void slavetop(int i)
{
	ec_slave[i].state = EC_STATE_OPERATIONAL;
	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUTRET);
	ec_writestate(0);

}
void endsignal(int sig)
{
	run = 0;
	printf("EtherCAT stop.\n");
	signal( SIGINT, SIG_DFL );
}
// 写目标位置
void position_to_add(int32 position)
{
	uint8 p1,p2,p3,p4,p5,p6;
	p1 = position % 255;
	p2 = position / 255;
	p3 = p2 % 255;
	p4 = p2 / 255;
	p5 = p4 % 255;
	p6 = p4 / 255;
	ec_slave[0].outputs[0x0002] = p1;
	ec_slave[0].outputs[0x0003] = p3;
	ec_slave[0].outputs[0x0004] = p5;
	ec_slave[0].outputs[0x0005] = p6;
	printf("%d--%d--%d--%d--%d\n",ec_slave[0].outputs[0x0002],ec_slave[0].outputs[0x0003],p5,p6,curr_position);
}
// 读取当前位置
int32 read_position()
{
	return ec_slave[0].outputs[0x000d] + (ec_slave[0].outputs[0x000e] << 8) + (ec_slave[0].outputs[0x000f] << 16);
}


void simpletest(char *ifname)
{
	if(ec_init(ifname))
	{
		printf("start ethernet at %s\n",ifname);
		if ( ec_config_init(FALSE) > 0 )
		{
			//发送 SDO写请求
			int8 mode=3;
			int8 flag1 = ec_SDOwrite(1, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM*3);  //csp是8,pv是3,pp是1

			printf("found %d slave on the bus\n",ec_slavecount);
			
			ec_config_map(&IOmap);
			ec_configdc();
			
			for(i=0;i<ec_slavecount;i++)
			{
				printf("slave%d to op\n", i);
				slavetop(i);
			}

			if(ec_slave[0].state == EC_STATE_OPERATIONAL)
			{
				ec_writestate(0);
				ec_configdc();

				while(run)
				{
					state = ec_slave[0].outputs[0x000a] + (ec_slave[0].outputs[0x000b]<<8);

					if((state & 0x004f) == 0x0040)
					{
						ec_slave[0].outputs[0x0000] = 0x06;
						ec_slave[0].outputs[0x0001] = 0x00;
					    //ec_slave[0].outputs[0x000c] = 0x08;// csp模式是8，pv模式是3
						printf("slave to op40,%X--%X\n",control_world,state);
					}
					else if((state & 0x006f) == 0x0021)
					{
						ec_slave[0].outputs[0x0000] = 0x07;
						ec_slave[0].outputs[0x0001] = 0x00;
						printf("slave to op21\n");
					}
					else if((state & 0x006f) == 0x0023)
					{
						ec_slave[0].outputs[0x0000] = 0x0f;
						ec_slave[0].outputs[0x0001] = 0x00;
						printf("slave to op23\n");
						printf("mode:%d\n",ec_slave[0].outputs[0x0c]);
					}
					else if((state & 0x006f) == 0x0027)
					{
						//pv模式写入速度
						//speed = 25000;
						//ec_SDOwrite(1, 0x60ff, 0x00, FALSE, sizeof(speed), &speed, EC_TIMEOUTRXM);
						ec_slave[0].outputs[0x0000] = 0x1f;
						ec_slave[0].outputs[0x0001] = 0x00;
						// 加速减速运行
						if(state == 0x1237)SDO
						{
							if(i < 100)
							{
								add_position++;
							}
							if(i > 5000 && i < 5100)
							{
								add_position--;
							}
							i++;
						}
						curr_position = read_position();// 读取位置
						position = curr_position + 10;
						position_to_add(position);// 将位置写入
					}
					else
					{
						ret = sizeof(control);
						ec_SDOread(1,0x6040,0x00,FALSE,&ret,&control,EC_TIMEOUTRXM);
						printf("why what who%X\n",control);
					}
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					usleep(500);// 周期大小
				}
			}
			else
			{
				printf("slave again to op\n");
			}
		}
		else
		{
			printf("no slave on the bus\n");
		}
	}
	else
	{
		printf("no ethernet card\n");
	}
}

int main(int argc, char *argv[])
{
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
	signal( SIGINT , endsignal );

	if (argc > 1)
	{      
		simpletest(argv[1]);
	}
	else
	{
		printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
	}   

	printf("End program\n");
	return (0);
}


