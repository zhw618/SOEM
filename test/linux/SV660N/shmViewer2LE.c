/* **********************************************
 *  共享内存SHM的内容,存在大小端的问题
 *   本工具将SHM视为大端存放,将它转化为小端后再查看.
 * 
 * **********************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "servoshm.h"
#include "servoPDO.h"

char * IOmap = NULL;  //共享内存指针,用于IOMap映射
RxPDO1t * pRxPDO1;    //slave1的 RxPDO结构体
TxPDO1t * pTxPDO1;    //slave1的 TxPDO结构体

/*--------开辟2个新的,存放转化后的小端结果 -----*/
RxPDO1t  RxPDO1_le = {};
TxPDO1t  TxPDO1_le = {};


void print_TxPDO(TxPDO1t * pTxPDO1){

    printf("TxPDO (servo output)---------------[已转换大小端后显示]-----\n");
    printf("[0] Error_Code: 0x%04x \n",     pTxPDO1->error_code );
    printf("[1] Status_Word: 0x%04x \n",    pTxPDO1->status_word );
    printf("[2] Current_Position: %d \n",   pTxPDO1->current_position );
    printf("[3] Current_Torque: %d \n",     pTxPDO1->current_torque );
    printf("[4] Current_OP_Mode: 0x%02x \n",pTxPDO1->current_operation_mode );
    printf("[5] Probe_Status: 0x%04x \n",   pTxPDO1->probe_status );
    printf("[6] Curr_Probe_Pos1: %d \n",    pTxPDO1->probe_up_edge_pos1 );
    printf("[7] Curr_Probe_Pos2: %d \n",    pTxPDO1->probe_up_edge_pos1 );
    printf("[8] Digital_Input: 0x%08x \n",  pTxPDO1->digital_input_status );
    printf("---------------------------------------------------------\n\n");

}

void print_RxPDO(RxPDO1t * pRxPDO1){
    printf("RxPDO (servo received)--------------[已转换大小端后显示]-----\n");
    printf("[0] control_word: 0x%04x \n",       pRxPDO1->control_word );
    printf("[1] target_position: %d \n",        pRxPDO1->target_position );
    printf("[2] target_velocity: %d \n",        pRxPDO1->target_velocity );
    printf("[3] target_torque: %d \n",          pRxPDO1->target_torque );
    printf("[4] operation_mode: 0x%02x \n",     pRxPDO1->operation_mode );
    printf("[5] probe_function: 0x%04x \n",     pRxPDO1->probe_function );
    printf("[6] max_velocity: %d \n",           pRxPDO1->max_velocity );
    printf("-----------------------------------------------------------\n\n");
}

char* RxPDO_Items[]={
    "control_word",     "target_position",
    "target_velocity",  "target_torque",
    "operation_mode",   "probe_function",
    "max_velocity",
};

void print_RxPDO_menu(){

    printf("RxPDO (servo input/received)-------\n");
    for(int i=0; i<7; i++){
        printf("[%d] %s\n", i, RxPDO_Items[i]);
    }
    printf("-----------------------------------\n");
    printf("请输入一个序号后回车: ");

}


//将第ord项的值 pRxPDO1, 转换为大端的格式写入!
void apply_new_value_to_big(int ord,RxPDO1t * pRxPDO1)
{
    printf("[%d] %s: ", ord, RxPDO_Items[ord]);

    int64_t value;
    scanf("%i", &value );

    switch(ord)
    {
        case 0: 
            RxPDO1_le.control_word = value & 0x000000000000FFFF;
            break;
        case 1: 
            RxPDO1_le.target_position= value & 0x00000000FFFFFFFF;
            break;
        case 2: 
            //scanf("%i", & pRxPDO1->target_velocity );
            RxPDO1_le.target_velocity = value & 0x00000000FFFFFFFF;
            break;
        case 3: 
            //scanf("%i", & pRxPDO1->target_torque );
            RxPDO1_le.target_torque = value & 0x000000000000FFFF;
            break;
        case 4: 
            //scanf("%i", & pRxPDO1->operation_mode );
            RxPDO1_le.operation_mode = value & 0x00000000000000FF;
            break;
        case 5: 
            //scanf("%i", & pRxPDO1->probe_function );
            RxPDO1_le.probe_function = value & 0x000000000000FFFF;
            break;
        case 6: 
            //scanf("%i", & pRxPDO1->max_velocity );
            RxPDO1_le.max_velocity = value & 0x00000000FFFFFFFF;
            break;
        default:
            printf("\nwrong order num.\n\n");
            return;
    }

    //大小端转化后,再写出
    RxPDO1_copyTo(&RxPDO1_le, pRxPDO1);
    printf("\n\n");
}

// 显示模式: 
//   仅显示 RxPDO区域和 TxPDO区域的内容,
//   用于测试shmViewer输入的值是否正确写入对应位置
void viewer_mode()
{
    static unsigned int count=0;
    while(1){

        //先将大端内容转化为小端
        RxPDO1_copyTo(pRxPDO1, &RxPDO1_le);

        //显示转化后的小端值
        print_RxPDO(&RxPDO1_le);
        print_TxPDO(&TxPDO1_le);
        printf("[%d]请按Enter键,刷新~~~", count++);
        getchar();
    }

};

int main(int argc, char *argv[]){

    /* ----   获取共享内存 用于映射空间 ---*/ 
    IOmap = getIOMapShm();
    if( IOmap == (void*)-1 )
    {
        printf("[ERROR] 获取共享内存失败. 退出程序~\n");
        return -1;
    } 

    //将shm内存指针强制转化为PDO.
    pRxPDO1 = (RxPDO1t*)IOmap; //共19个字节
    pTxPDO1 = (TxPDO1t*)(IOmap + 19);


    if(argc ==1){
    // 进入显示模式: 仅显示 RxPDO区域的内容,用于测试shmViewer输入的值是否正确写入对应位置
        viewer_mode();
        return 0;
    }

    while(1){
        //打印servo输出
        //print_TxPDO(pTxPDO1);

        //sleep(1);

        //打印servo接收值清单
        print_RxPDO_menu();
        
        char ord;
        scanf("%d", &ord);
        //ord= getchar();
        //将值按大端格式写入
        apply_new_value_to_big(ord, pRxPDO1);

        //sleep(1);
    }


    return 0;
}