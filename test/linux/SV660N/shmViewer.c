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


void print_TxPDO(TxPDO1t * pTxPDO1){

    printf("TxPDO (servo output)-------------------------------\n");
    printf("[0] Error_Code: 0x%04x \n",     pTxPDO1->error_code );
    printf("[1] Status_Word: 0x%04x \n",    pTxPDO1->status_word );
    printf("[2] Current_Position: %d \n",   pTxPDO1->current_position );
    printf("[3] Current_Torque: %d \n",     pTxPDO1->current_torque );
    printf("[4] Current_OP_Mode: 0x%02x \n",pTxPDO1->current_operation_mode );
    printf("[5] Probe_Status: 0x%04x \n",   pTxPDO1->probe_status );
    printf("[6] Curr_Probe_Pos1: %d \n",    pTxPDO1->probe_up_edge_pos1 );
    printf("[7] Curr_Probe_Pos2: %d \n",    pTxPDO1->probe_up_edge_pos1 );
    printf("[8] Digital_Input: 0x%08x \n",  pTxPDO1->digital_input_status );
    printf("---------------------------------------------------\n\n");

}

void print_RxPDO(RxPDO1t * pRxPDO1){
    printf("RxPDO (servo received)-------------------------------\n");
    printf("[0] control_word: 0x%04x \n",       pRxPDO1->control_word );
    printf("[1] target_position: %d \n",        pRxPDO1->target_position );
    printf("[2] target_velocity: %d \n",        pRxPDO1->target_velocity );
    printf("[3] target_torque: %d \n",          pRxPDO1->target_torque );
    printf("[4] operation_mode: 0x%02x \n",     pRxPDO1->operation_mode );
    printf("[5] probe_function: 0x%04x \n",     pRxPDO1->probe_function );
    printf("[6] max_velocity: %d \n",           pRxPDO1->max_velocity );
    printf("---------------------------------------------------\n\n");
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



void apply_new_value(int ord,RxPDO1t * pRxPDO1)
{
    printf("[%d] %s: ", ord, RxPDO_Items[ord]);

    int64_t value;
    scanf("%i", &value );

    switch(ord)
    {
        case 0: 
            pRxPDO1->control_word = value & 0x000000000000FFFF;
            break;
        case 1: 
            pRxPDO1->target_position= value & 0x00000000FFFFFFFF;
            break;
        case 2: 
            //scanf("%i", & pRxPDO1->target_velocity );
            pRxPDO1->target_velocity = value & 0x00000000FFFFFFFF;
            break;
        case 3: 
            //scanf("%i", & pRxPDO1->target_torque );
            pRxPDO1->target_torque = value & 0x000000000000FFFF;
            break;
        case 4: 
            //scanf("%i", & pRxPDO1->operation_mode );
            pRxPDO1->operation_mode = value & 0x00000000000000FF;
            break;
        case 5: 
            //scanf("%i", & pRxPDO1->probe_function );
            pRxPDO1->probe_function = value & 0x000000000000FFFF;
            break;
        case 6: 
            //scanf("%i", & pRxPDO1->max_velocity );
            pRxPDO1->max_velocity = value & 0x00000000FFFFFFFF;
            break;
        default:
            printf("\nwrong order num.\n\n");
            return;
    }
    printf("\n\n");
}

// 显示模式: 
//   仅显示 RxPDO区域和 TxPDO区域的内容,
//   用于测试shmViewer输入的值是否正确写入对应位置
void viewer_mode()
{
    static unsigned int count=0;
    while(1){
        print_RxPDO(pRxPDO1);
        print_TxPDO(pTxPDO1);
        printf("[%d]请按Enter键,刷新~~~", count++);
        getchar();
    }

};

int main(int argc, char *argv[]){

    /* ----   获取共享内存 用于映射空间 ---*/ 
    IOmap = getIOMapShm();
    if( (int)IOmap == -1 || IOmap == NULL)
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
        apply_new_value(ord, pRxPDO1);

        //sleep(1);
    }


    return 0;
}