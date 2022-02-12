#ifndef __SERVO_SHM_H
#define __SERVO_SHM_H

#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define SHM_PATH  "/usr/lib/servo1key"
#define SHM_SIZE 1024

//向系统申请或检索出shm,返回其指针用于servo控制器的IOMap映射.
// pData   (out)返回的地址指针
// 返回值   若成功返回shm地址,失败返回-1.
char* getIOMapShm()
{

    key_t key = ftok(SHM_PATH, 0x6666); //file to key，传入的文件SHM_PATH必须存在(权限不要求)，否则失败返回-1
    if(key<0){
        printf("[ERROR]shm key return -1. Please contact the device vendor.\n");
        return (char*)-1;
    }

    //获取此key已有的共享内存shm,未找到时创建新的(自动初始化为0x00).
    int shmid = shmget(key, SHM_SIZE, IPC_CREAT);

    //attach到进程空间
    void * addr = shmat(shmid, NULL , 0);
    if( addr == (void*)-1 ){
        printf("failed to attach share memory.\n");
        return (char*)-1;
    }

    //返回shm地址
    return (char*)addr;
}

#endif