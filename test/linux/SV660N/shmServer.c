#include <stdio.h>
//#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define SHM_PATH  "/var/run/pnkey"
#define SHM_SIZE 128

int main(){

    key_t key = ftok(SHM_PATH, 0x6666); //file to key，传入的文件SHM_PATH必须存在(权限不要求)，否则失败返回-1
    if(key<0){
        printf("shm key return -1. Please contact the device vendor.");
    }

    // //强制创建1块新的共享内存shm,并设置访问权限位.
    // int shmid = shmget(key, SHM_SIZE, IPC_CREAT|IPC_EXCL|0666);
    // if(shmid <0 ){
    //     printf("Failed to get/create share memory with key= 0x%8x.\n", (int)key );    
        
    //     //先尝试获取已有的shm
    //     shmid = shmget(key, 0, 0);
    //     if(shmid <0 ){
    //         printf("Failed to retrive share momory with key= 0x%4x.\n", (int)key);
    //         return -1;
    //     }
    //     //查看获取的shm的size
    //     struct shmid_ds * buf;
    //     int rst = shmctl(shmid, IPC_STAT, buf);
    //     if(buf->shm_segsz != SHM_SIZE){
    //         printf("The retrived SHM has different size.\n");
    //         return -1;
    //     }
    // }
    
    //获取此key已有的共享内存shm,未找到时创建新的.[可替代上面的手动检索!]
    int shmid = shmget(key, SHM_SIZE, IPC_CREAT);

    //attach到进程空间
    char * addr = shmat(shmid, NULL , 0);
    if(addr <= 0 ){
        printf("failed to attach share memory.\n");
        return -1;
    }

    //使用shm,写入
    sprintf(addr, "%s", "共享内存,您好~\n");

    return 0;
}