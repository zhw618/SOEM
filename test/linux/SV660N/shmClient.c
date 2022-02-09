#include <stdio.h>
#include <string.h>
//#include <unistd.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>


#define SHM_PATH  "/var/run/pnkey"
#define SHM_SIZE  128

int main(){

    key_t key = ftok(SHM_PATH, 0x6666);
    if(key<0){
        printf("shm key return -1. Please contact the device vendor.");
    }

    char buf[128];

    //int shmid = shmget(key, SHM_SIZE, IPC_CREAT);
    int shmid = shmget(key, SHM_SIZE, 0);
    if(shmid <0){
        printf("Failed to retrive share memory.\n");
        return -1;
    }

    char * addr = shmat(shmid, NULL, SHM_RDONLY);  //只读挂载
    if(addr<=0){
        printf("Failed to map share memory.\n");
        return -1;
    }

    //使用shm,打印字符串.
    strcpy(buf, addr);
    printf("%s", buf);
    printf("%s", addr);

    return 0;
}