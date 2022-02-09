## SOEM 控制汇川 InoSV660N 伺服控制器

 - 基于调试好的 pppv_pdo2.c 代码, 并使用共享内存做数据交换. 
 

 ### 数据操作流程:

   启动汇川 InoSV660N 伺服控制器到 OP模式, RxPDO和TxPDO都映射到共享内存中. 
   
      其他进程写控制内容到共享内存(其中的RxPDO映像区)----------->发送给 InoSV660N
                                                                     |
                                                                   \ |/
                                                                     V
      其他进程读取Servo状态反馈(从共享内存) <------InoSV660N的输出映射到TxPDO镜像区
 
 ### 说明:
 
  - InoSV660N使用第2组PDO映像, 即: 0x1c12--->0x1702, 0x1c13--->0x1b02

  - shmServer.c  和 shmClient.c 为共享内存测试程序,server端和client端.

  - shmViewer 为调试工具,可以显示当前 RxPDO 和 TxPDO 中的内容; 若加上任意参数启动,还可直接修改RxPDO的值,从而控制私服和电机运行

### 使用方法:

```
 # ./sv660n_run  eth0  //启动伺服, 运行后sv660n面板上应该显示83(8代表OP模式, 3代表PV模式)

 # ./shmviewer    //启动共享内存查看工具, 显示当前 RxPDO 和 TxPDO 值. 回车刷新.

 # ./shmviewer w //任何参数即进入修改模式. 修改shm中RxPDO值(如改变速度,改变位置等), 伺服和电机会有相应动作.

 ```

