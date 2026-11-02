/* 仅用于779隔离候选：不在I2C事务内部解析命令，避免测距状态重入。 */

/* 功能：非阻塞发送至多一个字节；无参数；无返回，UART忙时立即退出。 */
static void uart_tx_service(void)
{
    if(tail!=head&&!(rd(JDY_BASE,0x18)&0x100U)){
        wr(JDY_BASE,0x18,(uint8_t)tx[tail]);
        tail=(tail+1U)%sizeof(tx);
    }
}

/* 功能：有界排空接收FIFO；无参数；无返回，至多32字节或1000us后退出。
 * 命令执行仍位于主循环；到达预算后先服务IMU和本地控制，避免串口洪泛抢占采样。
 */
static void uart_rx_service(void)
{
    uint32_t start=rd(IMU_BASE,8); /* 本次服务起点，单位us，允许计数自然回绕。 */
    for(unsigned count=0;count<32;count++){
        if(rd(JDY_BASE,0x1c)){
            halt(T5_LINK);length=0;dropping=true;return;
        }
        uint32_t data=rd(JDY_BASE,0x14);
        if(data&0x100U)return;
        receive((uint8_t)data,rd(JDY_BASE,4));
        if((uint32_t)(rd(IMU_BASE,8)-start)>=1000U)return;
    }
}
