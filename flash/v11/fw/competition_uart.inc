/* 功能：每个端口最多发送一字节；无参数和返回，独立队列不串包。 */
static void uart_tx_service(void)
{
    if(tail!=head&&!(rd(reply_port,0x18)&0x100U)){wr(reply_port,0x18,(uint8_t)tx[tail]);tail=(tail+1U)%sizeof(tx);}
    if(legacy_tail!=legacy_head&&!(rd(BT_BASE,0x18)&0x100U)){
        wr(BT_BASE,0x18,(uint8_t)legacy_tx[legacy_tail]);legacy_tail=(legacy_tail+1U)%sizeof(legacy_tx);
    }
}
/* 功能：先撤销动力和旧手势再恢复串口；base为两个UART之一；无返回，不自动恢复控制。 */
static void uart_recover(uint32_t base)
{
    pad_stop();halt(T5_LINK);legacy_invalidate();length=0;dropping=true;
    head=tail=0;legacy_head=legacy_tail=0;
    /* 两端硬件FIFO都丢弃，防止另一端排队的旧任务在故障后重新启动。 */
    wr(base,12,4);wr(base==BT_BASE?JDY_BASE:BT_BASE,12,4);
    wr(JDY_BASE,12,8);wr(BT_BASE,12,8);
}
/* 功能：有界处理两个独立端口；无参数和返回，任一端口故障不永久堵塞另一端口。 */
static void uart_rx_service(void)
{
    uint32_t start=rd(IMU_BASE,8);
    for(unsigned i=0;i<32;i++){
        bool received=false;
        for(unsigned port=0;port<2;port++){
            uint32_t base=port?BT_BASE:JDY_BASE,now=rd(JDY_BASE,4);
            if(rd(base,0x1c)){uart_recover(base);continue;}
            uint32_t data=rd(base,0x14);if(data&0x100U)continue;
            received=true;
            if(port){
                if(pad_owner||pad_cal_stage||autonomy_owned){
                    if((uint8_t)data=='~'){pad_stop();legacy_invalidate();}
                    continue;
                }
                /* TX队列满后的软件锁存仅允许新波浪号重建停止边界。 */
                if(legacy.blocked){if((uint8_t)data!='~')continue;legacy_invalidate();legacy_head=legacy_tail=0;}
                jdy_motor_rx(&legacy,(uint8_t)data,now);
            }else{
                if((uint8_t)data=='~')legacy_invalidate();
                receive((uint8_t)data,now);
            }
            if((uint32_t)(rd(IMU_BASE,8)-start)>=1000U)return;
        }
        if(!received)return;
    }
}
