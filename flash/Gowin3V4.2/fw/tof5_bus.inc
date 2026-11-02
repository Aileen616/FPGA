static uint32_t tof_last_imu_us; /* 字节间IMU服务时间，单位us。 */

/* 功能：执行一个I2C字节；cmd为既有命令位，v为发送字节，out可空；返回0成功/-1失败。 */
static int tof_byte(unsigned cmd,uint8_t v,uint8_t *out)
{
    uint32_t imu_now=rd(IMU_BASE,8);
    /* 按1ms服务IMU/串口/本地保护，避免每个总线字节重复做相同MMIO查询。
     * 只在字节边界让出，单字节仍有原2ms硬超时；不改变10ms控制节拍和300ms硬件保护。 */
    if((uint32_t)(imu_now-tof_last_imu_us)>=1000U){
        tof_last_imu_us=imu_now;imu_service();uart_tx_service();control_tick();
    }
    uint32_t t=rd(TOF_BASE,8),s;
    wr(TOF_BASE,0x14,v);wr(TOF_BASE,0x10,cmd);
    do{
        s=rd(TOF_BASE,4);
        if((s&8U)||(uint32_t)(rd(TOF_BASE,8)-t)>2000U){
            tof_error_status=s;wr(TOF_BASE,0x10,256);return -1;
        }
    }while(!(s&2U));
    if(out)*out=(uint8_t)rd(TOF_BASE,0x18);
    if((cmd&8U)&&(s&4U)){tof_error_status=s;return -1;}
    return 0;
}
/* 功能：一次提交硬件寄存器事务；IMU在事务前后服务，等待期间继续通信和停机保护。
 * 参数：addr为7位地址，reg为16位寄存器，p非空；读n=1..17，写n=1；timeout为2000..20000us。
 * 返回：0完整成功，-1错误/超时；不重放事务，不修改样本时间戳。
 */
static int tof_transfer_burst(uint8_t addr,uint16_t reg,uint8_t *p,size_t n,bool read,uint32_t timeout)
{
    uint32_t start=rd(TOF_BASE,8),status=0;
    tof_error_addr=addr;tof_error_reg=reg;tof_error_stage=0x80;tof_error_status=0;
    if(rd(TOF_BASE,0x44)!=UINT32_C(0x54464231)||(rd(TOF_BASE,4)&193U)!=192U)goto fail;
    /* 总线空闲时先处理已到达的DRDY；I2C连续传输期间不再启动SPI，避免两路时钟并行活动。
     * 正常最长17字节事务约2ms；超时、20ms IMU失鲜和漏中断仍按原规则报错。 */
    imu_service();
    wr(TOF_BASE,0x20,addr|(read?256U:0U)|((uint32_t)n<<16));
    wr(TOF_BASE,0x24,reg);if(!read)wr(TOF_BASE,0x28,p[0]);
    wr(TOF_BASE,0x2c,1);
    for(;;){
        status=rd(TOF_BASE,0x2c);
        if(status&2U){if(status&4U)goto fail;break;}
        uint32_t now=rd(TOF_BASE,8);
        if((uint32_t)(now-start)>=timeout)goto fail;
        if((uint32_t)(now-tof_last_imu_us)>=1000U){
            tof_last_imu_us=now;uart_tx_service();control_tick();
        }
    }
    /* 硬件done保证STOP完成且SCL/SDA已经空闲，此时立即服务在I2C期间到达的DRDY。 */
    imu_service();
    if(read){
        for(size_t i=0;i<n;i+=4){
            uint32_t word=rd(TOF_BASE,0x30+(unsigned)i);
            for(unsigned j=0;j<4&&i+j<n;j++)p[i+j]=(uint8_t)(word>>(j*8));
        }
    }
    uint32_t duration=rd(TOF_BASE,8)-start;
    if(duration>guard_bus_max_us)guard_bus_max_us=duration;
    if(tof_running)vl53l1x_array_health(&joint_tof,rd(TOF_BASE,8));
    tof_error_stage=0;return 0;
fail:
    tof_error_status=status;
    if(!guard_bus_errors++){guard_bus_addr=addr;guard_bus_reg=reg;guard_bus_stage=tof_error_stage;guard_bus_status=status;guard_bus_elapsed=rd(TOF_BASE,8)-start;}
    wr(TOF_BASE,0x2c,256);return -1;
}

/* 功能：执行既有16位寄存器I2C事务；p非空，n=1..32，addr为7位；返回0/-1，失败不重放。 */
static int tof_transfer(uint8_t addr,uint16_t reg,uint8_t *p,size_t n,bool read,uint32_t timeout)
{
    if(!p||!n||n>32||addr<8||addr>0x77||timeout<2000)return -1;
    if((read&&n<=17)||(!read&&n==1))return tof_transfer_burst(addr,reg,p,n,read,timeout);
    uint32_t t=rd(TOF_BASE,8);
    tof_error_addr=addr;tof_error_reg=reg;tof_error_stage=1;tof_error_status=0;
    if((rd(TOF_BASE,4)&192U)!=192U)goto fail;
    if(tof_byte(9,(uint8_t)(addr<<1),0))goto fail;
    tof_error_stage=2;if(tof_byte(8,(uint8_t)(reg>>8),0))goto fail;
    tof_error_stage=3;if(tof_byte(8,(uint8_t)reg,0))goto fail;
    if(read){tof_error_stage=4;if(tof_byte(9,(uint8_t)((addr<<1)|1),0))goto fail;}
    for(size_t i=0;i<n;i++){
        tof_error_stage=5+(uint32_t)i;
        if((uint32_t)(rd(TOF_BASE,8)-t)>=timeout)goto fail;
        if(tof_byte(read?(i+1==n?22:4):(i+1==n?10:8),read?0:p[i],read?&p[i]:0))goto fail;
    }
    uint32_t duration=rd(TOF_BASE,8)-t;if(duration>guard_bus_max_us)guard_bus_max_us=duration;
    /* 用事务结束时刻观察尚未被新帧覆盖的旧样本，避免遗漏总线执行期间跨越200ms的失鲜。 */
    if(tof_running)vl53l1x_array_health(&joint_tof,rd(TOF_BASE,8));
    tof_error_stage=0;return 0;
fail:
    if(!guard_bus_errors++){guard_bus_addr=addr;guard_bus_reg=reg;guard_bus_stage=tof_error_stage;guard_bus_status=tof_error_status;guard_bus_elapsed=rd(TOF_BASE,8)-t;}
    (void)tof_byte(2,0,0);wr(TOF_BASE,0x10,256);return -1;
}
/* 功能：适配读回调；ctx不用可空，p非空，其余范围同tof_transfer；返回0/-1。 */
static int tof_read(void *ctx,uint8_t a,uint16_t r,uint8_t *p,size_t n,uint32_t t)
{(void)ctx;return tof_transfer(a,r,p,n,true,t);}
/* 功能：适配写回调；ctx可空，p指向n=1..32字节；返回0/-1，不修改调用方数组。 */
static int tof_write(void *ctx,uint8_t a,uint16_t r,const uint8_t *p,size_t n,uint32_t t)
{uint8_t copy[32];(void)ctx;if(!p||!n||n>32)return -1;memcpy(copy,p,n);return tof_transfer(a,r,copy,n,false,t);}
/* 功能：操作槽位0..2对应X2/X3/X4；ctx可空，release选择释放0/Z；成功0，否则-1。 */
static int tof_shutdown(void *ctx,uint8_t i,bool release)
{(void)ctx;if(i>3)return -1;uint32_t bit=1U<<(i==3?0:i+1),v=rd(TOF_BASE,12);wr(TOF_BASE,12,release?(v|bit)&15U:v&~bit&15U);return 0;}
/* 功能：关闭三路测距并拉低全部XSHUT；无参数/返回，保留故障记录。 */
static void tof_stop(void)
{tof_running=false;if(joint_tof.bus.xshut)(void)vl53l1x_array_stop(&joint_tof);wr(TOF_BASE,12,0);}
