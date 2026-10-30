#include "jdy31_driver.h"
#include <string.h>

enum { REG_ID=0,REG_MS=4,REG_STATE=8,REG_GUARD=12,REG_SEQUENCE=16,
       REG_RX=20,REG_TX=24,REG_ERROR=28 };

/* 功能：整条应答原子入队；ctx为有效驱动，data非空，n为字节数；空间不足返回false，不写半包。 */
static bool enqueue(void *ctx,const char *data,size_t n)
{
    jdy31_driver_t *d=ctx;
    if(n>(d->tail+sizeof(d->tx)-d->head-1U)%sizeof(d->tx))return false;
    for(size_t i=0;i<n;i++){
        d->tx[d->head]=(uint8_t)data[i];d->head=(d->head+1U)%sizeof(d->tx);
    }
    return true;
}
/* 功能：提交保护事件；ctx为有效驱动，event为JDY_G枚举，seq为uint32心跳序号；无返回。 */
static void guard(void *ctx,unsigned event,uint32_t seq)
{
    jdy31_driver_t *d=ctx;
    if(event==JDY_G_KICK)d->bus.write32(d->bus.ctx,REG_SEQUENCE,seq);
    else d->bus.write32(d->bus.ctx,REG_GUARD,event==JDY_G_CLEAR?1U:event==JDY_G_ARM?2U:4U);
}
/* 功能：读取独立RTL保护状态；ctx为有效驱动；返回bit0租约有效、bit1故障锁存。 */
static uint32_t state(void *ctx)
{
    jdy31_driver_t *d=ctx;return d->bus.read32(d->bus.ctx,REG_STATE);
}

/* 功能：验证外设ID并初始化；d/bus及回调不可空；成功true，否则false且驱动不可用。 */
bool jdy31_init(jdy31_driver_t *d,const jdy31_bus_t *bus)
{
    if(!d)return false;
    jdy31_bus_t selected={0};if(bus)selected=*bus;
    memset(d,0,sizeof(*d));
    if(!selected.read32||!selected.write32)return false;
    d->bus=selected;
    if(d->bus.read32(d->bus.ctx,REG_ID)!=UINT32_C(0x45535001))return false;
    const jdy_diag_io_t io={d,enqueue,guard,state};
    jdy_diag_init(&d->protocol,&io,d->bus.read32(d->bus.ctx,REG_MS));
    d->initialized=true;
    if(d->bus.read32(d->bus.ctx,REG_ERROR))jdy_diag_transport_fault(&d->protocol);
    return true;
}

/* 功能：执行有界非阻塞收发；d可空/未初始化，此时无操作；无返回。 */
void jdy31_poll(jdy31_driver_t *d)
{
    if(!d||!d->initialized)return;
    uint32_t now=d->bus.read32(d->bus.ctx,REG_MS);
    if(d->bus.read32(d->bus.ctx,REG_ERROR))jdy_diag_transport_fault(&d->protocol);
    jdy_diag_poll(&d->protocol,now);
    /* 每次服务有明确预算，避免恶意长串口流占满主循环；硬件超时始终独立运行。 */
    for(unsigned budget=0;budget<8;budget++){
        uint32_t byte=d->bus.read32(d->bus.ctx,REG_RX);
        if(byte&0x100U)break;
        jdy_diag_rx(&d->protocol,(uint8_t)byte,now);
    }
    if(d->protocol.blocked){d->tail=d->head;return;}
    if(d->tail!=d->head&&!(d->bus.read32(d->bus.ctx,REG_TX)&0x100U)){
        d->bus.write32(d->bus.ctx,REG_TX,d->tx[d->tail]);
        d->tail=(d->tail+1U)%sizeof(d->tx);
    }
}

/* 功能：获取当前有效意图，优先检查硬件保护；d/out不可空；接口可用返回true，参数/初始化错误false。 */
bool jdy31_get_command(jdy31_driver_t *d,jdy31_command_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));out->fault=true;
    if(!d||!d->initialized)return false;
    /* 即使上层本轮尚未poll，也必须立即看到硬件超时，不能返回缓存的有效方向。 */
    if(d->bus.read32(d->bus.ctx,REG_ERROR))jdy_diag_transport_fault(&d->protocol);
    jdy_diag_poll(&d->protocol,d->bus.read32(d->bus.ctx,REG_MS));
    uint32_t hw=state(d);
    out->fault=d->protocol.blocked||(hw&2U)!=0;
    out->valid=!out->fault&&hw==1&&d->protocol.key>=1&&d->protocol.key<=6&&
               d->protocol.level>=1&&d->protocol.level<=3;
    out->epoch=d->protocol.epoch;out->sequence=d->protocol.sequence;
    if(out->valid){out->key=(jdy31_key_t)d->protocol.key;out->level=(uint8_t)d->protocol.level;}
    return true;
}

/* 功能：本地立即撤销测试租约；d可空/未初始化，此时无操作；无返回。 */
void jdy31_stop(jdy31_driver_t *d)
{
    if(d&&d->initialized)jdy_diag_rx(&d->protocol,'~',d->bus.read32(d->bus.ctx,REG_MS));
}
