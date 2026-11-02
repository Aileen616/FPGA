#include "sensor_joint_array.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks,ops,fail_at;
static uint32_t now;
static uint8_t regs[4][0x112],addr[4];
static bool on[4],ready[4],range[4],badid;
static bool stuck_ready; /* 模拟清中断无效、GPIO卡在就绪，不得重复发布旧帧。 */
static bool fixed_stream; /* 模拟定时模式STREAM不递增，但就绪事件持续产生。 */
static bool continuous; /* 连续模型只在对应专项用例启用。 */
static uint32_t next_frame[4]; /* 各模块下次测量到期时间，单位us。 */
static vl53l1x_array_t a;
#define CHECK(x) do{checks++;if(!(x)){printf("FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
/* 功能：查找唯一响应模块；ad为7位地址；返回索引或-1。 */
static int find(uint8_t ad){int k=-1;for(int i=0;i<4;i++)if(on[i]&&addr[i]==ad){CHECK(k==-1);k=i;}return k;}
/* 功能：模型XSHUT释放和复位；参数遵循四路接口；返回0/-1。 */
static int shut(void *c,uint8_t i,bool r)
{(void)c;CHECK(i<4);if(++ops==fail_at)return -1;on[i]=r;addr[i]=0x29;range[i]=ready[i]=false;memset(regs[i],0,sizeof(regs[i]));regs[i][0x10f]=0xea;regs[i][0x110]=0xcc;regs[i][0xde]=1;regs[i][0xdf]=0x23;return 0;}
/* 功能：模型读寄存器；参数遵循sensor_bus；返回0/-1，支持总线故障注入。 */
static int read_i(void *c,uint8_t ad,uint16_t reg,uint8_t *p,size_t n,uint32_t t)
{
 (void)c;CHECK(t==20000&&p&&n&&reg+n<=0x112);if(++ops==fail_at)return -1;int i=find(ad);if(i<0)return -1;
 if(continuous&&range[i]&&!ready[i]&&now>=next_frame[i]){
  ready[i]=true;regs[i][0x89]=9;if(fixed_stream)regs[i][0x8b]=1;else regs[i][0x8b]++;
  regs[i][0x96]=1;regs[i][0x97]=(uint8_t)(now/100000);next_frame[i]=now+100000;
 }
 regs[i][0xe5]=1;regs[i][0x10f]=badid?0:0xea;
 regs[i][0x31]=((regs[i][0x30]&16)?!ready[i]:ready[i]);memcpy(p,regs[i]+reg,n);return 0;
}
/* 功能：模型写寄存器，核对改址后不冲突；参数遵循sensor_bus；返回0/-1。 */
static int write_i(void *c,uint8_t ad,uint16_t reg,const uint8_t *p,size_t n,uint32_t t)
{
 (void)c;CHECK(t==20000&&p&&n&&reg+n<=0x112);if(++ops==fail_at)return -1;int i=find(ad);if(i<0)return -1;
 memcpy(regs[i]+reg,p,n);if(reg==1){addr[i]=p[0];CHECK(find(addr[i])==i);}
 if(reg==0x87){range[i]=p[0]==0x40;ready[i]=range[i];next_frame[i]=now+100000;}
 if(reg==0x86&&!stuck_ready)ready[i]=false;return 0;
}
static sensor_bus_t bus={.i2c_read=read_i,.i2c_write=write_i,.xshut=shut};
static tof_config_t cfg={.count=4,.address7={0x30,0x31,0x32,0x33},.distance_mode=2,.budget_ms=50,.period_ms=100,.fresh_us=500000,.io_mode=TOF_IO_AVDD};
/* 功能：复位模型；f为注入故障操作号，0为关闭；无返回。 */
static void reset(unsigned f){memset(on,0,sizeof(on));memset(&a,0,sizeof(a));now=ops=0;fail_at=f;badid=false;continuous=false;fixed_stream=false;stuck_ready=false;memset(next_frame,0,sizeof(next_frame));}
/* 功能：推进完整四路初始化；无参数；返回最终错误或OK。 */
static int init(void){int rc=vl53l1x_array_start(&a,&bus,&cfg,now);for(unsigned k=0;k<2500&&rc==SENSOR_PENDING;k++){now+=1000;rc=vl53l1x_array_step(&a,now);}return rc;}
/* 功能：注入一路新帧；i=0..3、seq/status是字节、mm是距离；无返回。 */
static void frame(unsigned i,uint8_t seq,uint8_t status,uint16_t mm)
{ready[i]=true;regs[i][0x89]=status;regs[i][0x8b]=seq;regs[i][0x96]=(uint8_t)(mm>>8);regs[i][0x97]=(uint8_t)mm;}
/* 功能：校验四路改址、质量/过期与每一步故障；无参数；成功0。 */
int main(void)
{
 reset(0);CHECK(init()==0);unsigned count=ops;CHECK(vl53l1x_array_ready(&a));
 for(unsigned i=0;i<4;i++){CHECK(addr[i]==0x30+i&&range[i]);frame(i,1,9,400+i*100);a.poll_index=i;CHECK(vl53l1x_array_step(&a,now)==0);CHECK(a.sample[i].validity==SENSOR_VALID&&a.sample[i].distance_mm==400+i*100);}
 uint32_t stamp=a.sample[3].timestamp_us;a.poll_index=3;frame(3,1,9,1000);now+=1000;CHECK(vl53l1x_array_step(&a,now)==SENSOR_OK);CHECK(a.sample[3].timestamp_us>stamp);
 for(unsigned raw=0;raw<32;raw++){ready[3]=false;a.poll_index=3;CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);a.poll_index=3;frame(3,2+raw,raw,900);CHECK(vl53l1x_array_step(&a,now)==0);CHECK((a.sample[3].validity==SENSOR_VALID)==(raw==9));}
 ready[3]=false;a.poll_index=3;CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);a.poll_index=3;frame(3,99,9,0);CHECK(vl53l1x_array_step(&a,now)==0&&a.sample[3].validity==SENSOR_INVALID);
 ready[3]=false;a.poll_index=3;CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);a.poll_index=3;frame(3,100,9,900);CHECK(vl53l1x_array_step(&a,now)==0);vl53l1x_array_health(&a,now+500000);CHECK(a.sample[3].validity==SENSOR_STALE);
 for(unsigned f=1;f<=count;f++){reset(f);CHECK(init()==SENSOR_E_IO);CHECK(!vl53l1x_array_ready(&a));for(unsigned i=0;i<4;i++)CHECK(a.sample[i].validity!=SENSOR_VALID);}
 reset(0);badid=true;CHECK(init()==SENSOR_E_ID);
 reset(0);cfg.address7[3]=0x30;CHECK(init()==SENSOR_E_ARG);cfg.address7[3]=0x33;
 /* 连续30秒跨越8位帧计数回绕，按真实周期产帧，只有清中断后才允许下一帧。 */
 reset(0);cfg.count=3;cfg.address7[0]=0x31;cfg.address7[1]=0x32;cfg.address7[2]=0x33;
 CHECK(init()==0);continuous=true;
 for(unsigned k=0;k<30000;k++){now+=1000;CHECK(vl53l1x_array_step(&a,now)>=0);}
 for(unsigned i=0;i<3;i++){
  CHECK(a.sample[i].sequence>290&&a.sample[i].sequence<310);
  CHECK(a.sample[i].validity==SENSOR_VALID&&now-a.sample[i].timestamp_us<110000);
  CHECK(regs[i][0x6c]==0&&regs[i][0x6d]==0&&regs[i][0x6e]==0x7a&&regs[i][0x6f]==0x32);
 }
 /* STREAM固定1仍须接受真实就绪边沿，连续30秒；不是放宽500ms新鲜度。 */
 fixed_stream=true;
 uint32_t before[3];for(unsigned i=0;i<3;i++)before[i]=a.sample[i].sequence;
 for(unsigned k=0;k<30000;k++){now+=1000;CHECK(vl53l1x_array_step(&a,now)>=0);}
 for(unsigned i=0;i<3;i++){CHECK(a.sample[i].sequence-before[i]>290);CHECK(a.sample[i].stream_count==1);CHECK(a.sample[i].validity==SENSOR_VALID);}
 /* 相邻轮询跨过一个测量周期，没有另一次轮询看到低电平；清除后立即确认仍应持续发布。 */
 uint32_t slow_before[3];for(unsigned i=0;i<3;i++)slow_before[i]=a.sample[i].sequence;
 for(unsigned k=0;k<300;k++){now+=34000;CHECK(vl53l1x_array_step(&a,now)>=0);}
 for(unsigned i=0;i<3;i++){CHECK(a.sample[i].sequence-slow_before[i]>90);CHECK(a.clear_confirmed[i]>0);}
 /* 卡高模型必须让清中断写入无效，不能在模型已清除后人为制造新边沿再称为旧帧。 */
 continuous=false;a.poll_index=0;ready[0]=false;CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);
 stuck_ready=true;frame(0,1,9,900);a.poll_index=0;CHECK(vl53l1x_array_step(&a,now)==SENSOR_OK);
 uint32_t held_sequence=a.sample[0].sequence,held_stamp=a.sample[0].timestamp_us;
 for(unsigned k=0;k<600;k++){now+=1000;ready[0]=true;a.poll_index=0;CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);}
 CHECK(a.sample[0].sequence==held_sequence&&a.sample[0].timestamp_us==held_stamp&&a.sample[0].validity==SENSOR_STALE);
 /* 一次清除未完成后恢复：不能续期旧帧，下一次真实就绪才能发布。 */
 stuck_ready=false;a.poll_index=0;now+=1000;
 CHECK(vl53l1x_array_step(&a,now)==SENSOR_PENDING);
 CHECK(!a.wait_ready_low[0]&&a.sample[0].sequence==held_sequence&&a.sample[0].timestamp_us==held_stamp);
 frame(0,1,9,1234);a.poll_index=0;now+=1000;
 CHECK(vl53l1x_array_step(&a,now)==SENSOR_OK);
 CHECK(a.sample[0].sequence==held_sequence+1&&a.sample[0].timestamp_us==now&&a.sample[0].distance_mm==1234);
 CHECK(vl53l1x_array_stop(&a)==0);for(unsigned i=0;i<3;i++)CHECK(!on[i]);
 printf("HOST_TEST_PASS sensor_joint_array checks=%u\n",checks);return 0;
}
