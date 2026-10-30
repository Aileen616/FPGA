#include "robot_bsp.h"
#include "mecanum_open_loop_app.h"
#include "jga25_motor.h"
#include <stddef.h>
#ifndef BX72_SELFTEST
#define BX72_SELFTEST 0
#endif
static mecanum_open_loop_app_t app; /* 静态分配，避免在小栈上创建大状态。 */
static volatile uint32_t startup_data=UINT32_C(0x13579bdf); /* 验证ITCM到DTCM的data初始化。 */

#if BX72_SELFTEST
/* 功能：检查仿真条件；ok为true直接返回，false时停机并发布1..65535的code后不返回。 */
static void check(bool ok,uint32_t code)
{
    if(ok) return;
    (void)mecanum_app_stop(&app);bx72_stage(UINT32_C(0xdead0000)|code);
    for(;;) { }
}
/* 功能：仿真专用等待，可选择维护软件心跳/命令；ms<2^31；无返回。禁止用于正式运动流程。 */
static void test_wait(uint32_t ms,bool heartbeat,bool commands)
{
    uint32_t start=bx72_now_ms(),last=start;
    while((uint32_t)(bx72_now_ms()-start)<ms) {
        uint32_t now=bx72_now_ms();
        if(heartbeat && (uint32_t)(now-last)>=10U) {check(bx72_heartbeat(),2U);last=now;}
        if(commands) check(mecanum_app_poll(&app,now),3U);
    }
}
/* 功能：真实RISC-V指令执行的PL协作自测；bus为已初始化且不可空的总线接口；不返回。 */
static void selftest(const robot_bus_t *bus)
{
    robot_open_loop_snapshot_t snap;
    const mecanum_command_t command={500,250,-100};
    check(!bx72_diagnostic_only(),4U);
    check(mecanum_app_clear_fault(&app),5U);
    check(mecanum_app_set_input(&app,&command,bx72_now_ms()),6U);
    check(mecanum_app_start(&app,bx72_now_ms()),7U);
    check(robot_ol_snapshot(&app.drive,&snap),8U);
    check(snap.target.effort[0]==2867 && snap.target.effort[1]==5324 &&
          snap.target.effort[2]==6963 && snap.target.effort[3]==1228,9U);
    bx72_stage(0x100U);
    test_wait(40U,true,true);
    // 仍维护CPU心跳而停止命令刷新，验证运动看门狗独立失效。
    test_wait(120U,true,false);
    check(robot_ol_snapshot(&app.drive,&snap),10U);
    check((snap.status&0xfU)==6U && (snap.status&0x400U)!=0U,11U);
    bx72_stage(0x200U);
    check(mecanum_app_clear_fault(&app),12U);
    check(bus->write32(bus->ctx,ROBOT_OL_REG_CTRL,1U),13U);
    check(robot_ol_snapshot(&app.drive,&snap) && (snap.status&9U)==0U,14U);
    check(mecanum_app_set_input(&app,&command,bx72_now_ms()),15U);
    check(mecanum_app_start(&app,bx72_now_ms()),16U);
    bx72_stage(0x300U); // 测试台在这一阶段拉高急停，再在下一阶段释放。
    test_wait(10U,true,false);
    check(robot_ol_snapshot(&app.drive,&snap) && (snap.status&0x102U)==0x102U &&
          (snap.status&9U)==0U,17U);
    bx72_stage(0x301U);
    test_wait(10U,true,false);
    check(mecanum_app_clear_fault(&app),18U);
    check(mecanum_app_set_input(&app,&command,bx72_now_ms()),19U);
    check(mecanum_app_start(&app,bx72_now_ms()),20U);
    // 非法COMMIT应由BSP返回false，不得更新PL命令序号。
    check(!bus->write32(bus->ctx,ROBOT_OL_REG_COMMIT,0U),21U);
    check(mecanum_app_stop(&app),22U);
    bx72_stage(0x400U);
    test_wait(260U,false,false);
    check((bx72_safety_inputs()&32U)!=0U,23U);
    check(robot_ol_snapshot(&app.drive,&snap) && (snap.status&0x2002U)==0x2002U,24U);
    bx72_stage(UINT32_C(0x600df00d));
    for(;;) { }
}
#else
static char line[64]; /* 只接受有界ASCII命令，溢出整行丢弃。 */
static unsigned line_length;
static bool discard_line;
static char tx[256]; /* 非阻塞UART发送环；诊断输出满时丢弃，不延迟运动服务。 */
static unsigned tx_head,tx_tail;
/* 功能：加入诊断文本；s不可空且零结尾；队列满时停止加入，无返回。 */
static void message(const char *s)
{
    while(*s) {
        unsigned next=(tx_head+1U)%sizeof(tx);
        if(next==tx_tail) break;
        tx[tx_head]=*s++;tx_head=next;
    }
}
/* 功能：输出8位十六进制；v任意uint32；无返回，发送受队列容量限制。 */
static void hex32(uint32_t v)
{
    static const char digits[]="0123456789abcdef";
    char s[9];
    for(unsigned i=0;i<8U;i++) s[i]=digits[(v>>(28U-4U*i))&15U];
    s[8]=0;message(s);
}
/* 功能：解析一个-1000..1000整数；cursor/out不可空；成功更新位置并返回true。 */
static bool number(const char **cursor,int16_t *out)
{
    const char *p=*cursor;int sign=1,value=0;unsigned digits=0;
    while(*p==' ') p++;
    if(*p=='-' || *p=='+') {if(*p=='-') sign=-1;p++;}
    while(*p>='0' && *p<='9') {
        value=value*10+(*p-'0');p++;digits++;
        if(value>1000 || digits>4U) return false;
    }
    if(digits==0U || (*p!=0 && *p!=' ')) return false;
    *out=(int16_t)(sign*value);*cursor=p;return true;
}
/* 功能：比较完整命令；a/b均不可空且零结尾；相同true。 */
static bool equal(const char *a,const char *b)
{
    while(*a && *a==*b) {a++;b++;}
    return *a==*b;
}
/* 功能：执行一行命令；s不可空，最多63字符；错误停止运动，成功不自动清故障。 */
static void command_line(const char *s)
{
    if(equal(s,"status")) {
        robot_open_loop_snapshot_t snap;
        if(robot_ol_snapshot(&app.drive,&snap)) {
            message("STATUS ");hex32(snap.status);message(" SEQ ");hex32(snap.sequence);message("\r\n");
        } else message("BUS ERROR\r\n");
    } else if(equal(s,"stop")) {
        message(mecanum_app_stop(&app)?"STOPPED\r\n":"BUS ERROR\r\n");
    } else if(equal(s,"clear")) {
        message(mecanum_app_clear_fault(&app)?"CLEAR REQUESTED\r\n":"BUS ERROR\r\n");
    } else if(s[0]=='d' && s[1]=='r' && s[2]=='i' && s[3]=='v' && s[4]=='e' && s[5]==' ') {
        const char *p=s+6;mecanum_command_t cmd;
        bool ok=number(&p,&cmd.forward) && number(&p,&cmd.left) && number(&p,&cmd.ccw);
        while(*p==' ') p++;
        if(!ok || *p!=0 || bx72_diagnostic_only()) {
            (void)mecanum_app_stop(&app);message("DRIVE REJECTED\r\n");return;
        }
        ok=mecanum_app_set_input(&app,&cmd,bx72_now_ms());
        if(ok && !app.running) ok=mecanum_app_start(&app,bx72_now_ms());
        message(ok?"ACCEPTED\r\n":"DRIVE FAULT\r\n");
    } else {
        (void)mecanum_app_stop(&app);message("COMMAND ERROR\r\n");
    }
}
/* 功能：有界串口服务，一轮最多读取/发送一字节；无参数、无返回。 */
static void uart_service(void)
{
    uint8_t byte;int rc=bx72_uart_get(&byte);
    if(rc<0) {discard_line=true;(void)mecanum_app_stop(&app);}
    else if(rc>0) {
        if(byte=='\n') {
            if(!discard_line) {line[line_length]=0;command_line(line);}
            line_length=0;discard_line=false;
        } else if(byte!='\r' && !discard_line) {
            if(byte<32U || byte>126U || line_length>=sizeof(line)-1U) {
                discard_line=true;(void)mecanum_app_stop(&app);
            } else line[line_length++]=(char)byte;
        }
    }
    if(tx_tail!=tx_head && bx72_uart_put((uint8_t)tx[tx_tail])) tx_tail=(tx_tail+1U)%sizeof(tx);
}
#endif
/* 功能：裸机入口；无参数；正常不返回，失败停止心跳使PL保持故障禁能。 */
int main(void)
{
    robot_bus_t bus;
    mecanum_config_t config=jga25_open_loop_config();
    if(startup_data!=UINT32_C(0x13579bdf) || !bx72_bsp_init(&bus) ||
       !mecanum_app_init(&app,&bus,&config,20U,100U)) {
        bx72_stage(UINT32_C(0xdead0001));for(;;) { }
    }
    bx72_stage(UINT32_C(0xb0720001));
#if BX72_SELFTEST
    selftest(&bus);
#else
    uint32_t last_heartbeat=bx72_now_ms();
    message(bx72_diagnostic_only()?"BX72 PICO READY DIAGNOSTIC\r\n":"BX72 PICO READY\r\n");
    for(;;) {
        uint32_t now=bx72_now_ms();
        if((uint32_t)(now-last_heartbeat)>=20U) {
            if(!bx72_heartbeat()) {(void)mecanum_app_stop(&app);for(;;) { }}
            last_heartbeat=now;
        }
        (void)mecanum_app_poll(&app,now);
        uart_service();
    }
#endif
}
