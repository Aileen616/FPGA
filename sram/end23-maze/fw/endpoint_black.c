#include "endpoint_black.h"
#include <string.h>
#define GROUPS 64U
#define MEMBERS 512U
#define SUPPORTED 192U
#define RUNS 3000U
/* 容量有界，溢出显式报错；所有大数组静态分配，不使用堆或递归。 */
typedef struct { ep_box_t box; uint16_t group; } member_t;
typedef struct { uint32_t orders; uint16_t best; } group_t;
static struct {
    /* 每个RGB565像素只生成一次17阈值位图，三种尺度复用；比灰度缓存增加10320字节。 */
    uint32_t thresholds[17][60][4];
    uint32_t a[60][4],b[60][4],c[60][4];
    uint32_t queue[RUNS];
    member_t members[MEMBERS];group_t groups[GROUPS];
    ep_box_t supported[SUPPORTED];
    uint16_t nm,ng,ns,head,tail,scan,area,sort_index,emit_index,raw_area;
    uint8_t raw_row,raw_ready; /* 原始暗像素密度分片累计，不阻塞IMU服务。 */
    uint8_t phase,kernel_index,order,row,x0,x1,y0,y1;
    ep_result_t result;
} s;
/* 功能：得到已量化RGB565通道的精确8位强度；pixel任意；返回0..255，与Python除法取整一致。 */
static uint8_t maximum(uint16_t pixel)
{
    /* 精确查表代替每像素三次除法；表值仍为向下取整，不改变阈值边界。 */
    static const uint8_t five[32]={0,8,16,24,32,41,49,57,65,74,82,90,98,106,115,123,131,139,148,156,164,172,180,189,197,205,213,222,230,238,246,255};
    static const uint8_t six[64]={0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60,64,68,72,76,80,85,89,93,97,101,105,109,113,117,121,125,129,133,137,141,145,149,153,157,161,165,170,174,178,182,186,190,194,198,202,206,210,214,218,222,226,230,234,238,242,246,250,255};
    unsigned r=(pixel>>11)&31U,b=pixel&31U;uint8_t rb=five[r>b?r:b],g=six[(pixel>>5)&63U];
    return rb>g?rb:g;
}
void ep_reset(void){s.phase=0;s.raw_ready=0;s.result.done=false;s.result.count=0;s.result.error=0;}
void ep_pixel(uint32_t index,uint16_t pixel)
{
    if(s.phase||index>=EP_PIXELS)return;
    unsigned y=index/100,x=index-y*100,w=x>>5;uint32_t mask=UINT32_C(1)<<(x&31U);uint8_t value=maximum(pixel);
    /* 同时设置与清除每个阈值位，允许任意像素装载顺序，也不会把上帧位留给本帧。 */
    for(unsigned t=0;t<17;t++){
        if(value<16+4*t)s.thresholds[t][y][w]|=mask;
        else s.thresholds[t][y][w]&=~mask;
    }
}
bool ep_begin(void)
{
    if(s.phase)return false;
    memset(&s.result,0,sizeof(s.result));s.nm=s.ng=s.ns=0;s.kernel_index=s.order=s.row=0;s.phase=2;return true;
}
const ep_result_t *ep_result(void){return &s.result;}
uint32_t ep_workspace_bytes(void){return sizeof(s);}
/* 功能：终止本帧并发布错误；error为1..5；无返回，清候选以免误用部分结果。 */
static void fail(uint8_t error){s.result.error=error;s.result.count=0;s.result.done=true;s.phase=0;}
/* 功能：计算交并比的分子分母；a/b及n/d均不可空；无返回，合法框使分母非零。 */
static void iou(const ep_box_t *a,const ep_box_t *b,uint32_t *n,uint32_t *d)
{
    unsigned x=a->x>b->x?a->x:b->x,y=a->y>b->y?a->y:b->y;
    unsigned r=a->x+a->w<b->x+b->w?a->x+a->w:b->x+b->w;
    unsigned t=a->y+a->h<b->y+b->h?a->y+a->h:b->y+b->h;
    *n=r>x&&t>y?(r-x)*(t-y):0;*d=a->w*a->h+b->w*b->h-*n;
}
/* 功能：判断组内未舍入填充率排序；a/b不可空；a更优时true，相等保持原先顺序。 */
static bool better(const ep_box_t *a,const ep_box_t *b)
{
    uint32_t aa=a->w*a->h,bb=b->w*b->h,l=a->area*bb,r=b->area*aa;
    return l>r||(l==r&&aa<bb);
}
/* 功能：按最接近偶数规则计算千分填充率；a不可空；返回0..1000，与参考round(fill,3)排序对应。 */
static uint32_t fill1000(const ep_box_t *a)
{
    uint32_t d=a->w*a->h,n=a->area*1000U,q=n/d,r=n%d;
    return q+(2*r>d||(2*r==d&&(q&1U)));
}
/* 功能：判断尺度合并前的稳定排序；a/b不可空；a严格在b前时true。 */
static bool sort_before(const ep_box_t *a,const ep_box_t *b)
{
    uint32_t x=fill1000(a),y=fill1000(b);return x>y||(x==y&&a->w*a->h<b->w*b->h);
}
/* 功能：处理一行可分离开运算；phase为2..5；无返回，腐蚀外界为1、膨胀外界为0，与OpenCV默认边界一致。 */
static void morphology(unsigned phase)
{
    unsigned r=s.kernel_index+1,y=s.row;
    bool dilate=phase>=4,horizontal=phase==2||phase==4;
    uint32_t (*src)[4]=phase==2?s.thresholds[s.order]:phase==4?s.c:s.b;
    uint32_t (*dst)[4]=phase==2||phase==4?s.b:s.c;
    uint32_t outside=dilate?0:UINT32_MAX;
    if(!horizontal){
        /* 同一邻域行同时处理四个字，避免逐列重复边界运算和循环管理。 */
        unsigned low=y>=r?y-r:0,high=y+r<60?y+r:59;
        uint32_t v0=outside,v1=outside,v2=outside,v3=outside;
        if(dilate){for(unsigned row=low;row<=high;row++){v0|=src[row][0];v1|=src[row][1];v2|=src[row][2];v3|=src[row][3];}}
        else {for(unsigned row=low;row<=high;row++){v0&=src[row][0];v1&=src[row][1];v2&=src[row][2];v3&=src[row][3];}}
        dst[y][0]=v0;dst[y][1]=v1;dst[y][2]=v2;dst[y][3]=v3&15U;return;
    }
    for(int w=0;w<4;w++){
        uint32_t value;
        {
            /* 核只有3/5/7三种。常量移位并行处理32像素，避免每个偏移重复分支和跨字查找。 */
            uint32_t v=src[y][w],left=w?src[y][w-1]:outside,right=w<3?src[y][w+1]:outside;
            if(phase==2&&w==3)v|=UINT32_C(0xfffffff0); /* 原图100列之外的腐蚀中性值，不再整图复制补位。 */
            uint32_t a=(v<<1)|(left>>31),b=(v>>1)|(right<<31);
            value=dilate?(v|a|b):(v&a&b);
            if(r>=2){a=(v<<2)|(left>>30);b=(v>>2)|(right<<30);value=dilate?(value|a|b):(value&a&b);}
            if(r>=3){a=(v<<3)|(left>>29);b=(v>>3)|(right<<29);value=dilate?(value|a|b):(value&a&b);}
        }
        dst[y][w]=value;
    }
    /* 100列只占末字低4位；第二腐蚀后把填充位改为0供后续膨胀使用。 */
    dst[y][3]=(dst[y][3]&15U)|(phase==2?UINT32_C(0xfffffff0):0U);
}
/* 功能：查询最终二值图；x为0..99、y为0..59；返回该像素是否尚未消费。 */
/* 功能：查找非零32位字的最低置位；v必须非零；返回0..31。 */
static unsigned first_bit(uint32_t v)
{
    unsigned n=0;
    if(!(v&65535U)){v>>=16;n+=16;}if(!(v&255U)){v>>=8;n+=8;}
    if(!(v&15U)){v>>=4;n+=4;}if(!(v&3U)){v>>=2;n+=2;}
    return n+(!(v&1U));
}
/* 功能：查找非零32位字的最高置位；v必须非零；返回0..31。 */
static unsigned last_bit(uint32_t v)
{
    unsigned n=0;
    if(v>>16){v>>=16;n+=16;}if(v>>8){v>>=8;n+=8;}
    if(v>>4){v>>=4;n+=4;}if(v>>2){v>>=2;n+=2;}
    return n+(v>>1);
}
/* 功能：统计32位字中的暗像素数；v任意；返回0..32，不依赖可变循环次数。 */
static unsigned popcount(uint32_t v)
{
    v-=((v>>1)&UINT32_C(0x55555555));
    v=(v&UINT32_C(0x33333333))+((v>>2)&UINT32_C(0x33333333));
    v=(v+(v>>4))&UINT32_C(0x0f0f0f0f);return (v*UINT32_C(0x01010101))>>24;
}
/* 功能：按32位字查找行内首个未访问像素；y为0..59、low..high为0..99；返回列号或100表示无像素。 */
static unsigned row_first(unsigned y,unsigned low,unsigned high)
{
    for(unsigned w=low>>5;w<=high>>5;w++){
        uint32_t v=s.c[y][w];
        if(w==low>>5)v&=UINT32_MAX<<(low&31U);
        if(w==high>>5)v&=UINT32_MAX>>(31U-(high&31U));
        if(v)return w*32U+first_bit(v);
    }
    return 100;
}
/* 功能：将种子所在水平游程加入队列并原子标记访问；x/y合法且像素为1；返回游程最右列，容量错误时仍可读取error。 */
static unsigned add_run(unsigned x,unsigned y)
{
    unsigned l=x,r=x,w=x>>5;
    uint32_t zeros=~s.c[y][w]&(UINT32_MAX>>(31U-(x&31U)));
    while(!zeros&&w){--w;zeros=~s.c[y][w];}
    l=zeros?w*32U+last_bit(zeros)+1U:0;
    w=x>>5;zeros=~s.c[y][w]&(UINT32_MAX<<(x&31U));
    /* 末字高28位为0，因此右边界一定在第4个字内找到，绝不读取下一行。 */
    while(!zeros&&w<3){++w;zeros=~s.c[y][w];}
    r=zeros?w*32U+first_bit(zeros)-1U:99;
    for(w=l>>5;w<=r>>5;w++){
        uint32_t mask=UINT32_MAX;
        if(w==l>>5)mask&=UINT32_MAX<<(l&31U);
        if(w==r>>5)mask&=UINT32_MAX>>(31U-(r&31U));
        s.c[y][w]&=~mask;
    }
    s.area+=(uint16_t)(r-l+1);
    if(l<s.x0)s.x0=(uint8_t)l;
    if(r>s.x1)s.x1=(uint8_t)r;
    if(y<s.y0)s.y0=(uint8_t)y;
    if(y>s.y1)s.y1=(uint8_t)y;
    if(s.tail>=RUNS){fail(5);return r;}
    s.queue[s.tail++]=l|(r<<7)|(y<<14);return r;
}
/* 功能：将完成连通域加入同尺度阈值组；无参数/返回；使用全成员最大IoU，不仅比较最佳框。 */
static void component(void)
{
    ep_box_t a;memset(&a,0,sizeof(a));
    a.x=s.x0;a.y=s.y0;a.w=s.x1-s.x0+1;a.h=s.y1-s.y0+1;a.area=s.area;
    a.kernel=(uint8_t)(3+2*s.kernel_index);a.threshold=(uint8_t)(16+4*s.order);
    unsigned pixels=a.w*a.h,shorter=a.w<a.h?a.w:a.h;
    if(!a.x||!a.y||a.x+a.w>=100||a.y+a.h>=60||shorter<4U*a.kernel||a.area<64||
       100U*a.w<65U*a.h||2U*a.w>3U*a.h)return;
    if(100U*a.area<78U*pixels){
        /* 形态学用于分离细暗背景，原始阈值图用于核对真实暗密度；保留原通过路径。 */
        if(!s.raw_ready){s.raw_area=0;s.raw_row=s.y0;s.phase=11;return;}
        s.raw_ready=0;
        if(100U*s.raw_area<78U*pixels||5U*a.area<4U*s.raw_area)return;
    }
    uint32_t best_n=0,best_d=1;unsigned group=0;
    for(unsigned i=0;i<s.nm;i++){
        uint32_t n,d;iou(&a,&s.members[i].box,&n,&d);
        if(n*best_d>best_n*d||(n*best_d==best_n*d&&s.members[i].group>=group)){
            best_n=n;best_d=d;group=s.members[i].group;
        }
    }
    bool fresh=10U*best_n<7U*best_d;
    if(fresh){if(s.ng>=GROUPS){fail(1);return;}group=s.ng++;s.groups[group].orders=0;s.groups[group].best=s.nm;}
    if(s.nm>=MEMBERS){fail(2);return;}
    s.members[s.nm].box=a;s.members[s.nm].group=(uint16_t)group;
    if(!fresh&&better(&a,&s.members[s.groups[group].best].box))s.groups[group].best=s.nm;
    s.groups[group].orders|=UINT32_C(1)<<s.order;++s.nm;
    if(s.nm>s.result.members_peak)s.result.members_peak=s.nm;
    if(s.ng>s.result.groups_peak)s.result.groups_peak=s.ng;
}
/* 功能：保存当前尺度满足相邻阈值支持的组；无参数/返回；不同尺度不能凑阈值支持数量。 */
static void finish_kernel(void)
{
    for(unsigned g=0;g<s.ng;g++){
        uint32_t mask=s.groups[g].orders;
        if(!(mask&(mask>>1)))continue;
        if(s.ns>=SUPPORTED){fail(3);return;}
        s.supported[s.ns]=s.members[s.groups[g].best].box;s.supported[s.ns++].support=mask;
    }
    s.ng=s.nm=0;
}
bool ep_step(void)
{
    if(!s.phase)return s.result.done;
    ++s.result.steps;
    if(s.phase==11){
        /* 每次至多8行，额外密度检查也必须让出主循环；不改采集或结果有效期。 */
        unsigned low=s.x0>>5,high=s.x1>>5;
        uint32_t left=UINT32_MAX<<(s.x0&31U),right=UINT32_MAX>>(31U-(s.x1&31U));
        for(unsigned n=0;n<8&&s.raw_row<=s.y1;n++,s.raw_row++)for(unsigned w=low;w<=high;w++){
            uint32_t v=s.thresholds[s.order][s.raw_row][w];
            if(w==low)v&=left;
            if(w==high)v&=right;
            s.raw_area+=(uint16_t)popcount(v);
        }
        if(s.raw_row>s.y1){s.raw_ready=1;s.phase=6;component();}return false;
    }
    if(s.phase>=2&&s.phase<=5){
        /* 八行组成一个有限步骤，减少逐行返回/计时开销；片段仍受2ms预算和IMU让出点约束。 */
        for(unsigned n=0;n<8;n++){
            morphology(s.phase);
            if(s.phase==5)for(unsigned w=0;w<4;w++)s.a[s.row][w]=s.c[s.row][w];
            if(++s.row==60){s.row=0;++s.phase;if(s.phase==6)s.scan=s.head=s.tail=0;break;}
        }return false;
    }
    if(s.phase==9){
        /* 必须逐字完全相等才能复用；分四行比较，保证一次步骤有界。不同帧/尺度不共享缓存。 */
        for(unsigned n=0;n<4&&s.row<60;n++,s.row++)for(unsigned w=0;w<4;w++){
            if(s.thresholds[s.order][s.row][w]!=s.thresholds[s.order-1][s.row][w]){s.row=0;s.phase=2;return false;}
        }
        if(s.row==60){s.row=0;s.phase=10;}return false;
    }
    if(s.phase==10){
        /* 保持原连通域、成员容量、阈值支持和排序流程，只省去重复形态学计算。 */
        for(unsigned n=0;n<8;n++){
            for(unsigned w=0;w<4;w++)s.c[s.row][w]=s.a[s.row][w];
            if(++s.row==60){s.row=0;s.phase=6;s.scan=s.head=s.tail=0;break;}
        }return false;
    }
    if(s.phase==6){
        /* 一次只消费一个游程；以8连通关系检查上下行扩展1列范围。 */
        if(s.head<s.tail){
            uint32_t run=s.queue[s.head++];unsigned l=run&127U,r=(run>>7)&127U,y=run>>14;
            unsigned low=l?l-1:0,high=r<99?r+1:99;
            for(int dy=-1;dy<=1;dy+=2){
                int row=(int)y+dy;if(row<0||row>=60)continue;
                for(unsigned x=low;x<=high;){
                    x=row_first((unsigned)row,x,high);if(x==100)break;
                    x=add_run(x,(unsigned)row)+1;if(s.result.error)return true;
                }
            }
            return false;
        }
        if(s.tail){component();s.head=s.tail=0;if(s.result.error)return true;if(s.phase==11)return false;}
        /* 按32位机器字跳过空白，只在命中时找最低有效位。扫描顺序仍严格按行从左到右。 */
        unsigned budget=16;
        while(s.scan<EP_PIXELS&&budget--){
            unsigned y=s.scan/100,x=s.scan-y*100,w=x>>5;
            uint32_t bits=s.c[y][w]&(UINT32_MAX<<(x&31U));
            if(w==3)bits&=15U;
            if(bits){
                x=w*32+first_bit(bits);
                s.scan=(uint16_t)(y*100+x+1);s.x0=s.x1=(uint8_t)x;s.y0=s.y1=(uint8_t)y;s.area=0;add_run(x,y);return false;
            }
            s.scan=(uint16_t)(y*100+(w==3?100:(w+1)*32));
        }
        if(s.scan<EP_PIXELS)return false;
        if(++s.order<17){s.phase=9;return false;}
        finish_kernel();if(s.result.error)return true;s.order=0;
        if(++s.kernel_index<3){s.phase=2;return false;}
        s.sort_index=1;s.phase=7;return false;
    }
    if(s.phase==7){
        if(s.sort_index<s.ns){
            unsigned i=s.sort_index++;ep_box_t item=s.supported[i];
            while(i&&sort_before(&item,&s.supported[i-1])){s.supported[i]=s.supported[i-1];--i;}s.supported[i]=item;return false;
        }
        s.emit_index=0;s.phase=8;return false;
    }
    if(s.emit_index<s.ns){
        ep_box_t a=s.supported[s.emit_index++];bool duplicate=false;
        for(unsigned i=0;i<s.result.count;i++){uint32_t n,d;iou(&a,&s.result.box[i],&n,&d);if(10*n>=7*d)duplicate=true;}
        if(!duplicate){if(s.result.count>=EP_RESULTS){fail(4);return true;}s.result.box[s.result.count++]=a;}return false;
    }
    s.result.done=true;s.phase=0;return true;
}
