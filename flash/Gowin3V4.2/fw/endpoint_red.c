#include "endpoint_red.h"
#include <string.h>
#define WORDS ((EP_PIXELS+31U)/32U)
/* 单实例工作区全为静态分配；队列每像素最多进入一次，无堆和递归。 */
static struct {
    uint32_t red[WORDS],loaded[WORDS]; /* 红色掩码及本帧装载位图，单位为像素位。 */
    uint16_t queue[EP_PIXELS]; /* 待遍历像素下标，范围0～5999。 */
    uint16_t loaded_count,head,tail,scan,area;
    uint8_t phase,x0,y0,x1,y1; /* phase=0空闲、1查找新域、2遍历当前域。 */
    ep_result_t result;
} s;

/* 功能：判断无除法的红色条件。参数：pixel为RGB565。返回：true为满足亮度、饱和度及红色色差。 */
bool ep_red_pixel(uint16_t pixel)
{
    uint32_t r=((pixel>>11)&31U)*255U/31U;
    uint32_t g=((pixel>>5)&63U)*255U/63U;
    uint32_t b=(pixel&31U)*255U/31U;
    if(r<35U||r<g||r<b)return false;
    uint32_t low=g<b?g:b,delta=r-low,difference=g>b?g-b:b-g;
    /* 连续HSV等价条件：V≥35、S≥70/255、色相距红色≤36°。
     * 颜色条件采用交叉乘法，不做浮点HSV转换；与OpenCV量化HSV在阈值舍入边界可不同。 */
    return delta*255U>=r*70U&&difference*5U<=delta*3U;
}

/* 功能：复位新帧元数据。参数：无。返回：无；旧红色位由完整新帧逐像素覆盖。 */
void ep_reset(void)
{
    memset(s.loaded,0,sizeof(s.loaded));
    memset(&s.result,0,sizeof(s.result));
    s.loaded_count=0;s.phase=0;
}

/* 功能：装载一像素并记录覆盖。参数：index为0～5999，pixel为RGB565。返回：无，越界及忙时忽略。 */
void ep_pixel(uint32_t index,uint16_t pixel)
{
    if(s.phase||index>=EP_PIXELS)return;
    uint32_t word=index>>5,bit=UINT32_C(1)<<(index&31U);
    if(!(s.loaded[word]&bit)){s.loaded[word]|=bit;++s.loaded_count;}
    if(ep_red_pixel(pixel))s.red[word]|=bit;else s.red[word]&=~bit;
}

/* 功能：发布错误且撤销候选。参数：error为非零错误码。返回：无。 */
static void fail(uint8_t error)
{s.result.count=0;s.result.error=error;s.result.done=true;s.phase=0;}

/* 功能：启动已装载的完整帧。参数：无。返回：true启动，false忙或不完整；不完整时error=6。 */
bool ep_begin(void)
{
    if(s.phase)return false;
    if(s.loaded_count!=EP_PIXELS){fail(6);return false;}
    memset(&s.result,0,sizeof(s.result));
    s.red[WORDS-1]&=UINT32_C(0x0000ffff); /* 6000像素最后一字仅低16位有效。 */
    s.scan=0;s.phase=1;
    /* 掩码在遍历时会被消费，禁止未重新装载就重复开始同一帧。 */
    s.loaded_count=0;memset(s.loaded,0,sizeof(s.loaded));
    return true;
}

/* 功能：入队一个红色像素。参数：index为有效下标。返回：无；入队即清位，防止重复入队。 */
static void enqueue(uint16_t index)
{
    uint32_t bit=UINT32_C(1)<<(index&31U),word=index>>5;
    if(!(s.red[word]&bit))return;
    if(s.tail>=EP_PIXELS){fail(5);return;}
    s.red[word]&=~bit;s.queue[s.tail++]=index;
    if(s.tail>s.result.members_peak)s.result.members_peak=s.tail;
}

/* 功能：结束当前连通域并保留最大合格红域。参数：无。返回：无，不限制形状或是否触边。 */
static void finish_component(void)
{
    if(s.area>=EP_RED_AREA_MIN&&(!s.result.count||s.area>s.result.box[0].area)){
        ep_box_t *box=&s.result.box[0];
        memset(box,0,sizeof(*box));
        box->x=s.x0;box->y=s.y0;box->w=(uint8_t)(s.x1-s.x0+1U);box->h=(uint8_t)(s.y1-s.y0+1U);
        box->area=s.area;box->threshold=70;s.result.count=1;
    }
    s.phase=1;
}

/* 功能：推进有界检测步骤。参数：无。返回：true结果已完成，false仍需推进。 */
bool ep_step(void)
{
    if(!s.phase)return s.result.done;
    ++s.result.steps;
    if(s.phase==1){
        if(s.scan>=WORDS){s.result.done=true;s.phase=0;return true;}
        uint32_t bits=s.red[s.scan];
        if(!bits){++s.scan;return false;}
        unsigned bit=0;while(!(bits&1U)){bits>>=1;++bit;}
        uint16_t index=(uint16_t)(s.scan*32U+bit);
        s.head=s.tail=s.area=0;s.x0=s.x1=(uint8_t)(index%100U);s.y0=s.y1=(uint8_t)(index/100U);
        s.phase=2;enqueue(index);return s.result.done;
    }
    if(s.head==s.tail){finish_component();return false;}
    uint16_t index=s.queue[s.head++];
    int x=index%100U,y=index/100U;
    ++s.area;
    if(x<s.x0)s.x0=(uint8_t)x;
    if(x>s.x1)s.x1=(uint8_t)x;
    if(y<s.y0)s.y0=(uint8_t)y;
    if(y>s.y1)s.y1=(uint8_t)y;
    /* 只访问图像内的8邻域；行边界显式判断，避免左右两行错误连通。 */
    for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){
        int nx=x+dx,ny=y+dy;
        if((dx||dy)&&nx>=0&&nx<100&&ny>=0&&ny<60){
            enqueue((uint16_t)(ny*100+nx));if(s.result.error)return true;
        }
    }
    return false;
}

/* 功能：取得检测结果。参数：无。返回：静态非空只读结果指针。 */
const ep_result_t *ep_result(void){return &s.result;}
/* 功能：取得静态内存用量。参数：无。返回：字节数。 */
uint32_t ep_workspace_bytes(void){return sizeof(s);}
