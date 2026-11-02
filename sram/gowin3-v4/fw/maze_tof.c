#include "maze_tof.h"
#include <string.h>

/* 功能：绝对值；参数x为本模块有界毫米或毫度量；返回非负值。 */
static int32_t mag(int32_t x) { return x < 0 ? -x : x; }
/* 功能：饱和限幅；参数x任意，m为非负上限；返回[-m,m]。 */
static int32_t clip(int32_t x, int32_t m) { return x > m ? m : x < -m ? -m : x; }
/* 功能：计算回绕安全时差；参数为间隔小于2^31毫秒的时钟；返回毫秒差。 */
static uint32_t age(uint32_t now, uint32_t then) { return now - then; }
/* 功能：比较回绕序号；参数a/b为32位序号；返回a是否严格更新。 */
static bool newer(uint32_t a, uint32_t b) { return (int32_t)(a - b) > 0; }
/* 功能：取得反方向；参数d为0至3；返回对应反向。 */
static uint8_t opposite(uint8_t d) { return (uint8_t)((d + 2u) & 3u); }
/* 功能：取得运动左侧；参数d为0至3；返回车体方向。 */
static uint8_t left_of(uint8_t d) { return (uint8_t)((d + 3u) & 3u); }
/* 功能：取得运动右侧；参数d为0至3；返回车体方向。 */
static uint8_t right_of(uint8_t d) { return (uint8_t)((d + 1u) & 3u); }
/* 功能：取得理想外廓尺寸；n不可空、d为0至3；返回沿该方向毫米数。 */
static int32_t body(const mt_t *n, uint8_t d) { return (d & 1u) ? n->cfg.width_mm : n->cfg.length_mm; }
/* 功能：估计允许偏航下的保守投影；n不可空、d为方向；返回毫米上界。 */
static int32_t span(const mt_t *n, uint8_t d) {
    return body(n,d) + (body(n,right_of(d)) * n->cfg.yaw_limit_md + 57295) / 57296;
}
/* 功能：计算距该方向墙的居中目标；n不可空、d为墙方向；返回毫米数。 */
static int32_t centre(const mt_t *n, uint8_t d) { return (n->cfg.corridor_mm - body(n,d)) / 2; }
/* 功能：检查缓存可信年龄；n不可空、d为0至3；返回当前可用性。 */
static bool fresh(const mt_t *n, uint8_t d) {
    return n->sensor[d].accepted && age(n->now_ms,n->sensor[d].stamp_ms) <= n->cfg.fresh_ms;
}
/* 功能：判断当前同宽模型中的近墙；n不可空、d为方向；返回近墙关系。 */
static bool wall(const mt_t *n, uint8_t d) {
    return fresh(n,d) && n->sensor[d].mm <= centre(n,d) + n->cfg.corridor_mm/4;
}
/* 功能：判断远距开口观测；n不可空、d为方向；返回候选结构关系，不独自批准运动。 */
static bool opening(const mt_t *n, uint8_t d) {
    return fresh(n,d) && n->sensor[d].mm >= centre(n,d) + n->cfg.corridor_mm/3;
}
/* 功能：改变状态并更新时间；n不可空、s为状态枚举；返回无。 */
static void state(mt_t *n, uint8_t s) {
    if(n->state != s) { n->state=s; n->entered_ms=n->now_ms; ++n->transitions; }
}
/* 功能：锁存故障并清输出；n不可空、why为原因；返回无，不自动恢复。 */
static void fault(mt_t *n, uint8_t why) {
    n->reason=why; state(n,MT_FAULT); memset(&n->output,0,sizeof(n->output));
}
/* 功能：保存普通等待恢复点；n不可空、why为信息不足原因；返回无。 */
static void wait_for(mt_t *n, uint8_t why) {
    if(!n->wait_active) { n->wait_since=n->now_ms; n->wait_active=1; }
    if(n->state != MT_WAIT) n->wait_resume=n->state;
    n->reason=why; state(n,MT_WAIT);
}
/* 功能：统计低8位异常数；x为窗口位图；返回0至8。 */
static unsigned bad_bits(uint16_t x) {
    unsigned count=0; for(unsigned i=0;i<8;++i) count+=(x>>i)&1u; return count;
}
/* 功能：判断持续质量异常；n不可空、d为通道；返回是否达到时间及错误率条件。 */
static bool bad_quality(const mt_t *n, uint8_t d) {
    const mt_sensor_t *s=&n->sensor[d];
    if(s->quality_count<8 || bad_bits(s->quality_bits)<6) return false;
    unsigned oldest=0,newest_bad=8;
    for(unsigned i=0;i<8;++i) if(s->quality_bits&(1u<<i)) {
        oldest=i; if(newest_bad==8) newest_bad=i;
    }
    return age(s->quality_stamp[newest_bad],s->quality_stamp[oldest])>=n->cfg.confirm_ms;
}
/* 功能：取得当前原始帧净距快照；n/in不可空、d为方向；返回净距，-1表示不可用。 */
static int32_t raw_distance(const mt_t *n,const mt_input_t *in,uint8_t d) {
    const mt_range_input_t *r=&in->range[d];
    if(!r->valid || r->raw_mm>n->cfg.max_range_mm ||
       age(in->now_ms,r->stamp_ms)>n->cfg.fresh_ms) return -1;
    int32_t v=(int32_t)r->raw_mm-n->cfg.inset_mm[d]; return v<0?0:v;
}
/* 功能：处理一路独立新帧与真实距离阶跃；n/in不可空、d为方向；返回无。 */
static void observe_range(mt_t *n,const mt_input_t *in,uint8_t d) {
    mt_sensor_t *s=&n->sensor[d]; const mt_range_input_t *r=&in->range[d];
    if(s->seen && !newer(r->seq,s->seq)) return;
    if((int32_t)(r->stamp_ms-n->now_ms)>0) return;
    if(s->seen && !newer(r->stamp_ms,s->sample_stamp)) { s->seq=r->seq; return; }
    uint32_t raw_dt=age(r->stamp_ms,s->sample_stamp);
    s->seen=1; s->seq=r->seq; s->sample_stamp=r->stamp_ms;
    int32_t v=raw_distance(n,in,d);
    for(unsigned k=7;k>0;--k) s->quality_stamp[k]=s->quality_stamp[k-1];
    s->quality_stamp[0]=r->stamp_ms;
    s->quality_bits=(uint16_t)(((s->quality_bits<<1)|(v<0))&255u);
    if(s->quality_count<8) ++s->quality_count;
    if(v<0) { s->pending_count=0; s->raw_count=0; return; }
    /* 以相邻实测帧判断同一平面的连续推进，不把请求速度当实测轮速。
     * 两段变化须同向、采集时间有效且变化率相容；单帧远/近尖峰和阶跃后的平台不能通过。 */
    int32_t raw_delta=v-s->raw_mm;
    bool axial=(d==n->dir && raw_delta<0 && s->raw_delta<0) ||
               (d==opposite(n->dir) && raw_delta>0 && s->raw_delta>0);
    uint32_t a=(uint32_t)mag(raw_delta)*s->raw_dt;
    uint32_t b=(uint32_t)mag(s->raw_delta)*raw_dt;
    uint32_t noise=(2U*n->cfg.range_error_mm+4U)*(raw_dt+s->raw_dt);
    bool continuous=s->raw_count>=2 && axial && raw_dt && s->raw_dt &&
                    raw_dt<=n->cfg.fresh_ms && s->raw_dt<=n->cfg.fresh_ms &&
                    a<=2U*b+noise && b<=2U*a+noise;
    s->raw_mm=v; s->raw_delta=raw_delta; s->raw_dt=raw_dt;
    if(s->raw_count<2)++s->raw_count;
    /* 超过运动与噪声预算的变化先确认；近障边界也独立确认，防止单帧锁故障。 */
    int32_t allowed=(n->cfg.speed_mm_s*age(r->stamp_ms,s->stamp_ms)+999)/1000+
                    2*n->cfg.range_error_mm+4;
    if(allowed>80) allowed=80;
    bool near_cross=s->accepted && v<n->cfg.margin_mm && s->mm>=n->cfg.margin_mm;
    if(s->accepted && ((mag(v-s->mm)>allowed && !continuous) || near_cross)) {
        if(!s->pending_count || mag(v-s->pending_mm)>60) {
            s->pending_count=1; s->pending_since=r->stamp_ms;
            s->edge_d=raw_distance(n,in,n->dir);
            s->edge_b=raw_distance(n,in,opposite(n->dir));
        } else if(s->pending_count<255) ++s->pending_count;
        s->pending_mm=v;
        if(s->pending_count<n->cfg.confirm_frames ||
           age(r->stamp_ms,s->pending_since)<n->cfg.confirm_ms) return;
        ++s->epoch; s->edge_ready=1;
    } else {
        s->pending_count=0;
        s->edge_d=raw_distance(n,in,n->dir);
        s->edge_b=raw_distance(n,in,opposite(n->dir));
    }
    s->mm=v; s->stamp_ms=r->stamp_ms; s->accepted=1; s->pending_count=0;
}
/* 功能：处理IMU新帧及持续航向异常；n/in不可空；返回无。 */
static void observe_imu(mt_t *n,const mt_input_t *in) {
    if(n->imu_seen && !newer(in->imu_seq,n->imu_seq)) return;
    n->imu_seq=in->imu_seq; n->imu_seen=1;
    if(!in->imu_valid || age(in->now_ms,in->imu_stamp_ms)>n->cfg.fresh_ms) return;
    if(mag(in->yaw_md)>n->cfg.yaw_limit_md) {
        if(!n->yaw_bad_count) n->yaw_bad_since=in->imu_stamp_ms;
        if(n->yaw_bad_count<255) ++n->yaw_bad_count;
        if(n->yaw_bad_count>=n->cfg.confirm_frames &&
           age(in->imu_stamp_ms,n->yaw_bad_since)>=n->cfg.confirm_ms) fault(n,MT_IMU);
        return;
    }
    n->yaw_bad_count=0; n->imu_stamp=in->imu_stamp_ms; n->imu_accepted=1;
    n->yaw_md=in->yaw_md; n->yaw_rate_md_s=in->yaw_rate_md_s;
}

/* 功能：建立待标定默认配置；c可为空，非空时写入已确认安装尺寸；返回无。 */
void mt_defaults(mt_config_t *c) {
    if(!c) return;
    memset(c,0,sizeof(*c));
    c->length_mm=260; c->width_mm=250;
    c->inset_mm[MT_RIGHT]=60; c->inset_mm[MT_LEFT]=60;
    c->installation_confirmed=1;
    /* 以下是候选离线预算；实车动态标定标志默认关闭，通道宽度仍为0。 */
    c->margin_mm=35; c->error_mm=25; c->range_error_mm=4;
    c->max_range_mm=6000; c->tolerance_mm=5;
    c->speed_mm_s=90; c->probe_mm_s=65; c->lateral_mm_s=35; c->brake_mm=10;
    c->fresh_ms=180; c->confirm_ms=70; c->settle_ms=300; c->wait_ms=4000;
    c->yaw_limit_md=3000; c->confirm_frames=3; c->vision_frames=2;
}

/* 功能：初始化静态上下文；n/c不可为空，配置范围见头文件；返回配置是否通过。 */
bool mt_init(mt_t *n,const mt_config_t *c) {
    if(!n || !c) return false;
    memset(n,0,sizeof(*n)); n->cfg=*c; n->current_node=-1; n->return_target=-1;
    n->goal_node=n->target_node=n->travel_target=-1;
    mo_config_t odom;mo_defaults(&odom);(void)mo_init(&n->odometry,&odom);
    n->dir=MT_FRONT;
    if(c->front_goal_only>1 || !c->installation_confirmed || (!c->offline_only && !c->dynamics_verified) ||
       c->length_mm<100 || c->length_mm>1000 || c->width_mm<100 || c->width_mm>1000 ||
       c->corridor_mm>3000 || c->corridor_mm==0 || c->max_range_mm>30000 ||
       c->max_range_mm<c->corridor_mm || c->margin_mm<10 || c->error_mm>200 ||
       c->yaw_limit_md<100 || c->yaw_limit_md>5000 ||
       c->speed_mm_s==0 || c->speed_mm_s>250 || c->probe_mm_s==0 ||
       c->probe_mm_s>c->speed_mm_s || c->lateral_mm_s>c->speed_mm_s ||
       c->fresh_ms<100 || c->fresh_ms>500 || c->confirm_ms<40 ||
       c->confirm_ms>=c->fresh_ms || c->confirm_frames<2 || c->confirm_frames>8 ||
       c->vision_frames<2 || c->settle_ms<100 || c->wait_ms<1000 ||
       c->error_mm < (c->speed_mm_s*c->fresh_ms+999)/1000+c->range_error_mm) {
        fault(n,MT_CONFIG); return false;
    }
    for(uint8_t d=0;d<4;++d) {
        if(c->inset_mm[d]>=body(n,d)/2 ||
           (c->corridor_mm-span(n,d))/2<c->margin_mm+c->error_mm+c->tolerance_mm ||
           centre(n,d)<c->margin_mm+c->brake_mm+(c->speed_mm_s*c->fresh_ms+999)/1000 ||
           c->goal_mm<c->margin_mm+c->error_mm || c->goal_mm>centre(n,MT_FRONT)+c->tolerance_mm) {
            fault(n,MT_CONFIG); return false;
        }
    }
    state(n,MT_IDLE); return true;
}

/* 功能：显式启动检查；n不可为空、dir为0至3、now_ms为当前毫秒；返回请求是否接受。 */
bool mt_start(mt_t *n,uint8_t dir,uint32_t now_ms) {
    if(!n || n->state!=MT_IDLE || dir>=4) return false;
    if(n->cfg.mapping_enabled&&!n->node_count){
        mo_config_t c=n->odometry.cfg;(void)mo_init(&n->odometry,&c);
    }
    if(!n->cfg.mapping_enabled||!n->node_count)n->dir=dir;
    n->now_ms=now_ms; n->last_tick=now_ms; n->started=1;
    state(n,MT_CHECK); return true;
}

/* 功能：锁定人工停机；n可为空；返回无，非空时清除输出。 */
void mt_cancel(mt_t *n) {
    if(!n) return;
    memset(&n->output,0,sizeof(n->output)); n->reason=MT_USER_STOP; state(n,MT_STOPPED);
}

/* 功能：为当前移动设置侧墙检测初始状态；n不可空；返回无。 */
static void arm_sides(mt_t *n) {
    for(uint8_t d=0;d<4;++d) { n->sensor[d].armed=wall(n,d); n->sensor[d].edge_ready=0; }
    n->side_epoch[0]=UINT32_MAX; n->side_epoch[1]=UINT32_MAX;
}
/* 功能：记录下一开口及沿原方向定位依据；n不可空；返回无。 */
static void events(mt_t *n) {
    uint8_t dirs[2]={left_of(n->dir),right_of(n->dir)};
    for(unsigned j=0;j<2;++j) {
        uint8_t d=dirs[j]; mt_sensor_t *s=&n->sensor[d];
        /* 离开已知路口时，原入口是已记录通道，不能再把它的重新显露当成下一个新路口。 */
        if(n->state==MT_CROSS && n->current_node>=0 &&
           d==(n->cfg.mapping_enabled?n->depart_entry:n->nodes[n->current_node].entry)) { s->armed=0; continue; }
        if(wall(n,d)) s->armed=1;
        if(s->armed && opening(n,d)) {
            if(!n->pending_mask) {
                if(s->edge_d>=0 && fresh(n,n->dir)) {
                    n->anchor_dir=n->dir; n->edge_anchor=s->edge_d;
                    n->align_target=s->edge_d-n->cfg.corridor_mm/2;
                } else if(s->edge_b>=0 && fresh(n,opposite(n->dir))) {
                    n->anchor_dir=opposite(n->dir); n->edge_anchor=s->edge_b;
                    n->align_target=s->edge_b+n->cfg.corridor_mm/2;
                } else { wait_for(n,MT_REFERENCE); return; }
                n->anchor_epoch=n->sensor[n->anchor_dir].epoch;
                n->align_from_edge=1;
            }
            n->pending_mask|=(uint8_t)(1u<<d); ++n->pending_events;
            s->armed=0; s->edge_ready=0;
        }
    }
}
/* 功能：检查必要的运动观测；n不可空；返回true可继续，false已处理停止。 */
static bool guard(mt_t *n) {
    uint8_t l=left_of(n->dir),r=right_of(n->dir);
    if(!n->imu_accepted || age(n->now_ms,n->imu_stamp)>n->cfg.fresh_ms) { fault(n,MT_IMU); return false; }
    if(!fresh(n,n->dir) || bad_quality(n,n->dir)) { fault(n,MT_RANGE); return false; }
    if((!fresh(n,l)||bad_quality(n,l)) && (!fresh(n,r)||bad_quality(n,r))) {
        fault(n,MT_REFERENCE); return false;
    }
    if(n->sensor[n->dir].mm<n->cfg.margin_mm) { fault(n,MT_GEOMETRY); return false; }
    return true;
}
/* 功能：生成有界的四向平移意图；n不可空、along为非负毫米每秒；返回无。 */
static void move(mt_t *n,int32_t along) {
    uint8_t dirs[2]={left_of(n->dir),right_of(n->dir)};
    int32_t correction=0;
    bool usable[2]={fresh(n,dirs[0])&&!bad_quality(n,dirs[0]),fresh(n,dirs[1])&&!bad_quality(n,dirs[1])};
    bool close[2]={wall(n,dirs[0])&&usable[0],wall(n,dirs[1])&&usable[1]};
    if(close[0] && close[1]) correction=(n->sensor[dirs[1]].mm-n->sensor[dirs[0]].mm)/2;
    else if(close[0]) correction=centre(n,dirs[0])-n->sensor[dirs[0]].mm;
    else if(close[1]) correction=n->sensor[dirs[1]].mm-centre(n,dirs[1]);
    else {
        /* 两近墙消失时，只锁定确认后的新墙参考；不追逐旧近墙目标。 */
        unsigned j=usable[1]?1:0; uint8_t d=dirs[j];
        if(n->side_epoch[j]!=n->sensor[d].epoch) {
            n->side_epoch[j]=n->sensor[d].epoch; n->side_target[j]=n->sensor[d].mm;
        }
        correction=j ? n->sensor[d].mm-n->side_target[j] : n->side_target[j]-n->sensor[d].mm;
    }
    correction=clip(correction*2,n->cfg.lateral_mm_s);
    /* 纠偏也必须检查对应车身间隙，不能只保护主运动方向。 */
    uint8_t cd=correction>=0?dirs[1]:dirs[0];
    if(correction && (!fresh(n,cd)||n->sensor[cd].mm<n->cfg.margin_mm+n->cfg.error_mm)) correction=0;
    int32_t limit=n->sensor[n->dir].mm-n->cfg.margin_mm-n->cfg.brake_mm-n->cfg.error_mm;
    if(limit<0) limit=0;
    along=clip(along,limit*2);
    static const int8_t fx[4]={1,0,-1,0}, ry[4]={0,1,0,-1}; /* 前和右为正的意图坐标。 */
    n->output.forward_mm_s=(int16_t)(fx[n->dir]*along+fx[right_of(n->dir)]*correction);
    n->output.right_mm_s=(int16_t)(ry[n->dir]*along+ry[right_of(n->dir)]*correction);
    n->output.yaw_rate_md_s=clip(-n->yaw_md*2,6000);
}
/* 功能：按剩余距离生成低速接近请求；n不可空、remaining为毫米；返回非负请求速度。 */
static int32_t approach(const mt_t *n,int32_t remaining) {
    if(remaining<=n->cfg.tolerance_mm) return 0;
    int32_t v=remaining*2; if(v>n->cfg.probe_mm_s) v=n->cfg.probe_mm_s; return v;
}
/* 功能：记录停车基准；n不可空；返回无。 */
static void begin_stop(mt_t *n) {
    n->still_since=n->now_ms; n->goal_checked=0; n->decision_node_ready=0;
    for(unsigned i=0;i<4;++i) n->still_distance[i]=fresh(n,(uint8_t)i)?n->sensor[i].mm:-1;
    state(n,MT_DECIDE);
}
/* 功能：核对停车后观测一致性；n不可空；返回true表示达到有界稳定时间。 */
static bool settled(mt_t *n) {
    bool moved=mag(n->yaw_rate_md_s)>1500;
    for(uint8_t i=0;i<4;++i) if(fresh(n,i)) {
        if(n->still_distance[i]>=0 && mag(n->sensor[i].mm-n->still_distance[i])>n->cfg.range_error_mm+2) moved=true;
    }
    if(moved) {
        n->still_since=n->now_ms;
        for(uint8_t i=0;i<4;++i) n->still_distance[i]=fresh(n,i)?n->sensor[i].mm:-1;
    }
    return age(n->now_ms,n->still_since)>=n->cfg.settle_ms;
}
/* 功能：由停止位置得到四出口集合；n不可空、mask不可空；返回false表示结构未知。 */
static bool exits(const mt_t *n,uint8_t *mask) {
    *mask=(uint8_t)(1u<<opposite(n->dir));
    for(uint8_t d=0;d<4;++d) if(d!=opposite(n->dir)) {
        if(!fresh(n,d)) {
            /* 返回时允许保留已走过但当前无回波的非运动出口拓扑；不制造距离或运动许可。 */
            if(n->returning && d!=n->dir && n->return_target>=0 &&
               n->return_target<n->node_count && (n->nodes[n->return_target].exits&(1u<<d))) {
                *mask|=(uint8_t)(1u<<d); continue;
            }
            return false;
        }
        if(opening(n,d)) *mask|=(uint8_t)(1u<<d);
        else if(!wall(n,d)) return false;
    }
    return true;
}
/* 功能：确认新节点或匹配返回节点；n不可空、mask为出口集合；返回成功标志。 */
/* 功能：登记节点变更；n不可空、id为已有编号；返回无，通知积压不改变运动。 */
static void map_touch(mt_t *n,int16_t id) {
    if(!n->cfg.mapping_enabled||id<0)return;
    ++n->map_revision;n->nodes[id].revision=n->map_revision;
    n->map_events[(n->map_revision-1U)%16U]=id;
}
/* 功能：保存首次节点里程；n不可空、id为已有编号；返回无，不以里程替代ToF到达。 */
static void map_position(mt_t *n,int16_t id) {
    mt_node_t *p=&n->nodes[id];
    p->x_mm=n->odometry.x_mm;p->y_mm=n->odometry.y_mm;p->metric_valid=n->odometry.valid;
    p->edge_mm=(n->dir&1U)?mag(n->odometry.x_mm-n->depart_x_mm):mag(n->odometry.y_mm-n->depart_y_mm);
    map_touch(n,id);
}
/* 功能：确认新节点或匹配已知节点；n不可空、mask为出口集合；返回成功标志。 */
static bool arrive(mt_t *n,uint8_t mask) {
    int16_t known=n->cfg.mapping_enabled&&n->map_phase!=MT_MAP_EXPLORE?n->travel_target:n->return_target;
    if(n->returning||(n->cfg.mapping_enabled&&n->map_phase!=MT_MAP_EXPLORE)) {
        if(known<0 || known>=n->node_count) { fault(n,MT_NODE); return false; }
        mt_node_t *p=&n->nodes[known];
        uint8_t b=opposite(n->dir);
        if(p->exits!=mask || !fresh(n,b) || p->distance_mm[b]<0 ||
           mag(p->distance_mm[b]-n->sensor[b].mm)>2*n->cfg.error_mm+2*n->cfg.tolerance_mm) {
            wait_for(n,MT_NODE); return false;
        }
        /* 父节点身份、出口组合、同墙距离及持续航向共同核对后才返回。 */
        if(n->map_phase==MT_MAP_EXPLORE)p->completed|=(uint8_t)(1u<<b);
        n->current_node=known; n->returning=0; ++n->return_matches;
        n->depart_entry=b;
        if(n->cfg.mapping_enabled){
            p->return_error_mm=mag(n->odometry.x_mm-p->x_mm)+mag(n->odometry.y_mm-p->y_mm);
            map_touch(n,known);
        }
        return true;
    }
    if(n->node_count>=MT_MAX_NODES) { fault(n,MT_CAPACITY); return false; }
    mt_node_t *p=&n->nodes[n->node_count]; memset(p,0,sizeof(*p));
    p->parent=n->current_node; p->entry=opposite(n->dir); p->exits=mask;
    for(uint8_t d=0;d<4;++d) p->distance_mm[d]=fresh(n,d)?n->sensor[d].mm:-1;
    n->current_node=(int16_t)n->node_count++;
    n->depart_entry=opposite(n->dir);
    if(n->cfg.mapping_enabled)map_position(n,n->current_node);
    return true;
}
/* 功能：求树形图下一跳；n不可空、target为已有编号；返回邻居编号，无法路由返回-1。 */
static int16_t map_next(const mt_t *n,int16_t target) {
    int16_t previous=-1;
    for(int16_t p=target;p>=0;p=n->nodes[p].parent){
        if(p==n->current_node)return previous;
        previous=p;
    }
    return n->nodes[n->current_node].parent;
}
/* 功能：从已确认节点选择未探索出口或父方向；n不可空；返回无。 */
static void choose(mt_t *n) {
    mt_node_t *p=&n->nodes[n->current_node];
    uint8_t available=(uint8_t)(p->exits & ~p->tried & ~(1u<<p->entry));
    uint8_t order[3]={right_of(n->dir),n->dir,left_of(n->dir)}, chosen=255;
    if(n->cfg.mapping_enabled&&n->map_phase!=MT_MAP_EXPLORE){
        if(n->current_node==n->target_node){n->map_phase=MT_MAP_READY;state(n,MT_DONE);return;}
        int16_t next=map_next(n,n->target_node);
        if(next<0){fault(n,MT_NODE);return;}
        chosen=next==p->parent?p->entry:opposite(n->nodes[next].entry);
        n->travel_target=next;available=0;
    }else{
    for(unsigned i=0;i<3;++i) if(available&(1u<<order[i])) { chosen=order[i]; break; }
    if(chosen==255 && available) { fault(n,MT_NODE); return; }
    if(chosen==255) {
        if(p->parent<0) {
            if(n->cfg.mapping_enabled){
                n->map_complete=1;
                if(n->goal_node>=0){n->map_phase=MT_MAP_RETURN_GOAL;n->target_node=n->goal_node;choose(n);return;}
                n->map_phase=MT_MAP_NO_GOAL;
            }
            state(n,MT_EXHAUSTED);return;
        }
        chosen=p->entry;
    }
    }
    if(!fresh(n,chosen) || n->sensor[chosen].mm<n->cfg.margin_mm+n->cfg.error_mm+n->cfg.brake_mm) {
        wait_for(n,MT_RANGE); return;
    }
    if(n->cfg.mapping_enabled&&n->map_phase!=MT_MAP_EXPLORE)n->returning=0;
    else if(available){p->tried|=(uint8_t)(1u<<chosen);map_touch(n,n->current_node);}
    else { n->returning=1; n->return_target=p->parent; }
    n->depart_x_mm=n->odometry.x_mm;n->depart_y_mm=n->odometry.y_mm;
    n->dir=chosen; n->cross_dir=chosen; n->cross_anchor=n->sensor[chosen].mm;
    n->cross_epoch=n->sensor[chosen].epoch; n->pending_mask=0; n->align_from_edge=0;
    n->wait_active=0; n->reason=MT_OK; arm_sides(n); state(n,MT_CROSS);
}
/* 功能：发起停车后的新视觉请求；n不可空；返回无。 */
static void vision_begin(mt_t *n) {
    ++n->token; if(!n->token) ++n->token;
    n->vision_since=n->now_ms; n->vision_count=0; n->vision_value=MT_VISION_UNKNOWN;
    state(n,MT_GOAL);
}
/* 功能：接收令牌与采集时刻匹配的视觉结果；n/in不可空；返回无。 */
static void vision_tick(mt_t *n,const mt_input_t *in) {
    n->output.vision_request=1; n->output.vision_token=n->token;
    if(age(n->now_ms,n->vision_since)>n->cfg.wait_ms) {
        /* 建图只因已看到的候选进入确认；确认不成便继续探索，不把未知变成停车故障。 */
        if(n->cfg.mapping_enabled){n->goal_checked=1;state(n,MT_DECIDE);}
        else fault(n,MT_TIMEOUT);
        return;
    }
    if(in->vision_token!=n->token || !newer(in->vision_seq,n->vision_seq) ||
       !in->vision_coverage || in->vision_result==MT_VISION_UNKNOWN || in->vision_result>MT_VISION_POSITIVE ||
       (int32_t)(in->vision_capture_ms-n->vision_since)<0 ||
       age(n->now_ms,in->vision_capture_ms)>n->cfg.wait_ms) return;
    n->vision_seq=in->vision_seq;
    if(n->cfg.mapping_enabled&&in->vision_result==MT_VISION_NEGATIVE){
        /* 当前新图已没有标志，撤销候选即可，不要求先证明普通墙面是终点。 */
        n->goal_checked=1;state(n,MT_DECIDE);return;
    }
    if(in->vision_result!=n->vision_value) {
        n->vision_value=in->vision_result; n->vision_count=0; n->vision_first=in->vision_capture_ms;
    }
    if(n->vision_count<255) ++n->vision_count;
    if(n->vision_count<n->cfg.vision_frames ||
       age(in->vision_capture_ms,n->vision_first)<n->cfg.confirm_ms) return;
    if(n->cfg.mapping_enabled)n->nodes[n->current_node].vision_checked=1;
    if(n->vision_value==MT_VISION_POSITIVE) {
        if(n->dir!=MT_FRONT || !fresh(n,MT_FRONT) ||
           mag(n->sensor[MT_FRONT].mm-n->cfg.goal_mm)>n->cfg.error_mm+n->cfg.tolerance_mm) {
            fault(n,MT_GEOMETRY); return;
        }
        if(n->cfg.mapping_enabled){
            n->goal_node=n->current_node;n->nodes[n->current_node].goal=1;map_touch(n,n->current_node);
            n->goal_checked=1;state(n,MT_DECIDE);
        }else state(n,MT_DONE);
    } else { n->goal_checked=1; state(n,MT_DECIDE); }
}

/* 功能：非阻塞处理采样和状态转换；n/in不可为空，采样约定见头文件；返回内部输出或空指针。 */
const mt_output_t *mt_tick(mt_t *n,const mt_input_t *in) {
    if(!n || !in) return 0;
    n->now_ms=in->now_ms; memset(&n->output,0,sizeof(n->output));
    if(n->cfg.mapping_enabled)(void)mo_update(&n->odometry,&in->encoder);
    if(in->stop) mt_cancel(n);
    if(n->state==MT_IDLE || n->state>=MT_FAULT) goto done;
    if(age(n->now_ms,n->last_tick)>n->cfg.fresh_ms) { fault(n,MT_CLOCK); goto done; }
    n->last_tick=n->now_ms;
    for(uint8_t d=0;d<4;++d) observe_range(n,in,d);
    observe_imu(n,in);
    if(n->state==MT_FAULT) goto done;
    if(n->state==MT_CHECK) {
        bool ready=n->imu_accepted && age(n->now_ms,n->imu_stamp)<=n->cfg.fresh_ms;
        for(uint8_t d=0;d<4;++d) if(d!=opposite(n->dir)) ready=ready&&fresh(n,d);
        if(ready && age(n->now_ms,n->entered_ms)>=n->cfg.settle_ms) {
            if(n->cfg.mapping_enabled){
                if(!n->node_count){
                    mt_node_t *p=&n->nodes[0];p->parent=-1;p->entry=opposite(n->dir);
                    p->exits=p->tried=(uint8_t)(1u<<n->dir);n->node_count=1;n->current_node=0;n->depart_entry=p->entry;
                    for(unsigned d=0;d<4;d++)p->distance_mm[d]=fresh(n,(uint8_t)d)?n->sensor[d].mm:-1;
                    map_position(n,0);n->nodes[0].edge_mm=0;
                    n->depart_x_mm=n->odometry.x_mm;n->depart_y_mm=n->odometry.y_mm;
                }else{
                    n->decision_node_ready=1;n->goal_checked=1;n->align_from_edge=0;
                    n->still_since=n->now_ms;
                    for(unsigned d=0;d<4;d++)n->still_distance[d]=fresh(n,(uint8_t)d)?n->sensor[d].mm:-1;
                    state(n,MT_DECIDE);goto done;
                }
            }
            arm_sides(n);state(n,MT_CRUISE);
        }
        else if(age(n->now_ms,n->entered_ms)>n->cfg.wait_ms) fault(n,MT_RANGE);
        goto done;
    }
    if(!guard(n)) goto done;
    if(n->wait_active && age(n->now_ms,n->wait_since)>n->cfg.wait_ms) { fault(n,MT_TIMEOUT); goto done; }
    if(n->state==MT_WAIT) {
        /* 信息不足等待不自动猜测；同一次任务只在原条件重新可核对时恢复。 */
        if(age(n->now_ms,n->wait_since)>n->cfg.wait_ms) { fault(n,MT_TIMEOUT); goto done; }
        if(age(n->now_ms,n->entered_ms)>=n->cfg.confirm_ms) state(n,n->wait_resume);
        goto done;
    }
    if(n->state==MT_GOAL) { vision_tick(n,in); goto done; }
    if(n->cfg.mapping_enabled&&n->current_node>=0&&(n->state==MT_CRUISE||n->state==MT_CROSS||n->state==MT_ACQUIRE)){
        int16_t target=n->map_phase==MT_MAP_EXPLORE?(n->returning?n->return_target:-1):n->travel_target;
        if(target==0){
            n->anchor_dir=n->dir;n->anchor_epoch=n->sensor[n->dir].epoch;
            n->align_target=n->nodes[0].distance_mm[n->dir];n->align_from_edge=0;
            if(n->align_target<0){wait_for(n,MT_REFERENCE);goto done;}
            state(n,MT_ALIGN);
        }
    }
    if(n->state==MT_CRUISE || n->state==MT_CROSS || n->state==MT_ACQUIRE ||
       n->state==MT_OBSERVE || n->state==MT_ALIGN) events(n);
    if(n->state==MT_WAIT) goto done;
    switch(n->state) {
    case MT_CRUISE:
        if(n->pending_mask || n->sensor[n->dir].mm<=centre(n,n->dir)+120) state(n,MT_OBSERVE);
        else move(n,n->cfg.speed_mm_s);
        break;
    case MT_OBSERVE:
        if(!n->pending_mask) {
            n->anchor_dir=n->dir; n->anchor_epoch=n->sensor[n->dir].epoch;
            n->align_target=n->dir==MT_FRONT?n->cfg.goal_mm:centre(n,n->dir);
            n->align_from_edge=0;
        }
        state(n,MT_ALIGN);
        break;
    case MT_ALIGN: {
        if(!fresh(n,n->anchor_dir)) { fault(n,MT_RANGE); break; }
        if(n->sensor[n->anchor_dir].epoch!=n->anchor_epoch) { wait_for(n,MT_REFERENCE); break; }
        int32_t remaining=n->anchor_dir==n->dir ? n->sensor[n->anchor_dir].mm-n->align_target :
                          n->align_target-n->sensor[n->anchor_dir].mm;
        if(remaining < -(int32_t)n->cfg.error_mm) { fault(n,MT_GEOMETRY); break; }
        if(remaining<=n->cfg.tolerance_mm) begin_stop(n);
        else move(n,approach(n,remaining));
        break;
    }
    case MT_DECIDE: {
        if(age(n->now_ms,n->entered_ms)>n->cfg.wait_ms) { fault(n,MT_TIMEOUT); break; }
        if(!settled(n)) break;
        if(n->align_from_edge) {
            if(!fresh(n,n->anchor_dir) || n->sensor[n->anchor_dir].epoch!=n->anchor_epoch) { wait_for(n,MT_REFERENCE); break; }
            int32_t p=n->anchor_dir==n->dir ? n->edge_anchor-n->sensor[n->anchor_dir].mm :
                      n->sensor[n->anchor_dir].mm-n->edge_anchor;
            if(p-n->cfg.error_mm<span(n,n->dir)/2+n->cfg.margin_mm ||
               p+n->cfg.error_mm+span(n,n->dir)/2>n->cfg.corridor_mm-n->cfg.margin_mm) {
                wait_for(n,MT_GEOMETRY); break;
            }
        }
        uint8_t mask=0; if(!exits(n,&mask)) { wait_for(n,MT_GEOMETRY); break; }
        if(mask==15) { fault(n,MT_UNSUPPORTED); break; }
        if(n->cfg.mapping_enabled&&!n->decision_node_ready){
            if(!arrive(n,mask))break;
            n->decision_node_ready=1;
        }
        if(n->cfg.mapping_enabled&&n->nodes[n->current_node].vision_checked)n->goal_checked=1;
        bool candidate=mask==(1u<<opposite(n->dir));
        if(n->cfg.mapping_enabled&&n->dir==MT_FRONT&&n->sensor[MT_FRONT].mm<=n->cfg.goal_mm+n->cfg.tolerance_mm)candidate=true;
        if(n->cfg.mapping_enabled){
            /* 近墙仅提供观察位置；后台尚未看到有效标志时，仍按原出口选择探索。 */
            candidate=candidate&&in->vision_result==MT_VISION_POSITIVE&&in->vision_coverage&&
                (int32_t)(in->vision_capture_ms-n->now_ms)<=0&&
                age(n->now_ms,in->vision_capture_ms)<=n->cfg.wait_ms;
        }
        if(candidate && !n->goal_checked&&(!n->cfg.mapping_enabled||n->map_phase==MT_MAP_EXPLORE)) {
            if(n->cfg.front_goal_only && n->dir!=MT_FRONT) {
                /* 场地约定排除终点，不生成视觉阴性；仍须经过停稳、出口和父节点核对。 */
                n->goal_checked=1; ++n->excluded_deadends; n->excluded_mask|=(uint8_t)(1U<<n->dir);
            } else { vision_begin(n); break; }
        }
        if(!n->decision_node_ready) {
            if(!arrive(n,mask)) break;
            n->decision_node_ready=1;
        }
        choose(n);
        break;
    }
    case MT_CROSS: {
        if(!fresh(n,n->cross_dir)) { fault(n,MT_RANGE); break; }
        if(n->sensor[n->cross_dir].epoch!=n->cross_epoch) { wait_for(n,MT_REFERENCE); break; }
        int32_t p=n->cross_anchor-n->sensor[n->cross_dir].mm;
        int32_t clear=n->cfg.corridor_mm/2+span(n,n->dir)/2+n->cfg.margin_mm+n->cfg.error_mm;
        /* 连续短直角允许下一个真实开口先出现；交给同一套沿墙对齐和整车过线检查。
         * 继续等待固定跨越距离会在已到前墙居中位置时永远停留于CROSS。 */
        if(n->pending_mask || n->sensor[n->dir].mm<=centre(n,n->dir)+120) state(n,MT_OBSERVE);
        else if(p>=clear) state(n,MT_ACQUIRE);
        else move(n,approach(n,n->sensor[n->dir].mm-centre(n,n->dir)));
        break;
    }
    case MT_ACQUIRE:
        state(n,n->pending_mask?MT_OBSERVE:MT_CRUISE);
        break;
    default: fault(n,MT_NODE); break;
    }
done:
    if(n->state>=MT_WAIT) {
        n->output.forward_mm_s=0; n->output.right_mm_s=0; n->output.yaw_rate_md_s=0;
    }
    n->output.state=n->state; n->output.reason=n->reason;
    if(n->state!=MT_GOAL) n->output.vision_request=0;
    n->output.hardware_permitted=(!n->cfg.offline_only && n->cfg.dynamics_verified &&
                                 n->started && n->state<MT_WAIT && n->state!=MT_IDLE);
    return &n->output;
}

/* 功能：准备已知节点导航；n不可空、target为现有节点；返回是否可路由，不直接启动。 */
bool mt_request_target(mt_t *n,uint16_t target) {
    if(!n||!n->cfg.mapping_enabled||!n->map_complete||target>=n->node_count||
       n->current_node<0||(n->state!=MT_DONE&&n->state!=MT_EXHAUSTED))return false;
    n->target_node=(int16_t)target;n->travel_target=-1;n->map_phase=MT_MAP_GOTO;
    n->reason=MT_OK;n->started=0;n->wait_active=0;state(n,MT_IDLE);return true;
}
/* 功能：读取增量节点通知；n/id/next不可空；after为电脑已收序号；返回0无、1有、2全图。 */
unsigned mt_map_event(const mt_t *n,uint32_t after,int16_t *id,uint32_t *next) {
    if(!n||!id||!next)return 2;
    uint32_t pending=n->map_revision-after;
    if(!pending)return 0;
    if(pending>16)return 2;
    *next=after+1U;*id=n->map_events[after%16U];return 1;
}
