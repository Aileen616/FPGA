/* 依赖：tof5_main.c的MMIO、文本格式化及maze_board_static.inc上下文；只旁路读取编码器。 */
static bool maze_begin(uint32_t seq,bool trial);
static uint32_t maze_map_session;
static int16_t maze_requested_target=-1;
static bool maze_map_trial_mode;
/* 功能：锁存同组四轮快照；now为毫秒；返回无，不写PWM、配置、输出许可或计数清零位。 */
static void maze_encoder_sample(uint32_t now)
{
    if(!maze_shadow.cfg.mapping_enabled)return;
    wr(MOTOR_BASE,8,8);
    maze_input.encoder.seq=rd(MOTOR_BASE,0xd0);maze_input.encoder.stamp_ms=now;
    maze_input.encoder.yaw_md=maze_input.yaw_md;maze_input.encoder.valid=maze_input.imu_valid;
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];uint32_t raw=rd(MOTOR_BASE,0x20+4*bit);
        maze_input.encoder.count[i]=(5U&(1U<<bit))?0U-raw:raw;
    }
}
/* 功能：提交一条地图应答；b为NUL结尾文本，不可空；返回无，队列满时由电脑重试，不改变运动。 */
static void maze_map_reply(const char *b)
{(void)perf_enqueue(0,b,strlen(b));}
/* 功能：发送节点快照；id为已有编号；返回无，记录携带会话和节点版本。 */
static void maze_map_node(unsigned id)
{
    if(id>=maze_shadow.node_count){maze_map_reply("MN ERR=1\r\n");return;}
    const mt_node_t *n=&maze_shadow.nodes[id];char b[384],*p=text(b,"MN");
    p=field(p," S=",maze_map_session);p=field(p," R=",n->revision);p=field(p," ID=",id);
    p=text(p," PA=");p=signed_number(p,n->parent);p=field(p," EN=",n->entry);p=field(p," EX=",n->exits);
    p=field(p," TR=",n->tried);p=field(p," CO=",n->completed);
    p=text(p," X=");p=signed_number(p,n->x_mm);p=text(p," Y=");p=signed_number(p,n->y_mm);
    p=field(p," L=",(uint32_t)n->edge_mm);p=field(p," ER=",(uint32_t)n->return_error_mm);
    p=field(p," V=",n->metric_valid);p=field(p," G=",n->goal);p=text(p,"\r\n");*p=0;maze_map_reply(b);
}
/* 功能：处理地图查询和目标节点请求；s不可空；返回是否属于map命令，不提供任意控制通道。 */
static bool maze_map_command(const char *s)
{
    if(strncmp(s,"map ",4))return false;
    char b[256],*p;uint32_t value;
    if(!strcmp(s,"map pose")){
        p=field(text(b,"MP")," S=",maze_map_session);p=field(p," R=",maze_shadow.map_revision);
        p=field(p," N=",maze_shadow.node_count);p=text(p," I=");p=signed_number(p,maze_shadow.current_node);
        p=field(p," P=",maze_shadow.map_phase);p=field(p," C=",maze_shadow.map_complete);
        p=text(p," G=");p=signed_number(p,maze_shadow.goal_node);p=text(p," T=");p=signed_number(p,maze_shadow.target_node);
        p=text(p," X=");p=signed_number(p,maze_shadow.odometry.x_mm);p=text(p," Y=");p=signed_number(p,maze_shadow.odometry.y_mm);
        p=field(p," V=",maze_shadow.odometry.valid);p=field(p," MS=",maze_shadow.odometry.stamp_ms);
        p=text(p,"\r\n");*p=0;maze_map_reply(b);
    }else if(!strncmp(s,"map node ",9)&&number(s+9,&value))maze_map_node(value);
    else if(!strncmp(s,"map event ",10)&&number(s+10,&value)){
        int16_t id=-1;uint32_t next=value;unsigned result=mt_map_event(&maze_shadow,value,&id,&next);
        p=field(text(b,"ME")," S=",maze_map_session);p=field(p," K=",result);p=field(p," R=",next);
        p=text(p," ID=");p=signed_number(p,id);p=text(p,"\r\n");*p=0;maze_map_reply(b);
    }else if(!strncmp(s,"map goto ",9)&&number(s+9,&value)){
        bool ok=false;
        if(value<maze_shadow.node_count){maze_requested_target=(int16_t)value;ok=maze_begin(sequence+1U,maze_map_trial_mode);maze_requested_target=-1;}
        maze_map_reply(ok?"MG OK=1\r\n":"MG OK=0\r\n");
    }else maze_map_reply("MP ERR=1\r\n");
    return true;
}
