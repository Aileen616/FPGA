/* 功能：只读一轮冻结记录；channel为0..3；无返回，无新快照、清零或运动续期。 */
static void encoder_fault_reply(unsigned channel)
{
    char b[512],*p=text(b,"EFT V=1");const encoder_fault_trace_t *r=&encoder_trace;
    p=field(p," VALID=",r->valid);p=field(p," ID=",r->id);p=field(p," CH=",channel);
    if(r->valid){
        p=field(p," MASK=",r->mask);p=field(p," WHEELS=",r->wheels);p=field(p," PHASE=",r->phase);
        p=field(p," MS=",r->ms);p=field(p," ELAPSED=",r->elapsed);p=field(p," RINV=",r->invert);
        p=field(p," RAW=",r->position[channel]);p=field(p," BASE=",r->baseline[channel]);
        p=text(p," DELTA=");p=signed_number(p,r->delta[channel]);
        p=field(p," ILL=",r->illegal[channel]);p=field(p," ILL0=",r->illegal_base[channel]);
        p=field(p," IDIFF=",r->illegal[channel]-r->illegal_base[channel]);
        p=field(p," IDLE=",r->idle_ms[channel]);
        p=text(p," REQUEST=");p=signed_number(p,r->requested);
        p=text(p," ACTUAL=");p=signed_number(p,r->actual[channel]);
        p=field(p," RETURNCOUNT=",r->returned);
    }
    p=text(p,"\r\n");*p=0;reply(b);
}
