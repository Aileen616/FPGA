/* 功能：覆盖方向选择、旁侧无效/近物、失鲜边界及换向；无参数/返回，失败终止。 */
static void direction_policy_tests(void)
{
    const bypass_phase_t phases[3]={B_APPROACH,B_SHIFT_RIGHT,B_RETURN_LEFT};
    const unsigned expected[11]={0,0,1,1,0,0,2,2,3,0,0};
    for(unsigned p=0;p<11;p++)CHECK(bypass_tof_channel((bypass_phase_t)p)==expected[p]);
    for(unsigned d=0;d<3;d++)for(unsigned channel=0;channel<3;channel++)for(unsigned fault=0;fault<6;fault++){
        setup();healthy(510);planner.phase=phases[d];
        mission.raw_invert=d==1?10:d==2?5:12;
        tof_sample_t *t=&joint_tof.sample[channel];
        switch(fault){
        case 0:t->validity=SENSOR_INVALID;break;
        case 1:t->timestamp_us-=200000;break;
        case 2:t->range_status=2;break;
        case 3:t->error=SENSOR_E_IO;break;
        case 4:t->received=false;break;
        default:t->distance_mm=100;break;
        }
        control_tick();
        CHECK(channel==d?(d==0&&fault==5?planner.phase==B_WAIT_FRONT:(planner.phase==B_FAULT&&!mem[1][1])):planner.phase==phases[d]);
    }
    /* 仅前方参与初始放行；换向等待必须检查下一方向，不复用上一方向。 */
    setup();halt(T5_CANCEL);bypass_init(&planner);healthy(1000);
    for(unsigned i=1;i<3;i++){joint_tof.sample[i].validity=SENSOR_INVALID;joint_tof.sample[i].distance_mm=0;}
    CHECK(begin(sequence+1));CHECK(planner.phase==B_APPROACH);
    for(unsigned phase=B_WAIT_FRONT;phase<=B_WAIT_END;phase++){
        setup();healthy(510);planner.phase=(bypass_phase_t)phase;
        unsigned d=bypass_tof_channel(planner.phase);
        for(unsigned i=0;i<3;i++)joint_tof.sample[i].validity=SENSOR_INVALID;
        bypass_input_t v;planning_inputs(&v);CHECK(v.sensors_fresh==(d==3));
        if(d<3){joint_tof.sample[d].validity=SENSOR_VALID;planning_inputs(&v);CHECK(v.sensors_fresh);}
    }
    setup();healthy(510);joint_tof.sample[0].timestamp_us-=199999;
    bypass_input_t v;planning_inputs(&v);CHECK(v.sensors_fresh);
    joint_tof.sample[0].timestamp_us--;planning_inputs(&v);CHECK(!v.sensors_fresh);
    /* 终点只等待轮子停稳，传感器已无效也不能把已完成行程改成传感器故障。 */
    bypass_fsm_t f;memset(&v,0,sizeof(v));bypass_init(&f);
    f.phase=B_WAIT_END;v.lease_valid=v.imu_ready=true;v.now_ms=200;
    bypass_step(&f,&v);CHECK(f.phase==B_DONE);
}
