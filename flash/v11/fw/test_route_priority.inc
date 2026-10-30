/* 功能：验证换向等待、真实新帧放行和运动失鲜停车；无参数；全部通过返回0，否则断言退出。 */
int main(void)
{
    original_platform_tests();
    const bypass_phase_t phases[3]={B_WAIT_FRONT,B_WAIT_SHIFT,B_WAIT_PASS};
    for(unsigned n=0;n<3;n++){
        setup();healthy(510);tof5_stop(&mission,T5_DISTANCE);
        planner.phase=phases[n];planner.phase_ms=planner.quiet_ms=510;planner.quiet_confirmed=false;
        unsigned d=bypass_tof_channel(planner.phase);uint32_t generation=planner.generation;
        for(unsigned t=520;t<1100;t+=10){
            healthy(t);renewed_ms=t;joint_tof.sample[d].timestamp_us=100000;
            control_tick();CHECK(planner.phase==phases[n]&&planner.generation==generation);
            for(unsigned i=0;i<4;i++)CHECK(mem[1][(0x50+i*4)/4]==0);
        }
        /* 不重置帧时间的调度选择必须指向下一运动方向。 */
        uint32_t old=joint_tof.sample[d].timestamp_us;tof_select_priority(1090000);
        CHECK(joint_tof.poll_index==d&&joint_tof.sample[d].timestamp_us==old);
        healthy(1100);renewed_ms=1100;right_settled_counts=counts(450);control_tick();
        if(n==0)CHECK(planner.phase==B_SHIFT_RIGHT);
        else{
            CHECK(planner.phase==phases[n]&&planner.clear_samples==1);
            for(unsigned t=1200;t<=1300;t+=100){healthy(t);renewed_ms=t;control_tick();}
            CHECK(planner.phase==(n==1?B_PASS:B_RETURN_LEFT));
        }
        CHECK(planner.generation==generation+1&&tof5_active(&mission)&&mission.effort==0);
        healthy(1400);renewed_ms=1400;joint_tof.sample[d].timestamp_us-=200000;guard_reset();control_tick();
        CHECK(planner.phase==B_FAULT&&!mem[1][1]&&mission.reason==T5_SENSOR);
        CHECK(guard_mask==8&&guard_channel==d);
        setup();healthy(510);tof5_stop(&mission,T5_DISTANCE);
        planner.phase=phases[n];planner.phase_ms=planner.quiet_ms=510;planner.quiet_confirmed=false;
        for(unsigned t=520;t<=5510;t+=10){healthy(t);renewed_ms=t;joint_tof.sample[d].validity=SENSOR_INVALID;control_tick();}
        CHECK(planner.phase==B_FAULT&&planner.reason==B_SENSOR&&!mem[1][1]);
    }
    /* 300mm正常停车转入右移等待；前向近物不再阻断右移。 */
    setup();healthy(510);joint_tof.sample[0].distance_mm=300;control_tick();
    CHECK(planner.phase==B_WAIT_FRONT&&mission.reason==T5_OBSTACLE&&mission.effort==0);
    healthy(720);renewed_ms=720;joint_tof.sample[0].validity=SENSOR_INVALID;control_tick();
    CHECK(planner.phase==B_SHIFT_RIGHT&&mission.raw_invert==10);
    printf("HOST_TEST_PASS route_priority checks=%u\n",checks);return 0;
}
