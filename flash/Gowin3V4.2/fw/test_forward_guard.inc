/* 功能：测试前进后终止及故障首帧锁存；无参数；全部通过返回0，否则断言退出。 */
int main(void)
{
    setup();healthy(510);joint_tof.last_poll_us[0]=510000;
    joint_tof.sample[0].timestamp_us=430000;tof_select_priority(510000);CHECK(joint_tof.poll_index==0);
    healthy(510);joint_tof.last_poll_us[0]=490000;tof_select_priority(510000);CHECK(joint_tof.poll_index==0);
    healthy(510);joint_tof.last_poll_us[0]=510000;tof_background_next=1;
    tof_select_priority(510000);CHECK(joint_tof.poll_index==1);
    tof_select_priority(510000);CHECK(joint_tof.poll_index==2);
    CHECK(joint_tof.sample[0].timestamp_us==510000);
    setup();guard_reset();healthy(510);joint_tof.sample[0].validity=SENSOR_INVALID;
    joint_tof.sample[0].error=SENSOR_E_IO;control_tick();
    CHECK(guard_mask==3&&guard_error==2&&guard_phase==B_APPROACH);
    CHECK(mission.reason==T5_SENSOR&&planner.reason==B_SENSOR);
    healthy(520);control_tick();CHECK(guard_mask==3&&guard_error==2);
    CHECK(!mem[1][1]&&!tof5_active(&mission));
    setup();guard_reset();healthy(510);joint_tof.sample[0].timestamp_us-=200000;
    control_tick();CHECK(guard_mask==8&&guard_age==200000&&mission.reason==T5_SENSOR);
    setup();guard_reset();healthy(510);joint_tof.sample[0].range_status=2;
    joint_tof.sample[0].validity=SENSOR_INVALID;control_tick();CHECK(guard_mask==5&&guard_range==2);
    setup();guard_reset();healthy(510);joint_tof.sample[1].validity=SENSOR_INVALID;control_tick();
    CHECK(guard_mask==0&&planner.phase==B_APPROACH);
    setup();guard_reset();healthy(510);joint_tof.sample[0].distance_mm=300;control_tick();
    CHECK(mission.reason==T5_OBSTACLE&&planner.phase==B_WAIT_END);
    healthy(720);renewed_ms=720;control_tick();CHECK(planner.phase==B_DONE&&mission.effort==0&&!mem[1][1]);
    setup();unsigned seen=0;
    for(unsigned t=510;t<5000&&bypass_active(&planner);t+=10){tick(t,true);seen|=1U<<planner.phase;}
    CHECK(planner.phase==B_DONE&&mission.reason==T5_OBSTACLE);
    CHECK(!(seen&((1U<<B_SHIFT_RIGHT)|(1U<<B_PASS)|(1U<<B_RETURN_LEFT))));
    CHECK(!mem[1][1]&&!mem[0][2]);
    head=tail=0;guard_reply();CHECK(head<512);
    printf("HOST_TEST_PASS forward_guard checks=%u\n",checks);return 0;
}
