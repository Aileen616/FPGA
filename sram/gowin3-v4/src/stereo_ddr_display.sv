// 功能：双目HDMI显示支路，DDR三缓冲加片内行缓存，沿用E3D5 DVI及启动流程。
// 输入：sys_clk名义21MHz、reference_clk为千兆PHY的125MHz；sys_rst_n低有效，DVP源320×240。
// 输出：status[9]PHY参考、[8:5]失锁计数、[4]扫描心跳、[3]左换帧活动、[2:1]已接收完整帧、[0]锁定。
// 每路160×120三帧存DDR，片内仅有FIFO和双行缓存；沿用E3D5显示时钟。
module stereo_ddr_display(
    input wire sys_clk,sys_rst_n,reference_clk,ready,input wire [1:0] configured,
    input wire mem_clk,mem_rst_n,
    output wire [3:0] req_valid,req_write,input wire [3:0] req_ready,rsp_valid,
    output wire [115:0] req_addr,output wire [1023:0] req_data,input wire [255:0] rsp_data,
    output wire display_lock,output wire [1:0] overflow,underflow,
    input wire c0_pclk,c0_vsync,c0_href,input wire [7:0] c0_data,
    input wire c1_pclk,c1_vsync,c1_href,input wire [7:0] c1_data,
    output wire hdmi_clk_p,hdmi_clk_n,output wire [2:0] hdmi_dat_p,hdmi_dat_n,
    output wire hdmi_oe,output wire [9:0] status);
    wire serial_clk,pixel_clk,raw_lock,video_lock,pll_reset,clock_enable,dvi_rst;
    wire [5:0] icpsel;wire [2:0] lpfres;
    // 125MHz / 5 * 50.25 = 1256.25MHz VCO；/5和/25得到251.25MHz与50.25MHz。
    // 避免片内OSC误差放大到HDMI；此诊断版本依赖PHY保持千兆参考时钟。
    ddr_hdmi_pll_MOD clock_generator(.clkin(reference_clk),.clkout0(serial_clk),.clkout1(),
        .lock(raw_lock),.reset(pll_reset),.clock_enable(clock_enable),.icpsel(icpsel),.lpfres(lpfres),.lpfcap(2'b00));
    PLL_INIT #(.CLK_PERIOD(48),.MULTI_FAC(50)) clock_init(.CLKIN(sys_clk),.I_RST(!sys_rst_n),
        .O_RST(pll_reset),.PLLLOCK(raw_lock),.O_LOCK(video_lock),.ICPSEL(icpsel),.LPFRES(lpfres));
    hdmi_startup video_startup(sys_clk,sys_rst_n,video_lock,dvi_rst,clock_enable);
    // OSER10的并行时钟由同一串行时钟专用分频产生，保持固定5:1关系和分频相位。
    // 分频器在PLL停钟窗口释放复位，随后与DVI一起开始出钟，不使用组合门控时钟。
    CLKDIV #(.DIV_MODE("5")) pixel_divider(.HCLKIN(serial_clk),.RESETN(dvi_rst),.CALIB(1'b0),.CLKOUT(pixel_clk));
    wire timing_rst,display_rst,fb_rst0,fb_rst1;
    wire common_fb_rst=ready;
    assign display_lock=video_lock;
    reset_sync timing_reset(pixel_clk,sys_rst_n && video_lock,timing_rst);
    reset_sync display_reset(pixel_clk,common_fb_rst,display_rst);
    reset_sync fb_reset0(c0_pclk,common_fb_rst,fb_rst0);
    reset_sync fb_reset1(c1_pclk,common_fb_rst,fb_rst1);
    wire [7:0] col;wire [6:0] row,fetch_row;wire vblank,fetch_start,image_active;
    wire [15:0] pixel0,pixel1;wire [1:0] image_valid,published,acknowledge;
    wire [1:0] bank0,bank1,read_bank0,read_bank1;
    reg [1:0] frame_seen,ack_previous;
    reg [7:0] scan_frames;
    always @(posedge pixel_clk or negedge display_rst)
        if(!display_rst)begin frame_seen<=0;ack_previous<=0;end
        else begin
            // 完整帧接收后保持标志，避免串口恰在消隐期间采样pixel_valid而误报未就绪。
            frame_seen<=frame_seen | (acknowledge ^ ack_previous);ack_previous<=acknowledge;
        end
    always @(posedge pixel_clk or negedge timing_rst)
        if(!timing_rst)scan_frames<=0;
        else if(vblank)scan_frames<=scan_frames+1'b1;
    ddr_dvp_writer #(.SNAPSHOT(0),.BASE(29'd131072)) buffer0(
        .pclk(c0_pclk),.cam_rst_n(fb_rst0),.vsync(c0_vsync),.href(c0_href),.data(c0_data),.time_gray(64'b0),
        .request(1'b0),.display_ack(acknowledge[0]),.read_bank(read_bank0),.published(published[0]),
        .published_bank(bank0),.frame_valid(),.timestamp(),.overflow(overflow[0]),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.req_valid(req_valid[0]),.req_write(req_write[0]),
        .req_ready(req_ready[0]),.req_addr(req_addr[0+:29]),.req_data(req_data[0+:256]),.rsp_valid(rsp_valid[0]));
    ddr_dvp_writer #(.SNAPSHOT(0),.BASE(29'd196608)) buffer1(
        .pclk(c1_pclk),.cam_rst_n(fb_rst1),.vsync(c1_vsync),.href(c1_href),.data(c1_data),.time_gray(64'b0),
        .request(1'b0),.display_ack(acknowledge[1]),.read_bank(read_bank1),.published(published[1]),
        .published_bank(bank1),.frame_valid(),.timestamp(),.overflow(overflow[1]),
        .mem_clk(mem_clk),.mem_rst_n(mem_rst_n),.req_valid(req_valid[1]),.req_write(req_write[1]),
        .req_ready(req_ready[1]),.req_addr(req_addr[29+:29]),.req_data(req_data[256+:256]),.rsp_valid(rsp_valid[1]));
    assign req_write[3:2]=0;assign req_data[1023:512]=0;
    ddr_display_reader #(.BASE(29'd131072)) reader0(pixel_clk,display_rst,vblank,fetch_start,fetch_row,row,col,
        published[0],bank0,acknowledge[0],read_bank0,pixel0,image_valid[0],underflow[0],image_active,
        mem_clk,mem_rst_n,req_valid[2],req_ready[2],req_addr[58+:29],rsp_valid[2],rsp_data);
    ddr_display_reader #(.BASE(29'd196608)) reader1(pixel_clk,display_rst,vblank,fetch_start,fetch_row,row,col,
        published[1],bank1,acknowledge[1],read_bank1,pixel1,image_valid[1],underflow[1],image_active,
        mem_clk,mem_rst_n,req_valid[3],req_ready[3],req_addr[87+:29],rsp_valid[3],rsp_data);
    (* ASYNC_REG="TRUE" *) reg [1:0] done_meta,done_sync;
    always @(posedge pixel_clk or negedge timing_rst)
        if(!timing_rst) begin done_meta<=0;done_sync<=0;end
        else begin done_meta<=configured;done_sync<=done_meta;end
    wire hs,vs,de;wire [7:0] red,green,blue;
    ddr_stereo_scanout scanout(pixel_clk,timing_rst,image_valid,done_sync,pixel0,pixel1,col,row,vblank,fetch_start,image_active,fetch_row,hs,vs,de,red,green,blue);
    DVI_TX_Top dvi(.I_rst_n(dvi_rst),.I_serial_clk(serial_clk),.I_rgb_clk(pixel_clk),
        .I_rgb_vs(vs),.I_rgb_hs(hs),.I_rgb_de(de),.I_rgb_r(red),.I_rgb_g(green),.I_rgb_b(blue),
        .O_tmds_clk_p(hdmi_clk_p),.O_tmds_clk_n(hdmi_clk_n),.O_tmds_data_p(hdmi_dat_p),.O_tmds_data_n(hdmi_dat_n));
    assign hdmi_oe=1'b1;
    // 仅同步独立状态位；活动位取源域已寄存Gray位，不异步采样多位二进制计数。
    (* ASYNC_REG="TRUE" *) reg [3:0] status_meta,status_sync;
    always @(posedge sys_clk or negedge sys_rst_n)
        if(!sys_rst_n) begin status_meta<=0;status_sync<=0;end
        else begin status_meta<={scan_frames[4],acknowledge[0],frame_seen};status_sync<=status_meta;end
    wire lock_sync;wire [3:0] lock_losses;
    hdmi_lock_monitor lock_monitor(sys_clk,sys_rst_n,video_lock,lock_sync,lock_losses);
    assign status={1'b1,lock_losses,status_sync,lock_sync};
endmodule
