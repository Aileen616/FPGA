// 功能：双目HDMI显示支路，沿用3536三缓冲、DVI IP及受控启动流程。
// 输入：sys_clk名义21MHz、reference_clk为千兆PHY的125MHz；sys_rst_n低有效，DVP源320×240。
// 输出：status为[9]PHY参考标记、[8:5]失锁饱和计数、[4:3]换帧活动、[2:1]有效帧、[0]同步锁定。
// 显示缓存每路160×120，三缓冲只在消隐交换；网络仍独立发送320×240原图。
module stereo_local_display(
    input wire sys_clk,sys_rst_n,reference_clk,input wire [1:0] configured,
    input wire c0_pclk,c0_vsync,c0_href,input wire [7:0] c0_data,
    input wire c1_pclk,c1_vsync,c1_href,input wire [7:0] c1_data,
    output wire hdmi_clk_p,hdmi_clk_n,output wire [2:0] hdmi_dat_p,hdmi_dat_n,
    output wire hdmi_oe,output wire [9:0] status);
    wire serial_clk,pixel_clk,raw_lock,video_lock,pll_reset,clock_enable,dvi_rst;
    wire [5:0] icpsel;wire [2:0] lpfres;
    // 125MHz / 5 * 50.25 = 1256.25MHz VCO；/5和/25得到251.25MHz与50.25MHz。
    // 避免片内OSC误差放大到HDMI；此诊断版本依赖PHY保持千兆参考时钟。
    stereo_600p_pll_MOD clock_generator(.clkin(reference_clk),.clkout0(serial_clk),.clkout1(pixel_clk),
        .lock(raw_lock),.reset(pll_reset),.clock_enable(clock_enable),.icpsel(icpsel),.lpfres(lpfres),.lpfcap(2'b00));
    PLL_INIT #(.CLK_PERIOD(48),.MULTI_FAC(50)) clock_init(.CLKIN(sys_clk),.I_RST(!sys_rst_n),
        .O_RST(pll_reset),.PLLLOCK(raw_lock),.O_LOCK(video_lock),.ICPSEL(icpsel),.LPFRES(lpfres));
    hdmi_startup video_startup(sys_clk,sys_rst_n,video_lock,dvi_rst,clock_enable);
    wire timing_rst,display_rst,fb_rst0,fb_rst1;
    wire common_fb_rst=sys_rst_n && video_lock && (&configured);
    reset_sync timing_reset(pixel_clk,sys_rst_n && video_lock,timing_rst);
    reset_sync display_reset(pixel_clk,common_fb_rst,display_rst);
    reset_sync fb_reset0(c0_pclk,common_fb_rst,fb_rst0);
    reset_sync fb_reset1(c1_pclk,common_fb_rst,fb_rst1);
    wire [14:0] addr;wire vblank;
    wire [15:0] pixel0,pixel1,displayed_gray0,displayed_gray1;
    wire [1:0] image_valid;
    dvp_triple_framebuffer buffer0(.pclk(c0_pclk),.cam_rst_n(fb_rst0),.vsync(c0_vsync),.href(c0_href),.data(c0_data),
        .rd_clk(pixel_clk),.rd_rst_n(display_rst),.rd_vblank(vblank),.rd_addr(addr),.rd_data(pixel0),.rd_valid(image_valid[0]),
        .frames(),.bad_frames(),.displayed(),.frames_gray(),.bad_gray(),.displayed_gray(displayed_gray0));
    dvp_triple_framebuffer buffer1(.pclk(c1_pclk),.cam_rst_n(fb_rst1),.vsync(c1_vsync),.href(c1_href),.data(c1_data),
        .rd_clk(pixel_clk),.rd_rst_n(display_rst),.rd_vblank(vblank),.rd_addr(addr),.rd_data(pixel1),.rd_valid(image_valid[1]),
        .frames(),.bad_frames(),.displayed(),.frames_gray(),.bad_gray(),.displayed_gray(displayed_gray1));
    (* ASYNC_REG="TRUE" *) reg [1:0] done_meta,done_sync;
    always @(posedge pixel_clk or negedge timing_rst)
        if(!timing_rst) begin done_meta<=0;done_sync<=0;end
        else begin done_meta<=configured;done_sync<=done_meta;end
    wire hs,vs,de;wire [7:0] red,green,blue;
    stereo_1024x600 scanout(pixel_clk,timing_rst,image_valid,done_sync,pixel0,pixel1,addr,vblank,hs,vs,de,red,green,blue);
    DVI_TX_Top dvi(.I_rst_n(dvi_rst),.I_serial_clk(serial_clk),.I_rgb_clk(pixel_clk),
        .I_rgb_vs(vs),.I_rgb_hs(hs),.I_rgb_de(de),.I_rgb_r(red),.I_rgb_g(green),.I_rgb_b(blue),
        .O_tmds_clk_p(hdmi_clk_p),.O_tmds_clk_n(hdmi_clk_n),.O_tmds_data_p(hdmi_dat_p),.O_tmds_data_n(hdmi_dat_n));
    assign hdmi_oe=1'b1;
    // 仅同步独立状态位；活动位取源域已寄存Gray位，不异步采样多位二进制计数。
    (* ASYNC_REG="TRUE" *) reg [3:0] status_meta,status_sync;
    always @(posedge sys_clk or negedge sys_rst_n)
        if(!sys_rst_n) begin status_meta<=0;status_sync<=0;end
        else begin status_meta<={displayed_gray1[4],displayed_gray0[4],image_valid};status_sync<=status_meta;end
    wire lock_sync;wire [3:0] lock_losses;
    hdmi_lock_monitor lock_monitor(sys_clk,sys_rst_n,video_lock,lock_sync,lock_losses);
    assign status={1'b1,lock_losses,status_sync,lock_sync};
endmodule
