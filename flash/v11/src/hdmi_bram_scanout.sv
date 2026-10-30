// 功能：片内双缓冲与480×854原生扫描，横放旋转由读坐标完成；模块没有网络或DDR接口。
// 参数：固定接收800×480 RGB565 DVP，偶数行列抽样为400×240，再居中放大2倍。
// 接口：cam_rst_n/rd_rst_n由同一显示就绪条件异步置位、各域同步释放；
// timing_rst_n仅控制扫描，允许相机未就绪时显示背景，done表示相机初始化完成。
// 输出：RGB/HS/VS/DE及坐标同拍；valid在消隐接收首个完整帧后有效；
// frames/bad_frames为相机域16位累计值，published为单比特完整帧发布翻转。
module hdmi_bram_scanout(
    input wire pclk,cam_rst_n,vsync,href,input wire [7:0] data,
    input wire pixel_clk,rd_rst_n,timing_rst_n,done,
    output wire hs,vs,de,output wire [7:0] red,green,blue,
    output wire [10:0] scan_x,output wire [9:0] scan_y,
    output wire vblank,valid,published,
    output wire [15:0] frames,bad_frames
);
    wire [8:0] col;
    wire [7:0] row;
    wire [16:0] address=row*17'd400+col;
    wire [15:0] pixel;
    dvp_hdmi_double_buffer buffer(
        .pclk(pclk),.cam_rst_n(cam_rst_n),.vsync(vsync),.href(href),.data(data),
        .rd_clk(pixel_clk),.rd_rst_n(rd_rst_n),.rd_vblank(vblank),.rd_addr(address),
        .rd_data(pixel),.rd_valid(valid),.frames(frames),.bad_frames(bad_frames),
        .displayed(),.displayed_gray());
    assign published=frames[0];
    mono_scanout scan(
        .clk(pixel_clk),.rst_n(timing_rst_n),.valid(valid),.done(done),.pixel(pixel),
        .col(col),.row(row),.vblank(vblank),.fetch_start(),.fetch_row(),.image_active(),
        .hs(hs),.vs(vs),.de(de),.red(red),.green(green),.blue(blue),
        .scan_x(scan_x),.scan_y(scan_y));
endmodule
