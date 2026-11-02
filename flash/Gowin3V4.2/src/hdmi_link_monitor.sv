// 功能：跨域只读视频监测，功能复位不清除计数；仅共同控制复位清零。
// 输入：clk为控制时钟，rst_n低有效；四个时钟沿用现有域，不新增时钟源。
// levels[18:0]：帧有效、像素有效、OSD确认/发布、显示/网络溢出、欠载、
// 配置故障/完成、DDR复位/校准、内存/显示/OSD/扫描复位、DVI复位、门控、锁定、原始锁定。
// 输出：words为16个32位状态字；累计值自然回绕，读取快照由OSD总线模块完成。
module hdmi_link_monitor(
    input wire clk,rst_n,pixel_clk,net_clk,mem_clk,
    input wire raw_lock,input wire [18:0] levels,
    input wire hs,vs,de,miss_event,published,
    output wire [511:0] words
);
    wire pixel_rst_n,net_rst_n,mem_rst_n;
    reset_sync pr(pixel_clk,rst_n,pixel_rst_n);
    reset_sync nr(net_clk,rst_n,net_rst_n);
    reset_sync mr(mem_clk,rst_n,mem_rst_n);
    reg [31:0] pc,hc,vc,dc,mc,rc,nc,ac;
    reg [31:0] pg,hg,vg,dg,mg,rg,ng,ag;
    reg h_old,v_old,miss_old;
    // 扫描计数在功能扫描复位后保留，时钟停止时自然停在最后一次值。
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            pc<=0;hc<=0;vc<=0;dc<=0;mc<=0;rc<=0;
            pg<=0;hg<=0;vg<=0;dg<=0;mg<=0;rg<=0;
            h_old<=0;v_old<=0;miss_old<=0;
        end else begin
            pc<=pc+1'b1;h_old<=hs;v_old<=vs;miss_old<=miss_event;
            if(hs&&!h_old)hc<=hc+1'b1;
            if(vs&&!v_old)vc<=vc+1'b1;
            if(de)dc<=dc+1'b1;
            if(miss_event)mc<=mc+1'b1;
            if(miss_event&&!miss_old)rc<=rc+1'b1;
            pg<=(pc>>1)^pc;hg<=(hc>>1)^hc;vg<=(vc>>1)^vc;
            dg<=(dc>>1)^dc;mg<=(mc>>1)^mc;rg<=(rc>>1)^rc;
        end
    end
    always @(posedge net_clk or negedge net_rst_n)
        if(!net_rst_n)begin nc<=0;ng<=0;end
        else begin nc<=nc+1'b1;ng<=(nc>>1)^nc;end
    always @(posedge mem_clk or negedge mem_rst_n)
        if(!mem_rst_n)begin ac<=0;ag<=0;end
        else begin ac<=ac+1'b1;ag<=(ac>>1)^ac;end

    (* ASYNC_REG="TRUE" *) reg [255:0] gray_meta,gray_sync;
    (* ASYNC_REG="TRUE" *) reg [18:0] level_meta,level_sync;
    (* ASYNC_REG="TRUE" *) reg pub_meta,pub_sync,sticky_meta,sticky_sync;
    reg [18:0] previous;
    reg pub_old,armed;
    reg [7:0] arm_delay;
    reg [31:0] raw_losses,qualified_losses,ddr_losses,acks,frames,overflows,cycles;
    reg raw_loss_latch;
    // 原始LOCK低脉冲即使短于控制周期，也异步置位该锁存器；只统计发生过，
    // 不能将它解释成精确脉冲次数。首次稳定启动前清除初始未锁定状态。
    always @(posedge clk or negedge raw_lock)
        if(!raw_lock)raw_loss_latch<=1;
        else if(!rst_n||!armed)raw_loss_latch<=0;

    // 功能：格雷码解码；value为32位同步格雷值；返回对应自然回绕计数。
    function automatic [31:0] ungray(input [31:0] value);
        integer i;
        begin for(i=0;i<32;i=i+1)ungray[i]=^(value>>i);end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            gray_meta<=0;gray_sync<=0;level_meta<=0;level_sync<=0;
            pub_meta<=0;pub_sync<=0;sticky_meta<=0;sticky_sync<=0;
            previous<=0;pub_old<=0;armed<=0;arm_delay<=0;
            raw_losses<=0;qualified_losses<=0;ddr_losses<=0;acks<=0;frames<=0;overflows<=0;cycles<=0;
        end else begin
            gray_meta<={ag,ng,rg,mg,dg,vg,hg,pg};gray_sync<=gray_meta;
            level_meta<=levels;level_sync<=level_meta;
            pub_meta<=published;pub_sync<=pub_meta;pub_old<=pub_sync;
            sticky_meta<=raw_loss_latch;sticky_sync<=sticky_meta;
            previous<=level_sync;cycles<=cycles+1'b1;
            if(!armed)begin
                if(level_sync[0]&&level_sync[1]&&level_sync[8]&&level_sync[10]&&!level_sync[9])begin
                    if(&arm_delay)armed<=1;else arm_delay<=arm_delay+1'b1;
                end else arm_delay<=0;
            end
            if(armed)begin
                if(previous[0]&&!level_sync[0])raw_losses<=raw_losses+1'b1;
                if(previous[1]&&!level_sync[1])qualified_losses<=qualified_losses+1'b1;
                if((previous[8]&&!level_sync[8])||(!previous[9]&&level_sync[9]))ddr_losses<=ddr_losses+1'b1;
            end
            if(previous[16]!=level_sync[16])acks<=acks+1'b1;
            if(pub_old!=pub_sync)frames<=frames+1'b1;
            if((|(level_sync[14:13]&~previous[14:13])))overflows<=overflows+1'b1;
        end
    end
    assign words[0+:32]={sticky_sync&&armed,armed,11'b0,level_sync};
    assign words[32+:32]=raw_losses;
    assign words[64+:32]=qualified_losses;
    assign words[96+:32]=ddr_losses;
    genvar g;
    generate for(g=0;g<8;g=g+1)begin:decode
        assign words[(g+4)*32+:32]=ungray(gray_sync[g*32+:32]);
    end endgenerate
    assign words[384+:32]=acks;
    assign words[416+:32]=frames;
    assign words[448+:32]=overflows;
    assign words[480+:32]=cycles;
endmodule
