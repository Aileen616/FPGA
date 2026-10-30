// 功能：四轮逻辑驱动力映射为TB6612的IN1、IN2、PWM及公共待机许可；不分配物理引脚。
// 来源：用户提供的TB6612FNG数据表第4页；轮序bit0..3为FL、FR、RL、RR。
// 参数：DIR_INVERT每位翻转一个轮子的电气方向，默认0；不能照搬例程的左右轮极性。
// 输入：rst_n低有效复位；drive_allowed全局许可；enable逐轮许可；dir=0为逻辑正向。
// pwm_mag是已带换向空窗的PWM；所有逻辑输入须来自同一时钟域，异步复位可立即撤销许可。
// 输出：驱动时IN1/IN2为10或01，PWM独立调制；禁能时11且PWM=0，形成电气短路制动。
// stby为全局许可，不随单轮PWM或零目标抖动；可连接已确认外引的STBY，低时芯片高阻。
// 四驱参考板未标外接STBY：若板内固定高，禁能仍由11/PWM=0制动，不能称为高阻或已停稳。
module tb6612_adapter #(parameter [3:0] DIR_INVERT=4'b0000)(
    input wire rst_n,drive_allowed,
    input wire [3:0] pwm_mag,dir,enable,
    output wire [3:0] in1,in2,pwm,
    output wire stby
);
    wire [3:0] active={4{rst_n && drive_allowed}} & enable;
    wire [3:0] direction=dir^DIR_INVERT;
    assign stby=rst_n && drive_allowed;
    // 不在方向线上叠加PWM，避免把MDD3A双PWM真值表带入TB6612。
    assign in1=~active | ~direction;
    assign in2=~active | direction;
    assign pwm=active & pwm_mag;
endmodule
