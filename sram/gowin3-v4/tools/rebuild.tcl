# 功能：将原构建入口重定位到本归档，使用已冻结ROM；不访问板卡。
# 参数：无。返回：综合/布局布线失败时沿用原脚本非零退出。
set root [file normalize [file join [file dirname [info script]] ..]]
set handle [open [file join $root build.tcl] r]
set original [read $handle]
close $handle
set rebased [string map [list F:/FPGA/build/soc/task4_red_presence $root] $original]
eval $rebased
