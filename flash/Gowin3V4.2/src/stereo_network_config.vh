// 功能：2026-09-23已通过SSH核对香橙派eth0 MAC及192.168.50.1/24静态配置。
// 重载相同码流前必须重启接收服务，清除帧去重状态；会话号按部署构建更新。
`define BXV_DST_MAC 48'h0000a406ca07
`define BXV_SRC_IP 32'hc0a83202
`define BXV_DST_IP 32'hc0a83201
`define BXV_STREAM_ID 32'h26092301
