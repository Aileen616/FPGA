param([string]$BoardHost = 'orangepi-wifi')
$ErrorActionPreference = 'Stop'
# 功能：启动电脑地图上位机；BoardHost为可编辑地址；返回Python退出码，不修改板端配置。
$taskRuntimes = @(
    "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe",
    'F:\yolo\.venv\Scripts\python.exe'
)
$taskPython = $taskRuntimes | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (!$taskPython) { throw '未发现项目Python环境，请使用带Tkinter和paramiko的Python运行pc_station/station.py。' }
& $taskPython -X utf8 "$PSScriptRoot\pc_station\station.py" --host $BoardHost
exit $LASTEXITCODE
