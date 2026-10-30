# FPGA

本仓库的本地目录为 `F:\FPGA_git`，远端为 `git@github.com:Aileen616/FPGA.git`。

首次提交用于验证本机 Git、SSH 和 GitHub 上传环境，包含仓库配置及环境检查脚本。原 `F:\FPGA` 工程未复制到此处。

## 环境检查

在 PowerShell 中执行：

```powershell
& F:\FPGA_git\tools\Test-GitEnvironment.cmd
```

检查远端读取权限时执行：

```powershell
& F:\FPGA_git\tools\Test-GitEnvironment.cmd --check-remote
```

## 日常提交

将需要管理的源码放入本目录，检查变更后按文件添加并提交：

```powershell
cd F:\FPGA_git
git status
git add <需要提交的文件路径>
git diff --cached
git commit -m "说明本次修改"
git push
```

仓库为公开仓库。私钥、密码、令牌和本地连接配置应保留在电脑上；编译输出和工具安装包默认忽略。

Git LFS 已在本地仓库启用；尚未为任何文件类型配置 LFS 跟踪。
