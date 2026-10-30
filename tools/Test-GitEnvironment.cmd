@echo off
setlocal
rem 检查本仓库的工具、提交身份和远端配置，不修改系统或仓库设置。
rem 可选参数 --check-remote 用于查询远端主分支；成功返回零，失败返回一。
rem 由脚本位置确定仓库路径，支持从其他目录调用。
set "GIT_REPO_ROOT=%~dp0.."

git -C "%GIT_REPO_ROOT%" --version
if errorlevel 1 exit /b 1
git -C "%GIT_REPO_ROOT%" lfs version
if errorlevel 1 exit /b 1
git -C "%GIT_REPO_ROOT%" rev-parse --show-toplevel
if errorlevel 1 exit /b 1
git -C "%GIT_REPO_ROOT%" config --get user.name
if errorlevel 1 exit /b 1
git -C "%GIT_REPO_ROOT%" config --get user.email
if errorlevel 1 exit /b 1
git -C "%GIT_REPO_ROOT%" remote -v
if errorlevel 1 exit /b 1

rem 只读检查实际远端分支，避免将本地配置正常误报为网络验证通过。
if /i "%~1"=="--check-remote" (
    git -C "%GIT_REPO_ROOT%" ls-remote --exit-code origin refs/heads/main
    if errorlevel 1 exit /b 1
)

echo Git environment check passed.
exit /b 0
