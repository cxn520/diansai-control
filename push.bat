@echo off
setlocal

set /p COMMIT_MSG=请输入提交说明:
if "%COMMIT_MSG%"=="" set COMMIT_MSG=update

powershell -ExecutionPolicy Bypass -File "%~dp0push.ps1" "%COMMIT_MSG%"

echo.
pause
