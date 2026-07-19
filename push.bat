@echo off
setlocal EnableExtensions

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0push.ps1"
set "PUSH_EXIT=%ERRORLEVEL%"

echo.
if not "%PUSH_EXIT%"=="0" echo Upload failed. Please read the error message above.
pause
exit /b %PUSH_EXIT%
