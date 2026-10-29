@echo off
rem Open the project in Ubuntu, not in the Windows shell.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\windows_launch.ps1" -Action install -ProjectPath "%~dp0." %*
set "result=%ERRORLEVEL%"
if not "%result%"=="0" echo Project command failed with exit code %result%.
pause
exit /b %result%
