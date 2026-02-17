@echo off
set COMMANDER=C:\Users\mjeuw\.silabs\slt\installs\archive\Simplicity Commander\commander.exe
set FIRMWARE=C:\Users\mjeuw\SimplicityStudio\TEST\wifi_gspi_merged\cmake_gcc\build\base\wifi_gspi_merged.rps

echo Flashing firmware...
"%COMMANDER%" flash "%FIRMWARE%" --device Si917 --serialno 440189400
echo Exit code: %ERRORLEVEL%
pause
