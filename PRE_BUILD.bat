:: ENTRY POINT
::
@echo off
@SETLOCAL EnableDelayedExpansion

:: Change active working directory in case we run script for outside of TheForge
cd /D "%~dp0"

set filename=Art.zip
@REM if exist %filename% (
@REM     del %filename%
@REM )

@REM echo Pulling Art Assets
@REM "Tools/wget" -O %filename% http://www.conffx.com/%filename%

echo Unzipping Art Assets...
"Tools/7z" x %filename% -y > NUL

echo Finishing up...
del %filename%

exit /b 0
