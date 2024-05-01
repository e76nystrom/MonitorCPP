@echo off

call :LinkFile "src\stm32Info.cpp" "..\..\..\EclipseCPP\LatheCPP\lathe_src\stm32Info.cpp"

EXIT /B %errorlevel%

:LinkFile
set "link=%~1"
set "file=%~2"
if exist "%link%" (
echo "remove link" "%link%"
rm %link%
)

set /a links+=1
echo  %links% "make link" "%link%" "%file%"
mklink "%link%" "%file%"

EXIT /B %errorlevel%
