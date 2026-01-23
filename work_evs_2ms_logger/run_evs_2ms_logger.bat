@echo off
setlocal EnableDelayedExpansion

set "WORK_DIR=C:\Project\work_evs_2ms_logger"
cd /d "%WORK_DIR%"

call :remove_path "C:\Program Files\Prophesee\bin"
call :remove_path "C:\Program Files\Prophesee\lib\metavision\hal\plugins"
call :remove_path "C:\Program Files\Prophesee\third_party\bin"

set "PATH=C:\Project\build\bin\Release;C:\Project\build\lib\Release;%PATH%"

set "METAVISION_HAL_PLUGIN_PATH=C:\Project\build\lib\metavision\hal\plugins"
set "MV_HAL_PLUGIN_PATH=%METAVISION_HAL_PLUGIN_PATH%"
set "HAL_PLUGIN_PATH=%METAVISION_HAL_PLUGIN_PATH%"
set "MV_HAL_PLUGIN_SEARCH_MODE=PLUGIN_PATH_ONLY"

echo === Runtime DLL resolution (before) ===
where metavision_psee_hw_layer.dll
echo METAVISION_HAL_PLUGIN_PATH=%METAVISION_HAL_PLUGIN_PATH%
echo MV_HAL_PLUGIN_PATH=%MV_HAL_PLUGIN_PATH%
echo HAL_PLUGIN_PATH=%HAL_PLUGIN_PATH%
echo MV_HAL_PLUGIN_SEARCH_MODE=%MV_HAL_PLUGIN_SEARCH_MODE%
echo ======================================

C:\Project\build\bin\Release\evs_2ms_logger.exe

echo === Runtime DLL resolution (after) ===
where metavision_psee_hw_layer.dll
echo METAVISION_HAL_PLUGIN_PATH=%METAVISION_HAL_PLUGIN_PATH%
echo MV_HAL_PLUGIN_PATH=%MV_HAL_PLUGIN_PATH%
echo HAL_PLUGIN_PATH=%HAL_PLUGIN_PATH%
echo MV_HAL_PLUGIN_SEARCH_MODE=%MV_HAL_PLUGIN_SEARCH_MODE%
echo =====================================

exit /b %ERRORLEVEL%

:remove_path
set "PATH=!PATH:%~1;=!"
set "PATH=!PATH:%~1=!"
goto :eof
