@echo off
setlocal EnableExtensions EnableDelayedExpansion
pushd "%~dp0"

set "REPO_ROOT=%CD%"
set "MV_HAL_PLUGIN_PATH=%REPO_ROOT%\build\lib\metavision\hal\plugins"
set "PATH=%REPO_ROOT%\build\bin\Release;%PATH%"

echo MV_HAL_PLUGIN_PATH=%MV_HAL_PLUGIN_PATH%
echo.
echo [Diagnostics] metavision_hal.dll
where metavision_hal.dll 2>NUL
echo.
echo [Diagnostics] hal_plugin_prophesee.dll
where hal_plugin_prophesee.dll 2>NUL
echo.

if not exist "%REPO_ROOT%\build\bin\Release\capture_image.exe" (
  echo ERROR: capture_image.exe not found at:
  echo   "%REPO_ROOT%\build\bin\Release\capture_image.exe"
  echo Build it first:
  echo   cmake --build build --config Release --target capture_image
  popd
  exit /b 1
)

echo Launching:
echo   "%REPO_ROOT%\build\bin\Release\capture_image.exe"
"%REPO_ROOT%\build\bin\Release\capture_image.exe"

popd
endlocal
