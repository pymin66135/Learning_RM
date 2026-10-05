@echo off
setlocal
rem Keep this legacy WHEELTEC project in GBK (Keil code page 936).
rem Close other CubeMX windows before launching this project.
set "JAVA_TOOL_OPTIONS=%JAVA_TOOL_OPTIONS% -Dfile.encoding=GBK"
if not exist "D:\STM32Cube\STM32CubeMX.exe" (
  echo STM32CubeMX not found. Update the path in this launcher.
  pause
  exit /b 1
)
start "" /D "D:\STM32Cube" "D:\STM32Cube\STM32CubeMX.exe" -i "%~dp0WHEELTEC.ioc"
endlocal
