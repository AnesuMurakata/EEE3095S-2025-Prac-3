@echo off
echo Attempting to flash with STM32CubeProgrammer...
echo.

REM Try to find STM32CubeProgrammer in common locations
set CUBEPROG_PATH=""

if exist "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" (
    set CUBEPROG_PATH="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
) else if exist "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" (
    set CUBEPROG_PATH="C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
) else (
    echo STM32CubeProgrammer not found. Please install it from STMicroelectronics website.
    echo Download from: https://www.st.com/en/development-tools/stm32cubeprog.html
    pause
    exit /b 1
)

echo Using STM32CubeProgrammer at: %CUBEPROG_PATH%
echo.

REM Try to connect and flash
%CUBEPROG_PATH% -c port=SWD -d build\debug\Practical_3A.hex -v -rst

if %ERRORLEVEL% neq 0 (
    echo.
    echo Flash failed. Trying with different parameters...
    echo.
    %CUBEPROG_PATH% -c port=SWD freq=1000 -d build\debug\Practical_3A.hex -v -rst
)

pause

