@echo off
echo ==============================================================
echo RISAbot BPU Model Compilation Tool (RDK X5)
echo ==============================================================
echo.
echo This script compiles your 'best.onnx' model using the Horizon Open Explorer
echo Docker toolchain image (openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8).
echo.
echo Requirements:
echo  1. Docker Desktop must be running.
echo  2. 'best.onnx' (downloaded from Colab) must be in this folder.
echo  3. 'calibration_data/' folder (downloaded from Colab) must be in this folder.
echo.

:: Check for best.onnx
if not exist "best.onnx" (
    echo [ERROR] 'best.onnx' is missing in this folder!
    echo Please download it from Colab and place it here.
    echo.
    pause
    exit /b 1
)

:: Check for calibration_data
if not exist "calibration_data" (
    echo [ERROR] 'calibration_data/' folder is missing in this folder!
    echo Please download it from Colab and extract it here.
    echo.
    pause
    exit /b 1
)

echo.
echo Docker is pulling/verifying the Horizon toolchain image and compiling...
echo Executing command: hb_mapper makertbin --config risabot_bpu_config.yaml --model-type onnx
echo.

docker run --rm -v "%cd%":/workspace -w /workspace openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8 hb_mapper makertbin --config risabot_bpu_config.yaml --model-type onnx

if %ERRORLEVEL% equ 0 (
    echo.
    echo ==============================================================
    echo [SUCCESS] BPU Model Compilation Completed!
    echo ==============================================================
    echo.
    echo Output files are located inside: 'model_output/'
    echo Copy 'risabot_signs_640x640_nv12.bin' to your RDK X5 board at:
    echo '/home/sunrise/risabot_signs_640x640_nv12.bin'
    echo.
) else (
    echo.
    echo ==============================================================
    echo [ERROR] BPU Model Compilation Failed!
    echo ==============================================================
    echo Please review the compiler log messages above for details.
    echo.
)

pause
