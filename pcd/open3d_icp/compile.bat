rem By carlos_lee 20220306
rem rd /s /q .\build
cmake -DOpen3D_ROOT=../../../../install/open3d141_r -S . -B ./build
cmake --build ./build --config Release --target ALL_BUILD
copy .\build\Release\open3d_icp141.exe ..\..\bin /y
pause
rem rd /s /q .\build