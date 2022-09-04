rem By carlos_lee 20220306
rem rd /s /q .\build_r
cmake -DOpen3D_ROOT=../../../../install/open3d141_r -S . -B ./build_r
cmake --build ./build_r --config Release --target ALL_BUILD
copy .\build_r\Release\open3d_icp141.exe ..\..\ /y
pause
rem rd /s /q .\build_r