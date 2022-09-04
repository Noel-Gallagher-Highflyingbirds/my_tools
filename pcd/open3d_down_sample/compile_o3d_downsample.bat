rem By carlos_lee 20220306
rem rd /s /q .\build_r
cmake -DCMAKE_BUILD_TYPE=Release -DOpen3D_ROOT=D:/carlos/install/open3d141_r -S .\ -B .\build_r
cmake --build .\build_r --config Release
copy .\build_r\release\open3d_down_sample.exe ..\..\ /y
pause