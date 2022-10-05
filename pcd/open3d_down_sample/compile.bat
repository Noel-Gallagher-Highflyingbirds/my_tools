rem By carlos_lee 20220306
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DOpen3D_ROOT=D:/carlos/install/open3d141_r -S .\ -B .\build
cmake --build .\build --config Release
copy .\build\release\open3d_down_sample.exe ..\..\bin /y
pause