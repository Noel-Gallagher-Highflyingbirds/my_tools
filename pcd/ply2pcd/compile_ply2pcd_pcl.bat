rem By carlos_lee 20220313
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="D:\carlos\graduate_files\lixin\install\PCL 1.10.0\cmake" -S .\ -B .\build
del .\build\Release\ply2pcd.exe
cmake --build .\build --config Release --target ALL_BUILD
copy .\build\Release\ply2pcd.exe ..\..\bin /y
pause