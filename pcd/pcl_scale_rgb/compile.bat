rem By carlos_lee 20220313
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="D:\carlos\graduate_files\lixin\install\PCL 1.10.0\cmake" -S .\ -B .\build
del .\build\Release\pcl_scale_rgb.exe
cmake --build .\build --config Release --target ALL_BUILD
copy .\build\Release\pcl_scale_rgb.exe ..\..\bin /y
pause