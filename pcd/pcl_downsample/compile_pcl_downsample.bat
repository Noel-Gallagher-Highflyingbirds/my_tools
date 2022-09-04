rem By carlos_lee 20220306
rem rd /s /q .\build_r
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="D:\carlos\graduate_files\lixin\install\PCL 1.10.0\cmake" -S .\ -B .\build_r
del .\build_r\Release\pcl_downsample.exe
cmake --build .\build_r --config Release --target ALL_BUILD
copy .\build_r\Release\pcl_downsample.exe ..\..\ /y
pause