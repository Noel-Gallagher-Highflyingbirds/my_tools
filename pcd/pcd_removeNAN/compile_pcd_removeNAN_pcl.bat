rem By carlos_lee 20220313
rem rd /s /q .\build_r
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\cmake" -S .\ -B .\build_r
del .\build_r\Release\pcd_removeNAN.exe
cmake --build .\build_r --config Release --target ALL_BUILD
copy .\build_r\Release\pcd_removeNAN.exe ..\..\ /y
pause