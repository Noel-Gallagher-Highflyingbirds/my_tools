rem By carlos_lee 20220314
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\cmake" -S .\ -B .\build

cmake --build .\build --config Release --target ALL_BUILD
copy .\build\Release\pcl_statistical_outlier_removal.exe ..\..\ /y
pause