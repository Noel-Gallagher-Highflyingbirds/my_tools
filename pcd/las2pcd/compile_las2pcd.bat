rem By carlos_lee 202206
rem rd /s /q .\build_r
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\cmake" -S .\source -B .\build_r
del .\source\bin\release\las2pcd.exe
cmake --build .\build_r --config Release --target ALL_BUILD
rem .\source\bin\release\las2pcd.exe .\source\ground.las .\pcdpcc.pcd 0
mkdir ..\..\las2pcd\
copy .\source\bin\release\*.* ..\..\las2pcd\ /y

pause