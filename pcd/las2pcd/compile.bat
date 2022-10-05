rem By carlos_lee 202206
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="I:\graduate_files\2019\code_docs\lixin\install\PCL 1.10.0\cmake" -S .\source -B .\build
del .\source\bin\release\las2pcd.exe
cmake --build .\build --config Release --target ALL_BUILD
rem .\source\bin\release\las2pcd.exe .\source\ground.las .\pcdpcc.pcd 0
mkdir ..\..\bin\las2pcd\
copy .\source\bin\release\*.* ..\..\bin\las2pcd\ /y

pause