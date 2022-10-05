rem By carlos_lee 20220313
rem rd /s /q .\build
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="D:\carlos\code\pcl\pcl-pcl-1.10.0\install_110\cmake" -DBoost_INCLUDE_DIR="D:/carlos/graduate_files/lixin/install/PCL 1.10.0/3rdParty/Boost/include/boost-1_72" -DBOOST_LIBRARYDIR="D:/carlos/graduate_files/lixin/install/PCL 1.10.0/3rdParty/Boost/lib" -S .\ -B .\build
del .\build\Release\compare_r_t.exe
cmake --build .\build --config Release --target ALL_BUILD
copy .\build\Release\compare_r_t.exe ..\..\ /y
pause