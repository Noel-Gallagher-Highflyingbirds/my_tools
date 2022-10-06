rem By carlos_lee 20220313
rem rd /s /q .\build_r
cmake -DCMAKE_BUILD_TYPE=Release -DPCL_DIR="D:/carlos/graduate_files/lixin/install/PCL 1.10.0/cmake" -S ./ -B ./build
cmake --build ./build --config Release --target ALL_BUILD
copy ./build/Release/line_fit.exe ../../bin /y
pause
