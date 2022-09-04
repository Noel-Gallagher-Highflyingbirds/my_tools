cmake -DQt5_DIR="D:\carlos\install\Qt\Qt5.14.2\5.14.2\msvc2017_64\lib\cmake\Qt5" -DOpen3D_ROOT="D:/carlos/install/open3d141_r/" -S ./source -B ./build
cmake --build ./build --config Release
copy .\build\Release\registration_qt.exe .\bin /y
pause
