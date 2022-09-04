cmake -DOpen3D_ROOT=~/3rd/registration_tool/open3d141_r -S ./source -B ./build
cmake --build ./build --config Release --parallel 4
cp ./build/registration_qt ./bin
