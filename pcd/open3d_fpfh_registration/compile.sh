# by carlos 202207
cmake -DOpen3D_ROOT=~/install/open3d141_r -S . -B ./build_r
cmake --build ./build_r --config Release
cp ./build_r/RegistrationRANSAC141 ../../
